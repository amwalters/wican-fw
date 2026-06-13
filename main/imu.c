/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#include "imu.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "hw_config.h"
#include "wc_timer.h"
#include "rtc.h"
#include "dev_status.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

#define TAG "imu"
#define STATIONARY_TIME_MS      3000

static icm42670_t dev = {0};
static QueueHandle_t imu_motion_evt_queue = NULL;
static QueueHandle_t activity_state_queue = NULL;
static imu_wom_settings_t imu_wom_settings;
static bool imu_initialized = false;
static volatile uint8_t imu_last_int_status2 = 0;
static volatile uint32_t imu_wom_x_count = 0;
static volatile uint32_t imu_wom_y_count = 0;
static volatile uint32_t imu_wom_z_count = 0;
static volatile uint32_t imu_last_active_ms = 0;
// Static storage for imu_motion_evt_queue (length 10, item size uint32_t)
static StaticQueue_t imu_motion_evt_queue_struct;
static uint8_t imu_motion_evt_queue_storage[10 * sizeof(uint32_t)];
// Static storage for activity_state_queue (length 1, item size activity_state_t)
static StaticQueue_t activity_state_queue_struct;
static uint8_t activity_state_queue_storage[1 * sizeof(activity_state_t)];

void imu_default_wom_settings(imu_wom_settings_t *settings)
{
    if (settings == NULL) {
        return;
    }

    settings->threshold = 8;
    settings->wom_x_enabled = true;
    settings->wom_y_enabled = true;
    settings->wom_z_enabled = false;
    settings->smd_enabled = false;
    settings->accel_odr = ICM42670_ACCEL_ODR_1_5625HZ;
    settings->accel_avg = ICM42670_ACCEL_AVG_32X;
    settings->wom_int_dur = ICM42670_WOM_INT_DUR_FOURTH;
    settings->wom_int_mode = ICM42670_WOM_INT_MODE_ALL_OR;
    settings->wom_ref_mode = ICM42670_WOM_MODE_REF_LAST;
}

void IRAM_ATTR imu_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(imu_motion_evt_queue, &gpio_num, NULL);
}

static void imu_motion_task(void *pvParameters)
{
    uint32_t gpio_num;
    activity_state_t current_state = ACTIVITY_STATE_STATIONARY;
    activity_state_t last_reported_state = ACTIVITY_STATE_STATIONARY;

    // Get configurable WoM threshold from configuration
    // The threshold value is an 8-bit value where 1 LSB = 1000mg / 256 for ICM-42670-P
    // Range: 0-255, where each unit represents about 3.9mg of acceleration
    imu_wom_settings_t settings = imu_wom_settings;
    ESP_LOGI(TAG, "Using IMU threshold: %d (%.1fmg)", settings.threshold, settings.threshold * 1000.0f / 256.0f);
    ESP_LOGI(TAG, "Using IMU WOM sources: x=%d y=%d z=%d smd=%d",
        settings.wom_x_enabled, settings.wom_y_enabled, settings.wom_z_enabled, settings.smd_enabled);
    ESP_LOGI(TAG, "Using IMU WOM config: odr=%d avg=%d dur=%d mode=%d ref=%d",
        settings.accel_odr, settings.accel_avg, settings.wom_int_dur, settings.wom_int_mode, settings.wom_ref_mode);

    esp_err_t ret = icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_DISABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable accel before WoM config");
        return;
    }

    ret = imu_config_wom(&settings);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure WoM");
        return;
    }

    ret = imu_enable_wom(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable WoM");
        return;
    }

    ESP_LOGI(TAG, "Wake on Motion enabled - Move the device to trigger interrupt");

    // Initialize timer for stationary detection
    wc_timer_t motion_timer;
    wc_timer_set(&motion_timer, 0);  // Initialize to expired


    activity_state_queue = xQueueCreateStatic(1, sizeof(activity_state_t), activity_state_queue_storage, &activity_state_queue_struct);
    if (activity_state_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create activity state queue");
        vTaskDelete(NULL);
        return;
    }

    xQueueOverwrite(activity_state_queue, &current_state);

    while (1) 
    {
        dev_status_wait_for_bits(DEV_AWAKE_BIT, portMAX_DELAY);
        // Wait for WoM interrupt event
        if (xQueueReceive(imu_motion_evt_queue, &gpio_num, pdMS_TO_TICKS(100)))
        {
            uint8_t status2 = 0;
            ret = icm42670_read_int_status2(&dev, &status2);
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to read INT_STATUS2: %s", esp_err_to_name(ret));
            }
            else if (status2 & (ICM42670_SMD_INT_BITS | ICM42670_WOM_X_INT_BITS | ICM42670_WOM_Y_INT_BITS | ICM42670_WOM_Z_INT_BITS))
            {
                imu_last_int_status2 = status2;
                imu_last_active_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

                if (status2 & ICM42670_WOM_X_INT_BITS)
                    imu_wom_x_count++;
                if (status2 & ICM42670_WOM_Y_INT_BITS)
                    imu_wom_y_count++;
                if (status2 & ICM42670_WOM_Z_INT_BITS)
                    imu_wom_z_count++;

                // Motion detected by SMD/WoM
                wc_timer_set(&motion_timer, STATIONARY_TIME_MS);

                // If we were stationary, change to active
                if (current_state == ACTIVITY_STATE_STATIONARY)
                {
                    current_state = ACTIVITY_STATE_ACTIVE;
                    ESP_LOGI(TAG, "State changed to ACTIVE");
                }
            }
            else
            {
                imu_last_int_status2 = status2;
                ESP_LOGD(TAG, "Ignoring IMU interrupt without WoM status: gpio=%lu status2=0x%02x",
                    (unsigned long)gpio_num, status2);
            }
        }

        // Check for transition to stationary state
        if (current_state == ACTIVITY_STATE_ACTIVE)
        {
            if (wc_timer_is_expired(&motion_timer))
            {
                current_state = ACTIVITY_STATE_STATIONARY;
                imu_last_int_status2 = 0;
                ESP_LOGI(TAG, "State changed to STATIONARY");
            }
        }

        // Report state change if it has changed
        if (current_state != last_reported_state)
        {
            // Overwrite the queue with new state
            xQueueOverwrite(activity_state_queue, &current_state);
            last_reported_state = current_state;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

activity_state_t imu_get_activity_state(void)
{
    activity_state_t state;

    if (activity_state_queue == NULL) {
        return ACTIVITY_STATE_INVALID;
    }

    // Peek at the current state without removing it from queue
    if (xQueuePeek(activity_state_queue, &state, 0) != pdTRUE) {
        return ACTIVITY_STATE_INVALID;
    }

    return state;
}

uint8_t imu_get_last_int_status2(void)
{
    return imu_last_int_status2;
}

uint32_t imu_get_wom_x_count(void)
{
    return imu_wom_x_count;
}

uint32_t imu_get_wom_y_count(void)
{
    return imu_wom_y_count;
}

uint32_t imu_get_wom_z_count(void)
{
    return imu_wom_z_count;
}

uint32_t imu_get_last_active_ms(void)
{
    return imu_last_active_ms;
}

//TODO: scl_gpio, scl_gpio amd int_gpio are not used
esp_err_t imu_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio, gpio_num_t int_gpio, const imu_wom_settings_t *settings)
{
    esp_err_t ret;

    imu_default_wom_settings(&imu_wom_settings);
    if (settings != NULL) {
        imu_wom_settings = *settings;
    }

    // Configure GPIO for IMU interrupt
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<IMU_INT_GPIO_NUM),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    
    gpio_config(&io_conf);

    imu_motion_evt_queue = xQueueCreateStatic(10, sizeof(uint32_t), imu_motion_evt_queue_storage, &imu_motion_evt_queue_struct);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(IMU_INT_GPIO_NUM, imu_isr_handler, (void *)IMU_INT_GPIO_NUM);

    // Initialize ICM42670 device
    ret = icm42670_init_desc(&dev, ICM42670_I2C_ADDR_GND, i2c_num, 
                            sda_gpio, scl_gpio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init device descriptor");
        return ret;
    }

    // Initialize the sensor
    ret = icm42670_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init device");
        return ret;
    }
    imu_initialized = true;

    // Configure default settings
    ret = icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro FSR");
        return ret;
    }

    ret = icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel FSR");
        return ret;
    }

    // Keep sensors off until WoM is configured.
    ret = icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_DISABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro power mode");
        return ret;
    }

    ret = icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_DISABLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel power mode");
        return ret;
    }

    ESP_LOGI(TAG, "IMU initialized successfully");

    // Allocate stack memory in PSRAM for the IMU motion task
    static StackType_t *imu_motion_task_stack;
    static StaticTask_t imu_motion_task_buffer;
    
    imu_motion_task_stack = heap_caps_malloc(1024*3, MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    
    if (imu_motion_task_stack == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate IMU motion task stack memory");
        return ESP_FAIL;
    }
    
    // Create static task
    TaskHandle_t imu_task_handle = xTaskCreateStatic(
        imu_motion_task,
        "imu_motion_task",
        1024*3,
        NULL,
        5,
        imu_motion_task_stack,
        &imu_motion_task_buffer
    );
    
    if (imu_task_handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create IMU motion task");
        heap_caps_free(imu_motion_task_stack);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "IMU motion task created successfully");
    return ESP_OK;
}

esp_err_t imu_config_wom(const imu_wom_settings_t *settings)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const icm42670_int_config_t int_config = {
        .mode = ICM42670_INT_MODE_PULSED,
        .drive = ICM42670_INT_DRIVE_PUSH_PULL,
        .polarity = ICM42670_INT_POLARITY_ACTIVE_HIGH,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(icm42670_config_int_pin(&dev, 1, int_config));

    icm42670_int_source_t sources = {0};
    sources.wom_x = settings->wom_x_enabled;
    sources.wom_y = settings->wom_y_enabled;
    sources.wom_z = settings->wom_z_enabled;
    sources.smd = settings->smd_enabled;

    ESP_ERROR_CHECK_WITHOUT_ABORT(icm42670_set_int_sources(&dev, 1, sources));

    const icm42670_wom_config_t wom_config = {
        .trigger = settings->wom_int_dur,
        .logical_mode = settings->wom_int_mode,
        .reference = settings->wom_ref_mode,
        .wom_x_threshold = settings->threshold,
        .wom_y_threshold = settings->threshold,
        .wom_z_threshold = settings->threshold,
    };

    return icm42670_config_wom(&dev, wom_config);
}

esp_err_t imu_enable_wom(bool enable)
{
    esp_err_t ret = ESP_OK;

    if (enable) {
        ret = icm42670_set_accel_odr(&dev, imu_wom_settings.accel_odr);
        if (ret != ESP_OK) return ret;

        ret = icm42670_set_accel_avg(&dev, imu_wom_settings.accel_avg);
        if (ret != ESP_OK) return ret;

        ret = icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_DISABLE);
        if (ret != ESP_OK) return ret;

        ret = icm42670_set_low_power_clock(&dev, ICM42670_LP_CLK_WUO);
        if (ret != ESP_OK) return ret;

        ret = icm42670_enable_wom(&dev, true);
        if (ret != ESP_OK) return ret;

        ret = icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LP_MODE);
        if (ret != ESP_OK) return ret;

        return ESP_OK;
    }

    ret = icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_DISABLE);
    if (ret != ESP_OK) return ret;

    return icm42670_enable_wom(&dev, false);
}

esp_err_t imu_read_register_state(imu_register_state_t *state)
{
    if (state == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (!imu_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(state, 0, sizeof(*state));
    state->configured_settings = imu_wom_settings;
    state->int_status2_cached = imu_get_last_int_status2();
    state->wom_x_count = imu_get_wom_x_count();
    state->wom_y_count = imu_get_wom_y_count();
    state->wom_z_count = imu_get_wom_z_count();
    state->last_active_ms = imu_get_last_active_ms();
    state->activity_state = imu_get_activity_state();

    esp_err_t ret = icm42670_read_register(&dev, ICM42670_REG_WHO_AM_I, &state->who_am_i);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_MCLK_RDY, &state->mclk_rdy);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_PWR_MGMT0, &state->pwr_mgmt0);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_INT_CONFIG, &state->int_config);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_INT_SOURCE1, &state->int_source1);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_WOM_CONFIG, &state->wom_config);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_ACCEL_CONFIG0, &state->accel_config0);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_ACCEL_CONFIG1, &state->accel_config1);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_register(&dev, ICM42670_REG_APEX_CONFIG1, &state->apex_config1);
    if (ret != ESP_OK) return ret;

    ret = icm42670_read_mreg_register(&dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_X_THR, &state->accel_wom_x_thr);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_mreg_register(&dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Y_THR, &state->accel_wom_y_thr);
    if (ret != ESP_OK) return ret;
    ret = icm42670_read_mreg_register(&dev, ICM42670_MREG1_RW, ICM42670_REG_ACCEL_WOM_Z_THR, &state->accel_wom_z_thr);
    if (ret != ESP_OK) return ret;

    state->accel_valid = (imu_read_accel(&state->accel_x, &state->accel_y, &state->accel_z) == ESP_OK);

    return ESP_OK;
}

esp_err_t imu_read_accel(float *ax, float *ay, float *az)
{
    int16_t raw_x, raw_y, raw_z;
    esp_err_t ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_X1, &raw_x);
    if (ret != ESP_OK) return ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Y1, &raw_y);
    if (ret != ESP_OK) return ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_ACCEL_DATA_Z1, &raw_z);
    if (ret != ESP_OK) return ret;

    // Convert to g's (assuming ±16g range)
    const float scale = 16.0f / 32768.0f;
    *ax = raw_x * scale;
    *ay = raw_y * scale;
    *az = raw_z * scale;

    return ESP_OK;
}

esp_err_t imu_read_gyro(float *gx, float *gy, float *gz)
{
    int16_t raw_x, raw_y, raw_z;
    esp_err_t ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_X1, &raw_x);
    if (ret != ESP_OK) return ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Y1, &raw_y);
    if (ret != ESP_OK) return ret;

    ret = icm42670_read_raw_data(&dev, ICM42670_REG_GYRO_DATA_Z1, &raw_z);
    if (ret != ESP_OK) return ret;

    // Convert to degrees/sec (assuming ±2000 dps range)
    const float scale = 2000.0f / 32768.0f;
    *gx = raw_x * scale;
    *gy = raw_y * scale;
    *gz = raw_z * scale;

    return ESP_OK;
}

esp_err_t imu_read_temp(float *temp)
{
    return icm42670_read_temperature(&dev, temp);
}

esp_err_t imu_get_device_id(uint8_t *id)
{
    uint16_t value;
    esp_err_t ret = icm42670_read_raw_data(&dev, ICM42670_REG_WHO_AM_I, (int16_t*)&value);
    
    if (ret == ESP_OK) {
        *id = (value>>8) & 0xFF;
    }
    return ret;
}

esp_err_t imu_set_accel_fsr(icm42670_accel_fsr_t fsr)
{
    return icm42670_set_accel_fsr(&dev, fsr);
}

esp_err_t imu_set_gyro_fsr(icm42670_gyro_fsr_t fsr)
{
    return icm42670_set_gyro_fsr(&dev, fsr);
}
