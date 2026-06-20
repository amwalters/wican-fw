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

#include "power_detection.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "elm327.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "ble.h"
#include "dev_status.h"
#include "hw_config.h"
#include "imu.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sleep_mode.h"
#include "wc_timer.h"

#define TAG "POWER_DETECTION"
#define POWER_DETECTION_IMU_STATIONARY_HOLD_MS (30 * 1000)
#define POWER_DETECTION_LOAD_TEST_DURATION_MS (60 * 1000)
#define POWER_DETECTION_LOAD_TEST_TARGET_IP "192.168.88.5"
#define POWER_DETECTION_LOAD_TEST_TARGET_PORT 65432
#define POWER_DETECTION_LOAD_TEST_STACK_WORDS 4096
#define POWER_DETECTION_LOAD_TEST_SAMPLE_INTERVAL_MS (2 * 1000)
#define POWER_DETECTION_LOAD_TEST_WINDOW_MS (10 * 1000)
#define POWER_DETECTION_LOAD_TEST_POST_WINDOW_COUNT 6
#define POWER_DETECTION_LOAD_TEST_SAMPLES_PER_WINDOW \
    (POWER_DETECTION_LOAD_TEST_WINDOW_MS / POWER_DETECTION_LOAD_TEST_SAMPLE_INTERVAL_MS)
#define POWER_DETECTION_LOAD_TEST_LOAD_WINDOW_COUNT \
    (POWER_DETECTION_LOAD_TEST_DURATION_MS / POWER_DETECTION_LOAD_TEST_WINDOW_MS)
#define POWER_DETECTION_LOAD_TEST_MAX_WINDOW_COUNT \
    (1 + POWER_DETECTION_LOAD_TEST_LOAD_WINDOW_COUNT + POWER_DETECTION_LOAD_TEST_POST_WINDOW_COUNT)
#define POWER_DETECTION_LOAD_TEST_LABEL_LEN 16
#define POWER_DETECTION_LOAD_TEST_ADC_SAMPLES 20
#define POWER_DETECTION_VOLTAGE_RISE_DELTA_V 0.20f
#define POWER_DETECTION_VOLTAGE_RISE_DEFAULT_TIME_SECONDS 20
#define POWER_DETECTION_VOLTAGE_RISE_HOLD_MS (60 * 1000)
#define POWER_DETECTION_VOLTAGE_RISE_MIN_SAMPLES 3
#define POWER_DETECTION_VOLTAGE_HISTORY_SIZE 32
#define LOAD_PACKET_SIZE 1024

typedef struct
{
    int64_t timestamp_ms;
    float voltage;
} voltage_history_sample_t;

typedef struct
{
    char label[POWER_DETECTION_LOAD_TEST_LABEL_LEN];
    uint32_t sample_count;
    int avg_median_raw;
} load_test_window_result_t;

typedef struct
{
    bool active;
    bool complete;
    bool has_last_adc;
    adc_stats_t last_adc;
    uint32_t window_count;
    load_test_window_result_t windows[POWER_DETECTION_LOAD_TEST_MAX_WINDOW_COUNT];
} load_test_status_t;

typedef enum
{
    SUPPLY_TRIGGER_NONE = 0,
    SUPPLY_TRIGGER_BOOT,
    SUPPLY_TRIGGER_IMU,
    SUPPLY_TRIGGER_VOLTAGE_RISE,
} supply_trigger_t;

static bool motion_was_active = false;
static wc_timer_t imu_stationary_hold_timer = 0;
static volatile bool load_test_active = false;
static TaskHandle_t load_test_task_handle = NULL;
static TaskHandle_t load_test_udp_task_handle = NULL;
static StaticTask_t load_test_task_buffer;
static StaticTask_t load_test_udp_task_buffer;
static StackType_t load_test_task_stack[POWER_DETECTION_LOAD_TEST_STACK_WORDS];
static StackType_t load_test_udp_task_stack[POWER_DETECTION_LOAD_TEST_STACK_WORDS];
static portMUX_TYPE load_test_status_lock = portMUX_INITIALIZER_UNLOCKED;
static load_test_status_t load_test_status = {0};
static voltage_history_sample_t voltage_history[POWER_DETECTION_VOLTAGE_HISTORY_SIZE] = {0};
static uint8_t voltage_history_next = 0;
static uint8_t voltage_history_count = 0;
static int64_t voltage_history_last_sample_ms = 0;
static wc_timer_t voltage_rising_hold_timer = 0;
static supply_trigger_t supply_trigger = SUPPLY_TRIGGER_NONE;
static power_detection_pid_polling_state_t pid_polling_state = {
    .paused = false,
    .reason = NULL,
    .voltage = NAN,
};

static void load_test_reset_status(void)
{
    taskENTER_CRITICAL(&load_test_status_lock);
    memset(&load_test_status, 0, sizeof(load_test_status));
    load_test_status.active = true;
    load_test_status.complete = false;
    taskEXIT_CRITICAL(&load_test_status_lock);
}

static void load_test_update_last_adc(const adc_stats_t *stats)
{
    taskENTER_CRITICAL(&load_test_status_lock);
    load_test_status.last_adc = *stats;
    load_test_status.has_last_adc = true;
    taskEXIT_CRITICAL(&load_test_status_lock);
}

static void load_test_record_window(const char *label, uint32_t sample_count, int avg_median_raw)
{
    taskENTER_CRITICAL(&load_test_status_lock);
    if (load_test_status.window_count < POWER_DETECTION_LOAD_TEST_MAX_WINDOW_COUNT)
    {
        load_test_window_result_t *window = &load_test_status.windows[load_test_status.window_count];
        memset(window, 0, sizeof(*window));
        strncpy(window->label, label, sizeof(window->label) - 1);
        window->sample_count = sample_count;
        window->avg_median_raw = avg_median_raw;
        load_test_status.window_count++;
    }
    taskEXIT_CRITICAL(&load_test_status_lock);
}

static void load_test_finish_status(bool complete)
{
    taskENTER_CRITICAL(&load_test_status_lock);
    load_test_status.active = false;
    load_test_status.complete = complete;
    taskEXIT_CRITICAL(&load_test_status_lock);
}

static esp_err_t load_test_read_adc_stats(adc_stats_t *stats)
{
#if HARDWARE_VER == WICAN_PRO
    return read_ss_adc_raw_stats(stats, POWER_DETECTION_LOAD_TEST_ADC_SAMPLES);
#else
    (void)stats;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void load_test_udp_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Starting UDP load for %d seconds to %s:%d",
             POWER_DETECTION_LOAD_TEST_DURATION_MS / 1000,
             POWER_DETECTION_LOAD_TEST_TARGET_IP,
             POWER_DETECTION_LOAD_TEST_TARGET_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        load_test_udp_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    uint8_t heartbeat_packet[LOAD_PACKET_SIZE];
    memset(heartbeat_packet, '7', sizeof(heartbeat_packet));

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(POWER_DETECTION_LOAD_TEST_TARGET_PORT),
    };
    if (inet_pton(AF_INET, POWER_DETECTION_LOAD_TEST_TARGET_IP, &dest_addr.sin_addr.s_addr) != 1)
    {
        ESP_LOGE(TAG, "Invalid load-test UDP target: %s", POWER_DETECTION_LOAD_TEST_TARGET_IP);
        close(sock);
        load_test_udp_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (POWER_DETECTION_LOAD_TEST_DURATION_MS * 1000LL))
    {
        sendto(sock,
               heartbeat_packet,
               sizeof(heartbeat_packet),
               0,
               (struct sockaddr *)&dest_addr,
               sizeof(dest_addr));

        // vTaskDelay(pdMS_TO_TICKS(1));
    }

    close(sock);
    ESP_LOGI(TAG, "UDP load finished");

    load_test_udp_task_handle = NULL;
    vTaskDelete(NULL);
}

static esp_err_t load_test_start_udp_task(void)
{
    if (load_test_udp_task_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    load_test_udp_task_handle = xTaskCreateStatic(load_test_udp_task,
                                                  "pd_load_udp",
                                                  POWER_DETECTION_LOAD_TEST_STACK_WORDS,
                                                  NULL,
                                                  5,
                                                  load_test_udp_task_stack,
                                                  &load_test_udp_task_buffer);
    if (load_test_udp_task_handle == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void load_test_collect_window(const char *label)
{
    int64_t median_raw_sum = 0;
    uint32_t sample_count = 0;

    for (uint32_t i = 0; i < POWER_DETECTION_LOAD_TEST_SAMPLES_PER_WINDOW; i++)
    {
        adc_stats_t stats = {0};
        esp_err_t ret = load_test_read_adc_stats(&stats);
        if (ret == ESP_OK)
        {
            median_raw_sum += stats.median_raw;
            sample_count++;
            load_test_update_last_adc(&stats);
        }
        else
        {
            ESP_LOGW(TAG, "Load-test ADC read failed in %s: %s", label, esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(POWER_DETECTION_LOAD_TEST_SAMPLE_INTERVAL_MS));
    }

    int avg_median_raw = 0;
    if (sample_count > 0)
    {
        avg_median_raw = (int)(median_raw_sum / sample_count);
    }

    load_test_record_window(label, sample_count, avg_median_raw);
    ESP_LOGI(TAG, "Load-test window %s: avg median raw=%d from %lu samples",
             label, avg_median_raw, (unsigned long)sample_count);
}

static void load_test_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Starting load test: %ds baseline, %ds WiFi TX, %ds post",
             POWER_DETECTION_LOAD_TEST_WINDOW_MS / 1000,
             POWER_DETECTION_LOAD_TEST_DURATION_MS / 1000,
             (POWER_DETECTION_LOAD_TEST_POST_WINDOW_COUNT * POWER_DETECTION_LOAD_TEST_WINDOW_MS) / 1000);

    load_test_collect_window("baseline");

    esp_err_t udp_ret = load_test_start_udp_task();
    if (udp_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start UDP load task: %s", esp_err_to_name(udp_ret));
    }

    for (uint32_t i = 0; i < POWER_DETECTION_LOAD_TEST_LOAD_WINDOW_COUNT; i++)
    {
        char label[POWER_DETECTION_LOAD_TEST_LABEL_LEN];
        snprintf(label, sizeof(label), "load_%lu", (unsigned long)(i + 1));
        load_test_collect_window(label);
    }

    while (load_test_udp_task_handle != NULL)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (uint32_t i = 0; i < POWER_DETECTION_LOAD_TEST_POST_WINDOW_COUNT; i++)
    {
        char label[POWER_DETECTION_LOAD_TEST_LABEL_LEN];
        snprintf(label, sizeof(label), "post_%lu", (unsigned long)(i + 1));
        load_test_collect_window(label);
    }

    ESP_LOGI(TAG, "Load test finished");

    load_test_finish_status(true);
    load_test_active = false;
    load_test_task_handle = NULL;
    vTaskDelete(NULL);
}

static bool is_12v_supply_mode(const autopid_config_t *config)
{
    if (!config ||
        !config->supply_mode_enabled ||
        !config->supply_mode_pid_name ||
        config->supply_mode_pid_name[0] == '\0' ||
        config->supply_mode_operator[0] == '\0')
    {
        return false;
    }

    return autopid_supply_mode_is_active();
}

bool power_detection_is_boot_pid_polling_keep_alive_active(const autopid_config_t *config)
{
    if (!config || config->boot_pid_polling_keep_alive_seconds == 0)
    {
        return false;
    }

    int64_t uptime_us = esp_timer_get_time();
    int64_t keep_alive_us = (int64_t)config->boot_pid_polling_keep_alive_seconds * 1000000LL;
    return uptime_us < keep_alive_us;
}

static uint32_t voltage_rise_time_seconds_or_default(uint32_t rise_time_seconds)
{
    return rise_time_seconds > 0
        ? rise_time_seconds
        : POWER_DETECTION_VOLTAGE_RISE_DEFAULT_TIME_SECONDS;
}

static int64_t voltage_rise_sample_interval_ms(uint32_t rise_time_seconds)
{
    uint32_t seconds = voltage_rise_time_seconds_or_default(rise_time_seconds);
    int64_t interval_ms = ((int64_t)seconds * 1000LL) / 10LL;
    return interval_ms > 0 ? interval_ms : 1;
}

static void voltage_history_record(float voltage, int64_t now_ms, int64_t sample_interval_ms)
{
    if (isnan(voltage))
    {
        return;
    }

    if (voltage_history_count > 0 &&
        (now_ms - voltage_history_last_sample_ms) < sample_interval_ms)
    {
        return;
    }

    voltage_history[voltage_history_next].timestamp_ms = now_ms;
    voltage_history[voltage_history_next].voltage = voltage;
    voltage_history_next = (voltage_history_next + 1) % POWER_DETECTION_VOLTAGE_HISTORY_SIZE;
    if (voltage_history_count < POWER_DETECTION_VOLTAGE_HISTORY_SIZE)
    {
        voltage_history_count++;
    }
    voltage_history_last_sample_ms = now_ms;
}

static void sort_float_values(float *values, uint8_t count)
{
    for (uint8_t i = 1; i < count; i++)
    {
        float value = values[i];
        uint8_t j = i;

        while (j > 0 && values[j - 1] > value)
        {
            values[j] = values[j - 1];
            j--;
        }

        values[j] = value;
    }
}

static bool voltage_history_window_median(int64_t now_ms, int64_t min_age_ms, int64_t max_age_ms, float *median, uint8_t *sample_count)
{
    float values[POWER_DETECTION_VOLTAGE_HISTORY_SIZE];
    uint8_t count = 0;

    for (uint8_t i = 0; i < voltage_history_count; i++)
    {
        int64_t age_ms = now_ms - voltage_history[i].timestamp_ms;
        if (age_ms >= min_age_ms && age_ms <= max_age_ms && !isnan(voltage_history[i].voltage))
        {
            values[count++] = voltage_history[i].voltage;
        }
    }

    if (sample_count)
    {
        *sample_count = count;
    }

    if (count < POWER_DETECTION_VOLTAGE_RISE_MIN_SAMPLES)
    {
        return false;
    }

    sort_float_values(values, count);

    if ((count % 2) == 0)
    {
        *median = (values[(count / 2) - 1] + values[count / 2]) / 2.0f;
    }
    else
    {
        *median = values[count / 2];
    }

    return true;
}

static bool voltage_rising_bypass_active(int64_t now_ms, float threshold_v, uint32_t rise_time_seconds)
{
    if (voltage_rising_hold_timer != 0 && !wc_timer_is_expired(&voltage_rising_hold_timer))
    {
        return true;
    }

    uint32_t seconds = voltage_rise_time_seconds_or_default(rise_time_seconds);
    int64_t current_max_age_ms = ((int64_t)seconds * 1000LL) / 2LL;
    int64_t baseline_min_age_ms = (int64_t)seconds * 1000LL;
    int64_t baseline_max_age_ms = ((int64_t)seconds * 3LL * 1000LL) / 2LL;
    float current_median = NAN;
    float baseline_median = NAN;
    uint8_t current_samples = 0;
    uint8_t baseline_samples = 0;

    bool have_current = voltage_history_window_median(now_ms,
                                                      0,
                                                      current_max_age_ms,
                                                      &current_median,
                                                      &current_samples);
    bool have_baseline = voltage_history_window_median(now_ms,
                                                       baseline_min_age_ms,
                                                       baseline_max_age_ms,
                                                       &baseline_median,
                                                       &baseline_samples);

    if (!have_current || !have_baseline)
    {
        return false;
    }

    float delta = current_median - baseline_median;
    if (delta > threshold_v)
    {
        uint64_t hold_ms = POWER_DETECTION_VOLTAGE_RISE_HOLD_MS;
        wc_timer_set(&voltage_rising_hold_timer, hold_ms);
        ESP_LOGI(TAG,
                 "Voltage rising bypass: current %.2fV (%u samples) baseline %.2fV (%u samples), delta %.2fV threshold %.2fV over %lus hold %lus",
                 current_median,
                 (unsigned int)current_samples,
                 baseline_median,
                 (unsigned int)baseline_samples,
                 delta,
                 threshold_v,
                 (unsigned long)seconds,
                 (unsigned long)(hold_ms / 1000ULL));
        return true;
    }

    return false;
}

static bool evaluate_pid_polling_pause(const autopid_config_t *config, float *out_voltage, const char **out_reason)
{
    if (out_voltage)
        *out_voltage = NAN;
    if (out_reason)
        *out_reason = NULL;

    if (!config)
    {
        if (out_reason)
            *out_reason = "not_configured";
        return false;
    }

    bool boot_keep_alive_active = power_detection_is_boot_pid_polling_keep_alive_active(config);

 //   if (dev_status_is_autopid_wake_bypass_low_voltage())
 //   {
 //       if (out_reason)
 //           *out_reason = "wakeup_poll";
 //       return false;
 //   }

    if (is_12v_supply_mode(config))
    {
        float v = NAN;
        bool above_automate_threshold = false;

        if (config->disable_pid_requests_on_automate_threshold &&
            sleep_mode_get_voltage(&v) == ESP_OK)
        {
            if (out_voltage)
                *out_voltage = v;

            above_automate_threshold = (v >= config->pid_polling_min_voltage);
        }

        if (out_reason)
        {
            if (supply_trigger == SUPPLY_TRIGGER_BOOT)
            {
                *out_reason = "12v supply: boot";
            }
            else if (above_automate_threshold)
            {
                *out_reason = "12v supply: Over threshold trigger";
            }
            else if (supply_trigger == SUPPLY_TRIGGER_IMU)
            {
                *out_reason = "12v supply: Motion trigger";
            }
            else if (supply_trigger == SUPPLY_TRIGGER_VOLTAGE_RISE)
            {
                *out_reason = "12v supply: V rise trigger";
            }
            else
            {
                *out_reason = "12v supply";
            }
        }
        return false;
    }

    supply_trigger = SUPPLY_TRIGGER_NONE;

    if (boot_keep_alive_active)
    {
        supply_trigger = SUPPLY_TRIGGER_BOOT;
        if (out_reason)
            *out_reason = "boot";
        return false;
    }

    if (config->imu_voltage_override_enabled)
    {
        activity_state_t imu_state = imu_get_activity_state();
        if (imu_state == ACTIVITY_STATE_ACTIVE)
        {
            motion_was_active = true;
            supply_trigger = SUPPLY_TRIGGER_IMU;
            if (out_reason)
                *out_reason = "imu_active";
            return false;
        }
    }
    else
    {
        motion_was_active = false;
        imu_stationary_hold_timer = 0;
    }

    float v = NAN;
    bool have_voltage = (sleep_mode_get_voltage(&v) == ESP_OK);
    if (have_voltage)
    {
        int64_t now_ms = esp_timer_get_time() / 1000LL;

        if (out_voltage)
            *out_voltage = v;

        voltage_history_record(v, now_ms, voltage_rise_sample_interval_ms(config->voltage_rise_time_seconds));
        if (config->voltage_rise_wakeup_enabled &&
            voltage_rising_bypass_active(now_ms,
                                         config->voltage_rise_threshold > 0.0f
                                             ? config->voltage_rise_threshold
                                             : POWER_DETECTION_VOLTAGE_RISE_DELTA_V,
                                         config->voltage_rise_time_seconds))
        {
            if (out_reason)
                *out_reason = "voltage_rise";
            supply_trigger = SUPPLY_TRIGGER_VOLTAGE_RISE;
            return false;
        }
        else if (!config->voltage_rise_wakeup_enabled)
        {
            voltage_rising_hold_timer = 0;
        }
    }

    // Mode: disable PID requests when below Power Saving -> Sleep Voltage threshold.
    // Uses dev_status voltage bit (set by sleep_mode task).
    if (config->disable_pid_requests_on_sleep_voltage && !dev_status_is_wake_voltage_ok())
    {
        if (out_reason)
            *out_reason = "sleep_voltage";
        return true;
    }

    // Mode: disable PID requests when below a custom Automate threshold.
    if (config->disable_pid_requests_on_automate_threshold)
    {
        if (have_voltage)
        {
            if (v < config->pid_polling_min_voltage)
            {
                if (out_reason)
                    *out_reason = "automate_threshold";
                return true;
            }
        }
    }

    if (out_reason)
        *out_reason = "none_found";
    return false;
}

bool power_detection_should_pause_pid_polling(const autopid_config_t *config, float *out_voltage, const char **out_reason)
{
    float voltage = NAN;
    const char *reason = NULL;
    bool paused = evaluate_pid_polling_pause(config, &voltage, &reason);

    pid_polling_state.paused = paused;
    pid_polling_state.reason = reason;
    pid_polling_state.voltage = voltage;

    if (out_voltage)
        *out_voltage = voltage;
    if (out_reason)
        *out_reason = reason;

    return paused;
}

power_detection_pid_polling_state_t power_detection_get_pid_polling_state(void)
{
    return pid_polling_state;
}

esp_err_t power_detection_start_load_test(void)
{
    if (load_test_active || load_test_task_handle != NULL || load_test_udp_task_handle != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    load_test_active = true;
    load_test_reset_status();

    load_test_task_handle = xTaskCreateStatic(load_test_task,
                                              "pd_load_test",
                                              POWER_DETECTION_LOAD_TEST_STACK_WORDS,
                                              NULL,
                                              5,
                                              load_test_task_stack,
                                              &load_test_task_buffer);
    if (load_test_task_handle == NULL)
    {
        load_test_active = false;
        load_test_finish_status(false);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool power_detection_load_test_is_active(void)
{
    return load_test_active;
}

char *power_detection_get_load_test_status_json(void)
{
    load_test_status_t snapshot;

    taskENTER_CRITICAL(&load_test_status_lock);
    snapshot = load_test_status;
    taskEXIT_CRITICAL(&load_test_status_lock);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        return NULL;
    }

    cJSON_AddBoolToObject(root, "active", snapshot.active);
    cJSON_AddBoolToObject(root, "complete", snapshot.complete);
    cJSON_AddStringToObject(root, "target_ip", POWER_DETECTION_LOAD_TEST_TARGET_IP);
    cJSON_AddNumberToObject(root, "target_port", POWER_DETECTION_LOAD_TEST_TARGET_PORT);
    cJSON_AddNumberToObject(root, "sample_interval_sec", POWER_DETECTION_LOAD_TEST_SAMPLE_INTERVAL_MS / 1000);
    cJSON_AddNumberToObject(root, "window_sec", POWER_DETECTION_LOAD_TEST_WINDOW_MS / 1000);

    if (snapshot.has_last_adc)
    {
        cJSON *last_adc = cJSON_CreateObject();
        if (last_adc != NULL)
        {
            cJSON_AddNumberToObject(last_adc, "avg_raw", snapshot.last_adc.avg_raw);
            cJSON_AddNumberToObject(last_adc, "median_raw", snapshot.last_adc.median_raw);
            cJSON_AddNumberToObject(last_adc, "min_raw", snapshot.last_adc.min_raw);
            cJSON_AddNumberToObject(last_adc, "max_raw", snapshot.last_adc.max_raw);
            cJSON_AddNumberToObject(last_adc, "avg_adc_mv", snapshot.last_adc.avg_adc_mv);
            cJSON_AddNumberToObject(last_adc, "battery_mv", snapshot.last_adc.battery_mv);
            cJSON_AddNumberToObject(last_adc, "valid_samples", snapshot.last_adc.valid_samples);
            cJSON_AddItemToObject(root, "last_adc", last_adc);
        }
    }

    cJSON *windows = cJSON_CreateArray();
    if (windows != NULL)
    {
        for (uint32_t i = 0; i < snapshot.window_count; i++)
        {
            cJSON *window = cJSON_CreateObject();
            if (window == NULL)
            {
                continue;
            }
            cJSON_AddStringToObject(window, "label", snapshot.windows[i].label);
            cJSON_AddNumberToObject(window, "sample_count", snapshot.windows[i].sample_count);
            cJSON_AddNumberToObject(window, "avg_median_raw", snapshot.windows[i].avg_median_raw);
            cJSON_AddItemToArray(windows, window);
        }
        cJSON_AddItemToObject(root, "windows", windows);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}
