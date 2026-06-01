#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include "icm42670.h"

typedef enum {
    ACTIVITY_STATE_STATIONARY = 0,
    ACTIVITY_STATE_ACTIVE = 1,
    ACTIVITY_STATE_INVALID
} activity_state_t;

typedef struct {
    uint8_t threshold;
    bool wom_x_enabled;
    bool wom_y_enabled;
    bool wom_z_enabled;
    bool smd_enabled;
    icm42670_accel_odr_t accel_odr;
    icm42670_accel_avg_t accel_avg;
    icm42670_wom_int_dur_t wom_int_dur;
    icm42670_wom_int_mode_t wom_int_mode;
    icm42670_wom_mode_t wom_ref_mode;
} imu_wom_settings_t;

void imu_default_wom_settings(imu_wom_settings_t *settings);
esp_err_t imu_init(i2c_port_t i2c_num, gpio_num_t sda_gpio, gpio_num_t scl_gpio, gpio_num_t int_gpio, const imu_wom_settings_t *settings);
esp_err_t imu_config_wom(const imu_wom_settings_t *settings);
esp_err_t imu_enable_wom(bool enable);
esp_err_t imu_read_accel(float *ax, float *ay, float *az);
esp_err_t imu_read_gyro(float *gx, float *gy, float *gz);
esp_err_t imu_read_temp(float *temp);
esp_err_t imu_get_device_id(uint8_t *id);
esp_err_t imu_set_accel_fsr(icm42670_accel_fsr_t fsr);
esp_err_t imu_set_gyro_fsr(icm42670_gyro_fsr_t fsr);
activity_state_t imu_get_activity_state(void);
uint8_t imu_get_last_int_status2(void);
uint32_t imu_get_wom_x_count(void);
uint32_t imu_get_wom_y_count(void);
uint32_t imu_get_wom_z_count(void);
uint32_t imu_get_last_active_ms(void);

#endif
