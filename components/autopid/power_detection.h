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

#ifndef __POWER_DETECTION_H__
#define __POWER_DETECTION_H__

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "autopid.h"

typedef struct
{
    bool paused;
    const char *reason;
    float voltage;
} power_detection_pid_polling_state_t;

bool power_detection_should_pause_pid_polling(const autopid_config_t *config, float *out_voltage, const char **out_reason);
power_detection_pid_polling_state_t power_detection_get_pid_polling_state(void);
esp_err_t power_detection_start_load_test(void);
bool power_detection_load_test_is_active(void);
char *power_detection_get_load_test_status_json(void);

#endif // __POWER_DETECTION_H__
