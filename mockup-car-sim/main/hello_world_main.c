/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "tasks/app_tasks.h"
#include "esp_log.h"

void app_main(void)
{
	esp_err_t err = app_tasks_init();
	if (err != ESP_OK) {
		ESP_LOGE("app_main", "Failed to initialize application tasks (err=%d)", (int)err);
		return;
	}
	ESP_LOGI("app_main", "Application tasks started");
}
