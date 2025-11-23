#pragma once
/*
 * Application Tasks - Header
 * Purpose: Declarations to initialize and configure FreeRTOS tasks/queues.
 * Notes: Add task prototypes and the init entry point here.
 */

#include "esp_err.h"

/* HAL / Drivers */
#include "hal_i2c.h"
#include "hal_gpio_timers.h"
#include "oled_i2c.h"
#include "rgb_led.h"
#include "rotary_encoder.h"
#include "hcsr04.h"
#include "motor_ctrl_i2c.h"

/* FreeRTOS / ESP-IDF */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

extern QueueHandle_t queue_encoder_events;


/* Entry point to create all application tasks. */
esp_err_t app_tasks_init(void);
