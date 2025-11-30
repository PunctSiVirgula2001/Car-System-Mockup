#pragma once
/*
 * HC-SR04 Distance Sensor - Header
 * Purpose: Declare interfaces for triggering and reading distance in cm.
 * Notes: Add function prototypes and pin configuration here.
 */

#include <stdint.h>
#include "project_config.h"
#include "hal_gpio_timers.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_tasks.h"

/* Initialize both HC-SR04 sensors */
void hcsr04_init(void);

/* Start periodic sensor triggering using FreeRTOS timer */
esp_err_t hcsr04_start_periodic_trigger(void);

/* Trigger measurement on forward sensor (ISR will send result to queue) */
void hcsr04_trigger_forward(void);

/* Trigger measurement on backward sensor (ISR will send result to queue) */
void hcsr04_trigger_backward(void);
