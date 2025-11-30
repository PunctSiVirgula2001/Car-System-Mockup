#pragma once
/*
 * Application Tasks - Header
 * Purpose: Declarations to initialize and configure FreeRTOS tasks/queues.
 * Notes: Add task prototypes and the init entry point here.
 */

#include <stdbool.h>
#include <stdint.h>
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
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"

/* Structure for OLED display updates */
typedef struct {
    uint8_t set_speed_percent;
    bool hl_on;
    bool rev_on;
    bool emr_br_active;
    uint8_t bar_graph_level;
    uint8_t act_speed_percent;
    oled_option_select_t selected_option;
} oled_update_t;

/* Structure for control task commands */
typedef struct {
    uint8_t speed_percent;
    bool reverse_mode;
    bool headlights_on;
} control_cmd_t;


extern QueueHandle_t queue_encoder_events;
extern QueueHandle_t queue_oled_updates_from_input;
extern QueueHandle_t queue_oled_updates_from_sensors;
extern QueueSetHandle_t queue_set_oled_updates;
extern QueueHandle_t queue_control_cmd;
extern QueueHandle_t queue_sensor_events;
extern QueueHandle_t queue_sensor_forward;
extern QueueHandle_t queue_sensor_backward;
extern QueueSetHandle_t queue_set_control;


/* Entry point to create all application tasks. */
esp_err_t app_tasks_init(void);
