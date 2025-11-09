/*
 * Application Tasks - Source
 * Purpose: Create and configure FreeRTOS tasks for UI, sensors, and control.
 * Notes: Implement task functions and initialization glue here.
 */

#include "app_tasks.h"
#include "hal_i2c.h"
#include "hal_gpio_timers.h"
#include "oled_i2c.h"
#include "rgb_led.h"
#include "rotary_encoder.h"
#include "hcsr04.h"
#include "motor_ctrl_i2c.h"

#include "hal_i2c.h"
#include "hal_gpio_timers.h"
