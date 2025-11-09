/*
 * RGB LED Driver - Source
 * Purpose: Implement RGB LED control (LEDC/PWM setup and color setting).
 * Notes: Wire in your chosen method (LEDC or direct PWM) here.
 */

#include "project_config.h"
#include "rgb_led.h"
#include "hal_gpio_timers.h"
