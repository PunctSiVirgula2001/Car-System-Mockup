#pragma once
/*
 * RGB LED Driver - Header
 * Purpose: Declare interfaces for controlling an RGB LED (GPIO/PWM/LEDC).
 * Notes: Add function prototypes and configuration defines here.
 */

#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

/* Forward LED modes */
typedef enum
{
    RGB_FWD_OFF = 0,
    RGB_FWD_BRIGHT_WHITE,
} rgb_forward_mode_t;

/* Backward LED modes */
typedef enum
{
    RGB_BWD_OFF = 0,
    RGB_BWD_HALF_BRIGHT_RED,
    RGB_BWD_BRIGHT_RED,
} rgb_backward_mode_t;

/* Initialise PWM channels for both forward and backward RGB LEDs. */
void rgb_led_init(void);

/* Set forward LEDs to OFF or BRIGHT_WHITE. */
void rgb_led_set_forward(rgb_forward_mode_t mode);

/* Set backward LEDs to OFF / HALF_BRIGHT_RED / BRIGHT_RED. */
void rgb_led_set_backward(rgb_backward_mode_t mode);
