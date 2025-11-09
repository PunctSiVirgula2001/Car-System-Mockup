/*
 * Project Config - Header
 * Purpose: Central include aggregating per-driver configuration headers.
 * Notes: Include from HAL and driver .c files to access constants without polluting public APIs.
 */

#pragma once

/* Per-driver configs */
#include "oled_config.h"
#include "motor_ctrl_config.h"
#include "rotary_encoder_config.h"
#include "hcsr04_config.h"
#include "rgb_led_config.h"
