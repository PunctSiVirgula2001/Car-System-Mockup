/*
 * Rotary Encoder Driver - Source
 * Purpose: Implement quadrature decoding and optional button handling.
 * Notes: Use GPIO interrupts/timers or ESP-IDF's PCNT peripheral as needed.
 */

#include "project_config.h"
#include "rotary_encoder.h"
#include "hal_gpio_timers.h"
