#pragma once
/*
 * Rotary Encoder Driver - Header
 * Purpose: Declare interfaces for reading a rotary encoder with push button.
 * Notes: Add function prototypes, event types, and pin config here.
 */

#include "driver/pulse_cnt.h"
#include "tasks/app_tasks.h"
#include "rotary_encoder_config.h"


/* Initialize rotary encoder GPIOs, interrupts, and state tracking. */
 void rotary_encoder_init();