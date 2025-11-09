#pragma once
/*
 * Rotary Encoder Config - Header
 * Purpose: Compile-time settings for encoder (A/B/BTN pins, pull-ups, debounce/PCNT mode).
 * Notes: Pins in board_pins.h or referenced here via macros; no code.
 * Usage: Included indirectly via project_config.h in rotary_encoder.c.
 */


/* GPIOs used for Rotary Encoder */
#define GPIO_ROTARY_A  (18U)
#define GPIO_ROTARY_B  (19U)
#define GPIO_ROTARY_SW (23U)