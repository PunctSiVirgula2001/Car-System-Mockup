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

/* Enable internal pull-up resistors on encoder pins */
#define ROTARY_USE_PULLUPS  (1)

/* Debounce time for the push button (ns) */
#define ROTARY_BUTTON_DEBOUNCE_NS  (10000U)

/* PCNT LOW and HIGH limits */
#define ROTARY_PCNT_LOW_LIMIT   (-2)
#define ROTARY_PCNT_HIGH_LIMIT  (2)
#define ROTARY_PCNT_WATCH_STEP  (25)  /* emit events roughly every 25 counts */

/* Rotary encoder menu navigation */
#define ROTARY_MENU_NAV_STEP_NEXT    (2)   /* Small step change */
#define ROTARY_MENU_NAV_STEP_PREV    (-2)  /* Small step change */
#define ROTARY_MENU_NAV_SELECT       (1)   /* Button press */
