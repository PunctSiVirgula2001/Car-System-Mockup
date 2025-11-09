#pragma once
/*
 * HAL GPIO/Timers - Header
 * Purpose: Peripheral abstraction for GPIO and timers used by drivers (e.g., HC-SR04, RGB LED, rotary encoder).
 * Notes: Place your enums/structs/defines and function prototypes for GPIO config, read/write, and timing here.
 * Usage: Drivers include with: #include "hal_gpio_timers.h"
 */

/* GPIOs used I2C */
#define GPIO_I2C_SCL (22U)
#define GPIO_I2C_SDA (21U)

/* GPIOs used for Rotary Encoder */
#define GPIO_ROTARY_A  (18U)
#define GPIO_ROTARY_B  (19U)
#define GPIO_ROTARY_SW (23U)

/* GPIO used for HC-SR04 forward sensor */
#define GPIO_HC_SR04_FORWARD_TRIG (16U)
#define GPIO_HC_SR04_FORWARD_ECHO (35U)

/* GPIO used for HC-SR04 backward sensor */
#define GPIO_HC_SR04_BACKWARD_TRIG (5U)
#define GPIO_HC_SR04_BACKWARD_ECHO (34U)
