#pragma once
/*
 * HC-SR04 Config - Header
 * Purpose: Compile-time settings for ultrasonic sensor (TRIG/ECHO pins, timeouts, conversion factors).
 * Notes: Pins typically come from board_pins.h; keep only constants/macros here.
 * Usage: Included indirectly via project_config.h in hcsr04.c.
 */

/* GPIO used for HC-SR04 forward sensor */
#define GPIO_HC_SR04_FORWARD_TRIG (16U)
#define GPIO_HC_SR04_FORWARD_ECHO (35U)

/* GPIO used for HC-SR04 backward sensor */
#define GPIO_HC_SR04_BACKWARD_TRIG (5U)
#define GPIO_HC_SR04_BACKWARD_ECHO (34U)


