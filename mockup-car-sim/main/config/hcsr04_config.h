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

/* HC-SR04 Configuration */
#define HCSR04_TRIGGER_PULSE_US    (10U)    /* Trigger pulse duration in microseconds */
#define HCSR04_TIMEOUT_MS          (30U)    /* Measurement timeout in milliseconds */
#define HCSR04_CM_PER_US           (58U)    /* Conversion factor: microseconds to centimeters */

#define HCSR04_TRIGGER_PERIOD_MS   (20U)    /* Period between sensor triggers in milliseconds */

#define HCSR04_DISTANCE_TO_STOP_CM (15U)   /* Distance threshold in cm to trigger stop/emergency brake */

typedef struct {
    uint8_t distance;
    bool forward;
    bool backward;
    bool emergency_brake;
} sensor_data_t;