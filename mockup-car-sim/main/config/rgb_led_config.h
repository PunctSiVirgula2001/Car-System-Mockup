#pragma once
/*
 * RGB LED Config - Header
 * Purpose: Compile-time settings for RGB LED (pin(s), PWM/LEDC channel(s), frequency, resolution).
 * Notes: Pins can be pulled from board_pins.h; keep device constants here.
 * Usage: Included indirectly via project_config.h in rgb_led.c.
 */

/* GPIOs used for RGB forward LEDs */
#define GPIO_RGB_FORWARD_RED    (27U)
#define GPIO_RGB_FORWARD_GREEN  (26U)
#define GPIO_RGB_FORWARD_BLUE   (25U)

/* GPIOs used for RGB backward LEDs */
#define GPIO_RGB_BACKWARD_RED   (14U)
#define GPIO_RGB_BACKWARD_GREEN (12U)
#define GPIO_RGB_BACKWARD_BLUE  (13U)

/* PWM/LEDC settings */
#define RGB_LED_PWM_FREQ_HZ         (5000U)  /* 5 kHz */
#define RGB_LED_PWM_RESOLUTION_BITS (8U)     /* 8-bit resolution (0-255) */

/* PWM/LEDC channels for RGB forward LEDs */
#define RGB_LED_PWM_CHANNEL_FORWARD_RED    (0)
#define RGB_LED_PWM_CHANNEL_FORWARD_GREEN  (1)
#define RGB_LED_PWM_CHANNEL_FORWARD_BLUE   (2)

/* PWM/LEDC channels for RGB backward LEDs */
#define RGB_LED_PWM_CHANNEL_BACKWARD_RED    (3)
#define RGB_LED_PWM_CHANNEL_BACKWARD_GREEN  (4)
#define RGB_LED_PWM_CHANNEL_BACKWARD_BLUE   (5)

