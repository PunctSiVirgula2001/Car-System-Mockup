#pragma once
/*
 * HAL GPIO/Timers - Header
 * Purpose: Peripheral abstraction for GPIO and timers used by drivers (e.g., HC-SR04, RGB LED, rotary encoder).
 * Notes: Thin, generic API; implement with ESP-IDF (GPIO/LEDC/PCNT/timers) in the .c file.
 * Usage: Drivers include with: #include "hal_gpio_timers.h"
 */

#include <stdint.h>
#include <stdbool.h>

/* GPIO helpers */
int  hal_gpio_set_mode(uint32_t pin, bool output, bool pullup, bool pulldown);
int  hal_gpio_write(uint32_t pin, bool level);
int  hal_gpio_read(uint32_t pin); /* returns 0/1 or <0 on error */

typedef enum {
    HAL_GPIO_EDGE_DISABLE = 0,
    HAL_GPIO_EDGE_RISING,
    HAL_GPIO_EDGE_FALLING,
    HAL_GPIO_EDGE_ANY,
} hal_gpio_edge_t;

int  hal_gpio_isr_attach(uint32_t pin, hal_gpio_edge_t edge, void (*cb)(void *), void *arg);
int  hal_gpio_isr_detach(uint32_t pin);

/* Timing helpers */
int  hal_pulse_out_us(uint32_t pin, bool level, uint32_t width_us);
int  hal_pulse_in_us(uint32_t pin, bool target_level, uint32_t timeout_us);

/* PWM (for RGB LED) */
typedef int hal_pwm_ch_t; /* implementation-defined channel/id */
int  hal_pwm_init(hal_pwm_ch_t ch, uint32_t pin, uint32_t freq_hz, uint8_t resolution_bits);
int  hal_pwm_set_duty(hal_pwm_ch_t ch, uint32_t duty);
int  hal_pwm_deinit(hal_pwm_ch_t ch);
