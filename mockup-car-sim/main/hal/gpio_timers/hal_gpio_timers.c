/*
 * HAL GPIO/Timers - Source
 * Purpose: Implement GPIO mode/control and timing utilities on top of ESP-IDF or another MCU HAL.
 * Notes: Add setup/teardown and simple helpers that drivers can reuse.
 */

#include "project_config.h"
#include "hal_gpio_timers.h"

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static inline ledc_timer_bit_t hal_bits_to_ledc_res(uint8_t bits)
{
    if (bits < 1) bits = 1;
    if (bits > 20) bits = 20; /* upper bound supported by LEDC on many targets */
    return (ledc_timer_bit_t)bits; /* LEDC_TIMER_1_BIT == 1, ... */
}

int hal_pwm_init(hal_pwm_ch_t ch, uint32_t pin, uint32_t freq_hz, uint8_t resolution_bits)
{
    if (ch < 0 || ch >= (hal_pwm_ch_t)LEDC_CHANNEL_MAX) {
        return -1;
    }

    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = hal_bits_to_ledc_res(resolution_bits),
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = (uint32_t)freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        return -1;
    }

    ledc_channel_config_t ccfg = {
        .gpio_num   = (int)pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)ch,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) {
        return -1;
    }
    return 0;
}

int hal_pwm_set_duty(hal_pwm_ch_t ch, uint32_t duty)
{
    if (ch < 0 || ch >= (hal_pwm_ch_t)LEDC_CHANNEL_MAX) 
    {
        return -1;
    }
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
    if (err != ESP_OK) {
        return -1;
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
    if (err != ESP_OK) {
        return -1;
    }
    return 0;
}

int hal_pwm_deinit(hal_pwm_ch_t ch)
{
    if (ch < 0 || ch >= (hal_pwm_ch_t)LEDC_CHANNEL_MAX) 
    {
        return -1;
    }
    /* Stop output and set idle level low */
    esp_err_t err = ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, 0);
    if (err != ESP_OK) 
    {
        return -1;
    }
    return 0;
}

/* ---------------- GPIO helpers ---------------- */

int hal_gpio_set_mode(uint32_t pin, bool output, bool pullup, bool pulldown)
{
    gpio_config_t cfg = {0};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = output ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    cfg.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    return (gpio_config(&cfg) == ESP_OK) ? 0 : -1;
}

int hal_gpio_write(uint32_t pin, bool level)
{
    return gpio_set_level((gpio_num_t)pin, level ? 1 : 0) == ESP_OK ? 0 : -1;
}

int hal_gpio_read(uint32_t pin)
{
    int v = gpio_get_level((gpio_num_t)pin);
    return (v == 0 || v == 1) ? v : -1;
}

/* ISR attach/detach */
static bool s_gpio_isr_service_installed = false;

static inline gpio_int_type_t map_edge(hal_gpio_edge_t edge)
{
    switch (edge) 
    {
        case HAL_GPIO_EDGE_RISING:  return GPIO_INTR_POSEDGE;
        case HAL_GPIO_EDGE_FALLING: return GPIO_INTR_NEGEDGE;
        case HAL_GPIO_EDGE_ANY:     return GPIO_INTR_ANYEDGE;
        case HAL_GPIO_EDGE_DISABLE:
        default:                    return GPIO_INTR_DISABLE;
    }
}

int hal_gpio_isr_attach(uint32_t pin, hal_gpio_edge_t edge, void (*cb)(void *), void *arg)
{
    if (!s_gpio_isr_service_installed) {
        if (gpio_install_isr_service(0) != ESP_OK) {
            return -1;
        }
        s_gpio_isr_service_installed = true;
    }
    gpio_set_intr_type((gpio_num_t)pin, map_edge(edge));
    return gpio_isr_handler_add((gpio_num_t)pin, cb, arg) == ESP_OK ? 0 : -1;
}

int hal_gpio_isr_detach(uint32_t pin)
{
    return gpio_isr_handler_remove((gpio_num_t)pin) == ESP_OK ? 0 : -1;
}

/* ---------------- Pulse helpers ---------------- */

int hal_pulse_out_us(uint32_t pin, bool level, uint32_t width_us)
{
    if (hal_gpio_write(pin, level) != 0)
        return -1;
    esp_rom_delay_us(width_us);
    return hal_gpio_write(pin, !level);
}

int hal_pulse_in_us(uint32_t pin, bool target_level, uint32_t timeout_us)
{
    uint64_t t_start = esp_timer_get_time();
    /* Wait for line to become target_level */
    while (hal_gpio_read(pin) != (target_level ? 1 : 0)) {
        if ((esp_timer_get_time() - t_start) >= timeout_us) {
            return -1; /* timeout waiting for start */
        }
    }
    uint64_t t0 = esp_timer_get_time();
    /* Measure while at target level */
    while (hal_gpio_read(pin) == (target_level ? 1 : 0)) {
        if ((esp_timer_get_time() - t0) >= timeout_us) {
            return -2; /* timeout while measuring */
        }
    }
    uint64_t t1 = esp_timer_get_time();
    return (int)(t1 - t0);
}


