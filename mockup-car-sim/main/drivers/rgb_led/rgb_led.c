/*
 * RGB LED Driver - Source
 * Purpose: Implement RGB LED control (LEDC/PWM setup and color setting).
 * Notes: Wire in your chosen method (LEDC or direct PWM) here.
 */

#include "project_config.h"
#include "rgb_led.h"
#include "hal_gpio_timers.h"
#include "driver/ledc.h"

/* LEDC timer configuration shared by all six channels. */
static void rgb_led_init_timer(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)RGB_LED_PWM_RESOLUTION_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = RGB_LED_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    (void)ledc_timer_config(&timer_conf);
}

static void rgb_led_init_channel(gpio_num_t gpio, ledc_channel_t channel)
{
    ledc_channel_config_t ch_conf = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    (void)ledc_channel_config(&ch_conf);
}

static inline void rgb_led_apply_duty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void rgb_led_init(void)
{
    rgb_led_init_timer();

    /* Forward RGB */
    rgb_led_init_channel(GPIO_RGB_FORWARD_RED, RGB_LED_PWM_CHANNEL_FORWARD_RED);
    rgb_led_init_channel(GPIO_RGB_FORWARD_GREEN, RGB_LED_PWM_CHANNEL_FORWARD_GREEN);
    rgb_led_init_channel(GPIO_RGB_FORWARD_BLUE, RGB_LED_PWM_CHANNEL_FORWARD_BLUE);

    /* Backward RGB */
    rgb_led_init_channel(GPIO_RGB_BACKWARD_RED, RGB_LED_PWM_CHANNEL_BACKWARD_RED);
    rgb_led_init_channel(GPIO_RGB_BACKWARD_GREEN, RGB_LED_PWM_CHANNEL_BACKWARD_GREEN);
    rgb_led_init_channel(GPIO_RGB_BACKWARD_BLUE, RGB_LED_PWM_CHANNEL_BACKWARD_BLUE);
}

void rgb_led_set_forward(rgb_forward_mode_t mode)
{
    switch (mode)
    {
        case RGB_FWD_BRIGHT_WHITE: /* amber/yellow forward light */
        {
            uint32_t duty_max = (1u << RGB_LED_PWM_RESOLUTION_BITS) - 1u;
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_RED, (duty_max)/4);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_GREEN, duty_max/4); /* trim green to avoid tint */
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_BLUE, (duty_max-100)/4);
            break;
        }
        case RGB_FWD_OFF:
        default:
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_RED, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_GREEN, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_FORWARD_BLUE, 0);
            break;
    }
}

void rgb_led_set_backward(rgb_backward_mode_t mode)
{
    uint32_t bright_duty = (1u << RGB_LED_PWM_RESOLUTION_BITS) - 1u;
    uint32_t half_duty = bright_duty / 2u - 50;

    switch (mode)
    {
        case RGB_BWD_BRIGHT_RED:
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_RED, bright_duty);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_GREEN, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_BLUE, 0);
            break;
        case RGB_BWD_HALF_BRIGHT_RED:
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_RED, half_duty);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_GREEN, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_BLUE, 0);
            break;
        case RGB_BWD_OFF:
        default:
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_RED, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_GREEN, 0);
            rgb_led_apply_duty(RGB_LED_PWM_CHANNEL_BACKWARD_BLUE, 0);
            break;
    }
}
