/*
 * Rotary Encoder Driver - Source
 * Purpose: Implement quadrature decoding and optional button handling.
 * Notes: Use GPIO interrupts/timers or ESP-IDF's PCNT peripheral as needed.
 */

#include "project_config.h"
#include "rotary_encoder.h"
#include "hal_gpio_timers.h"
#include "esp_attr.h"
#include "hal/gpio_types.h"


static bool IRAM_ATTR rotary_encoder_pcnt_rotation_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send watch point to queue, from this interrupt callback
    int value = (edata->watch_point_value == 1) ? 1 : (int)((-1) * (edata->watch_point_value)); // 1 -> button, others -> scaled rotation
    xQueueSendFromISR(queue, &value, &high_task_wakeup);
    // return whether a high priority task has been waken up by this function
    return (high_task_wakeup == pdTRUE);
}


void rotary_encoder_init()
{
    /* Ensure encoder inputs are pulled up (EC11 to GND). */
    hal_gpio_set_mode(GPIO_ROTARY_A, false, true, false);
    hal_gpio_set_mode(GPIO_ROTARY_B, false, true, false);
    hal_gpio_set_mode(GPIO_ROTARY_SW, false, true, false);

    /* Install PCNT unit. */
    pcnt_unit_config_t pcnt_config = 
    {
        .low_limit = ROTARY_PCNT_LOW_LIMIT,
        .high_limit = ROTARY_PCNT_HIGH_LIMIT,
        .flags.accum_count = true,
        .intr_priority = 1
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_config, &pcnt_unit));

    /* Install PCNT channel. */
    pcnt_chan_config_t chan_config = 
    {
        .edge_gpio_num = GPIO_ROTARY_A,
        .level_gpio_num = GPIO_ROTARY_B
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // decrease the counter on rising edge, increase the counter on falling edge
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_HOLD));
    // keep the counting mode when the control signal is high level, and reverse the counting mode when the control signal is low level
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // optional glitch filter to suppress contact bounce
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = ROTARY_BUTTON_DEBOUNCE_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_cfg));
    // add watch points at low/zero/high and mid steps to get callbacks while moving
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ROTARY_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, ROTARY_PCNT_HIGH_LIMIT));
    
    pcnt_unit_clear_count(pcnt_unit);

    pcnt_event_callbacks_t cbs = 
    {
        .on_reach = rotary_encoder_pcnt_rotation_callback,
    };

    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue_encoder_events));

    /* Enable and start counting. */
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

 
 
 
    /* Button via PCNT (edge only). */
    pcnt_unit_config_t btn_unit_config =
    {
        .low_limit = -1,
        .high_limit = 1,
        .flags.accum_count = true,
        .intr_priority = 1
    };
    pcnt_unit_handle_t btn_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&btn_unit_config, &btn_unit));

    pcnt_chan_config_t btn_chan_config =
    {
        .edge_gpio_num = GPIO_ROTARY_SW,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t btn_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(btn_unit, &btn_chan_config, &btn_chan));

    /* Press (falling) increments to 1, release (rising) decrements to 0. */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(btn_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));

    pcnt_glitch_filter_config_t btn_filter_cfg = {
        .max_glitch_ns = ROTARY_BUTTON_DEBOUNCE_NS, /* keep within PCNT limit; software debounce in task */
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(btn_unit, &btn_filter_cfg));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(btn_unit, 1));
    pcnt_unit_clear_count(btn_unit);

    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(btn_unit, &cbs, queue_encoder_events));
    ESP_ERROR_CHECK(pcnt_unit_enable(btn_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(btn_unit));


}
