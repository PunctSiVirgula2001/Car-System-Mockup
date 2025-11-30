/*
 * HC-SR04 Distance Sensor - Source
 * Purpose: Implement trigger pulse and echo timing to compute distance.
 * Notes: Uses GPIO ISRs to measure echo pulse duration.
 */
#include "hcsr04.h"

static const char *TAG = "hcsr04";

/* Timer handle for periodic sensor triggering */
static TimerHandle_t sensor_trigger_timer = NULL;

/* Forward sensor variables */
static volatile uint32_t forward_pulse_start = 0;
static volatile uint32_t forward_pulse_duration = 0;

/* Backward sensor variables */
static volatile uint32_t backward_pulse_start = 0;
static volatile uint32_t backward_pulse_duration = 0;

/* Timer callback to trigger HC-SR04 sensors periodically */
static void sensor_trigger_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
    static bool trigger_forward = true;
    
    /* Alternate between forward and backward sensors */
    if (trigger_forward)
    {
        hcsr04_trigger_forward();
    }
    else
    {
        hcsr04_trigger_backward();
    }
    
    trigger_forward = !trigger_forward;
}

/* Forward sensor ECHO ISR */
static void IRAM_ATTR forward_echo_isr(void *arg)
{
    (void)arg;
    if (hal_gpio_read(GPIO_HC_SR04_FORWARD_ECHO))
    {
        /* Rising edge - start timing */
        forward_pulse_start = esp_timer_get_time();
    }
    else
    {
        /* Falling edge - calculate duration */
        forward_pulse_duration = esp_timer_get_time() - forward_pulse_start;
        
        /* Send data to sensors_task */
        sensor_data_t data = {
            .distance = (uint8_t)(forward_pulse_duration / HCSR04_CM_PER_US),
            .forward = true,
            .backward = false
        };
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sensor_forward, &data, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Backward sensor ECHO ISR */
static void IRAM_ATTR backward_echo_isr(void *arg)
{
    (void)arg;
    if (hal_gpio_read(GPIO_HC_SR04_BACKWARD_ECHO))
    {
        /* Rising edge - start timing */
        backward_pulse_start = esp_timer_get_time();
    }
    else
    {
        /* Falling edge - calculate duration */
        backward_pulse_duration = esp_timer_get_time() - backward_pulse_start;
        
        /* Send data to sensors_task */
        sensor_data_t data = {
            .distance = (uint8_t)(backward_pulse_duration / HCSR04_CM_PER_US),
            .forward = false,
            .backward = true
        };
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(queue_sensor_backward, &data, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void hcsr04_init(void)
{
    /* Configure TRIG pins as output */
    hal_gpio_set_mode(GPIO_HC_SR04_FORWARD_TRIG, true, false, false);
    hal_gpio_set_mode(GPIO_HC_SR04_BACKWARD_TRIG, true, false, false);
    hal_gpio_write(GPIO_HC_SR04_FORWARD_TRIG, 0);
    hal_gpio_write(GPIO_HC_SR04_BACKWARD_TRIG, 0);
    
    /* Configure ECHO pins as input with interrupts on both edges */
    hal_gpio_set_mode(GPIO_HC_SR04_FORWARD_ECHO, false, false, false);
    hal_gpio_isr_attach(GPIO_HC_SR04_FORWARD_ECHO, HAL_GPIO_EDGE_ANY, forward_echo_isr, NULL);
    
    hal_gpio_set_mode(GPIO_HC_SR04_BACKWARD_ECHO, false, false, false);
    hal_gpio_isr_attach(GPIO_HC_SR04_BACKWARD_ECHO, HAL_GPIO_EDGE_ANY, backward_echo_isr, NULL);
}

void hcsr04_trigger_forward(void)
{
    hal_gpio_write(GPIO_HC_SR04_FORWARD_TRIG, 1);
    esp_rom_delay_us(HCSR04_TRIGGER_PULSE_US);
    hal_gpio_write(GPIO_HC_SR04_FORWARD_TRIG, 0);
}

void hcsr04_trigger_backward(void)
{
    hal_gpio_write(GPIO_HC_SR04_BACKWARD_TRIG, 1);
    esp_rom_delay_us(HCSR04_TRIGGER_PULSE_US);
    hal_gpio_write(GPIO_HC_SR04_BACKWARD_TRIG, 0);
}

esp_err_t hcsr04_start_periodic_trigger(void)
{
    /* Create software timer for periodic sensor triggering */
    sensor_trigger_timer = xTimerCreate(
        "hcsr04_trigger",                       // Timer name
        pdMS_TO_TICKS(HCSR04_TRIGGER_PERIOD_MS), // Period from config
        pdTRUE,                                  // Auto-reload
        NULL,                                    // Timer ID (not used)
        sensor_trigger_timer_callback            // Callback function
    );
    
    if (sensor_trigger_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sensor trigger timer");
        return ESP_FAIL;
    }
    
    /* Start the timer */
    if (xTimerStart(sensor_trigger_timer, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start sensor trigger timer");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}
