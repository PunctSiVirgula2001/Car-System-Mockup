/*
 * Application Tasks - Source
 * Purpose: Create and configure FreeRTOS tasks for UI, sensors, and control.
 * Notes: Implement task functions and initialization glue here.
 */

#include "app_tasks.h"

/* HAL / Drivers */
#include "hal_i2c.h"
#include "hal_gpio_timers.h"
#include "oled_i2c.h"
#include "rgb_led.h"
#include "rotary_encoder.h"
#include "hcsr04.h"
#include "motor_ctrl_i2c.h"

/* FreeRTOS / ESP-IDF */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

/* -------------------------------------------------------------------------- */
/* Task Configs                                                               */
/* -------------------------------------------------------------------------- */

#define INPUT_TASK_STACK_SIZE   (2048)
#define SENSORS_TASK_STACK_SIZE (2048)
#define CONTROL_TASK_STACK_SIZE (2048)
#define MOTOR_TASK_STACK_SIZE   (2048)
#define UI_TASK_STACK_SIZE      (2048)

/* Priorities mapping per user spec:
 * Core 1: control(6) > motor(5) > sensors(4)
 * Core 0: input(3) > ui(2)
 */
#define CONTROL_TASK_PRIORITY   (6)
#define MOTOR_TASK_PRIORITY     (5)
#define SENSORS_TASK_PRIORITY   (4)
#define INPUT_TASK_PRIORITY     (3)
#define UI_TASK_PRIORITY        (2)

/* Core pinning: HIGH_PERF on core 1, IO on core 0; on UNICORE pin all to core 0 */
#define CORE_HIGH_PERF  (1)
#define CORE_IO         (0)


static const char *TAG = "app_tasks";

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void input_task(void *arg);   /* Input Task */
static void sensors_task(void *arg); /* Sensors Task */
static void control_task(void *arg); /* Control Task */
static void motor_task(void *arg);   /* Motor Task */
static void ui_task(void *arg);      /* User Interface Task */

/* -------------------------------------------------------------------------- */
/* Task Implementations                                                       */
/* -------------------------------------------------------------------------- */

static void input_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Input task started");
    for (;;)
    {
        // TODO: Read user inputs (buttons, rotary encoder position) and push to queue
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void sensors_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Sensors task started");
    for (;;)
    {
        // TODO: Poll sensors (distance via HC-SR04, etc.) and update shared state
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Control task started");
    for (;;)
    {
        // TODO: Decide motor speed / LED status based on inputs & sensors
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void motor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Motor task started");
    for (;;)
    {
        // TODO: Execute motor commands (I2C) prepared by control task
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void ui_task(void *arg)
{
    (void)arg;
    bool hl = true;
    bool rev = true;
    int speed = 0U;
    ESP_LOGI(TAG, "UI task started");

    /* Draw the initial CONFIG / DEBUG screen on the OLED. */
    

    for (;;)
    {
        oled_draw_debug_screen(speed, hl, rev);
        // TODO: Refresh OLED display / RGB LED patterns
        vTaskDelay(pdMS_TO_TICKS(60));
        hl = !hl;
        rev = !rev;
        speed += 10U;
        speed = (speed == 100) ? 0U : speed;

    }
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t app_tasks_init(void)
{
    ESP_LOGI(TAG, "Creating application tasks");

    BaseType_t ok;

    /* Initialize I2C bus. Set ESP32 as master. */
    hal_i2c_init();

    /* Initialize OLED screen. Set as I2C slave. */
    oled_init();

    // ok = xTaskCreatePinnedToCore(input_task, "input_task", INPUT_TASK_STACK_SIZE, NULL, INPUT_TASK_PRIORITY, NULL, CORE_IO);
    // if (ok != pdPASS)
    // {
    //     ESP_LOGE(TAG, "Failed to create input_task");
    //     return ESP_FAIL;
    // }

    // ok = xTaskCreatePinnedToCore(sensors_task, "sensors_task", SENSORS_TASK_STACK_SIZE, NULL, SENSORS_TASK_PRIORITY, NULL, CORE_HIGH_PERF);
    // if (ok != pdPASS)
    // {
    //     ESP_LOGE(TAG, "Failed to create sensors_task");
    //     return ESP_FAIL;
    // }

    // ok = xTaskCreatePinnedToCore(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, NULL, CORE_HIGH_PERF);
    // if (ok != pdPASS)
    // {
    //     ESP_LOGE(TAG, "Failed to create control_task");
    //     return ESP_FAIL;
    // }

    // ok = xTaskCreatePinnedToCore(motor_task, "motor_task", MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL, CORE_HIGH_PERF);
    // if (ok != pdPASS)
    // {
    //     ESP_LOGE(TAG, "Failed to create motor_task");
    //     return ESP_FAIL;
    // }

    ok = xTaskCreatePinnedToCore(ui_task, "ui_task", UI_TASK_STACK_SIZE, NULL, UI_TASK_PRIORITY, NULL, CORE_IO);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ui_task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All tasks created successfully");
    return ESP_OK;
}
