/*
 * Application Tasks - Source
 * Purpose: Create and configure FreeRTOS tasks for UI, sensors, and control.
 * Notes: Implement task functions and initialization glue here.
 */

#include "app_tasks.h"

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
/* Queue Handles                                                              */
/* -------------------------------------------------------------------------- */

QueueHandle_t queue_encoder_events = NULL; // Queue for rotary encoder events, used in encoder callback.

QueueHandle_t queue_oled_updates_from_input = NULL;   // Queue for OLED display updates, used by UI task.
QueueHandle_t queue_oled_updates_from_sensors = NULL; // Queue for OLED display updates, used by UI task.
QueueSetHandle_t queue_set_oled_updates = NULL;       // Merged updates from multiple sources for OLED task.

// queue set for merging data for OLED updates


/* -------------------------------------------------------------------------- */
/* Task Implementations                                                       */
/* -------------------------------------------------------------------------- */

// TODO: Read user inputs (buttons, rotary encoder position) and push to queue
static void input_task(void *arg)
{
    (void)arg;
    int counter = 0;
    int value_from_queue = 0;
    TickType_t last_button_tick = 0;
    const TickType_t button_debounce_ticks = pdMS_TO_TICKS(50); /* 50 ms software debounce */
    ESP_LOGI(TAG, "Input task started");

    for (;;)
    {
        /* Wait indefinitely for encoder events pushed from the ISR callback. */
        if (xQueueReceive(queue_encoder_events, &value_from_queue, portMAX_DELAY) == pdPASS)
        {
            // Process the received encoder event
            if(1 == value_from_queue)
            {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_button_tick) >= button_debounce_ticks)
                {
                    //ESP_LOGI(TAG, "Encoder button pressed");
                    last_button_tick = now;
                }
            }
            else
            {
                counter += value_from_queue/2;
                ESP_LOGI(TAG, "Encoder event: value=%d, counter=%d", value_from_queue/2, counter);
            }

            // Send the processed input to OLED task for displaying update
            if(xQueueSend(queue_oled_updates_from_input, &value_from_queue, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGW(TAG, "Failed to send input update to OLED queue");
            }

        }
    }
}

// TODO: Poll sensors (distance via HC-SR04, etc.) and update shared state
static void sensors_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Sensors task started");
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

// TODO: Decide motor speed / LED status based on inputs & sensors
static void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Control task started");
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// TODO: Execute motor commands (I2C) prepared by control task
static void motor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Motor task started");
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static inline void clamp_option_index(int8_t *index, int8_t input_value)
{
    int8_t new_index = (int8_t)(*index + input_value);
    if (new_index < (int8_t)OLED_CONFIG_OPTION_SPEED)
    {
        new_index = (int8_t)OLED_CONFIG_OPTION_SPEED;
    }
    else if (new_index > (int8_t)OLED_CONFIG_OPTION_REVERSE)
    {
        new_index = (int8_t)OLED_CONFIG_OPTION_REVERSE;
    }
    *index = new_index;
}

// TODO: Refresh OLED display / RGB LED patterns
static void ui_task(void *arg)
{
    /* Parameters for drawing */
    uint8_t set_speed_percent = 0;
    bool hl_on = false;
    bool rev_on = false;
    bool emr_br_active = false;
    uint8_t bar_graph_level = 0;
    uint8_t act_speed_percent = 0;
    oled_option_select_t selected_option = OLED_CONFIG_OPTION_NONE;

    /* Initial look of the display when the app just started. */
    oled_draw_debug_screen(set_speed_percent, hl_on, rev_on, emr_br_active, (uint8_t)emr_br_active, (uint8_t)bar_graph_level, selected_option);
    
    /* OLED queue set update handler */
    QueueSetMemberHandle_t activated_member;

    /* Menu navigation states */

    /* Draw the initial CONFIG / DEBUG screen on the OLED. */
    bool rolling_through_options = true;
    int8_t option_index_or_value = 0;

    ESP_LOGI(TAG, "UI task started");
    for (;;)
    {
        /* Wait for any OLED update from input or sensors via the queue set. */
        activated_member = xQueueSelectFromSet(queue_set_oled_updates, portMAX_DELAY);

        /* Process input updates - config related only */
        if (activated_member == (QueueSetMemberHandle_t)queue_oled_updates_from_input)
        {
            int input_value = 0;
            if (xQueueReceive(queue_oled_updates_from_input, &input_value, 0) == pdPASS)
            {
                switch (input_value)
                {
                    case ROTARY_MENU_NAV_STEP_NEXT:
                        if(rolling_through_options)
                        {
                            clamp_option_index((int8_t *)&option_index_or_value, 1);
                            selected_option = (oled_option_select_t)option_index_or_value;
                        }
                        else
                        {
                            // adjust the selected option value
                            option_index_or_value++;
                            if(selected_option == OLED_CONFIG_OPTION_SPEED)
                            {
                                if(option_index_or_value > 100U)
                                {
                                    option_index_or_value = 100U;
                                }
                                set_speed_percent = (uint8_t)option_index_or_value;
                            }
                            else if(selected_option == OLED_CONFIG_OPTION_HEADLIGHTS)
                            {
                                hl_on = (option_index_or_value % 2U) ? true : false;
                            }
                            else if(selected_option == OLED_CONFIG_OPTION_REVERSE)
                            {
                                rev_on = (option_index_or_value % 2U) ? true : false;
                            }
                        }
                        break;
                    case ROTARY_MENU_NAV_STEP_PREV:
                        if(rolling_through_options)
                        {
                            clamp_option_index((int8_t *)&option_index_or_value, -1);
                            selected_option = (oled_option_select_t)option_index_or_value;
                        }
                        else
                        {
                            // adjust the selected option value
                            option_index_or_value--;
                            if(option_index_or_value < 0)
                            {
                                option_index_or_value = 0;
                            }
                            if(selected_option == OLED_CONFIG_OPTION_SPEED)
                            {
                                set_speed_percent = (uint8_t)option_index_or_value;
                            }
                            else if(selected_option == OLED_CONFIG_OPTION_HEADLIGHTS)
                            {
                                hl_on = (option_index_or_value % 2U) ? true : false;
                            }
                            else if(selected_option == OLED_CONFIG_OPTION_REVERSE)
                            {
                                rev_on = (option_index_or_value % 2U) ? true : false;
                            }
                        }
                        break;
                    case ROTARY_MENU_NAV_SELECT:
                        rolling_through_options = !rolling_through_options;
                        option_index_or_value = selected_option;
                        /* code */
                        break;
                    default:
                        break;
                }

            }

        }
        /* Process sensor updates - debug related only */
        else if (activated_member == (QueueSetMemberHandle_t)queue_oled_updates_from_sensors)
        {
            int sensor_value = 0;
            if (xQueueReceive(queue_oled_updates_from_sensors, &sensor_value, 0) == pdPASS)
            {
                // TODO: Update debug parameters based on sensor data
            }
        }

        /* Redraw the OLED display with updated parameters */
        oled_draw_debug_screen(set_speed_percent, hl_on, rev_on, emr_br_active, (uint8_t)set_speed_percent, (uint8_t)set_speed_percent, selected_option);
    }
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t app_tasks_init(void)
{
    ESP_LOGI(TAG, "Creating queues and queue sets the tasks.");
    /* Queue for event triggered by rotary encoder. */
    queue_encoder_events = xQueueCreate(10, sizeof(int));

    /* Queues and queue set for OLED updates from multiple sources: input/sensors */
    queue_oled_updates_from_input = xQueueCreate(10, sizeof(int));
    queue_oled_updates_from_sensors = xQueueCreate(10, sizeof(int));
    queue_set_oled_updates = xQueueCreateSet(20);
    xQueueAddToSet(queue_oled_updates_from_input, queue_set_oled_updates);
    xQueueAddToSet(queue_oled_updates_from_sensors, queue_set_oled_updates);

    ESP_LOGI(TAG, "Creating application tasks");

    BaseType_t ok;

    /* Initialize I2C bus. Set ESP32 as master. */
    hal_i2c_init();

    /* Initialize OLED screen. Set as I2C slave. */
    oled_init();

    rotary_encoder_init();

    ok = xTaskCreatePinnedToCore(input_task, "input_task", INPUT_TASK_STACK_SIZE, NULL, INPUT_TASK_PRIORITY, NULL, CORE_IO);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create input_task");
        return ESP_FAIL;
    }

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
