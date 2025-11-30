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
QueueSetHandle_t queue_set_oled_updates = NULL;       // Merged updates from multiple sources for OLED task : queue_oled_updates_from_input + queue_oled_updates_from_sensors.

QueueHandle_t queue_control_cmd = NULL;               // Queue for passing config to control task.
QueueHandle_t queue_sensor_events = NULL;             // Unified sensor data for control/supervision.

QueueHandle_t queue_sensor_forward = NULL;            // HC-SR04 forward sensor readings.
QueueHandle_t queue_sensor_backward = NULL;           // HC-SR04 backward sensor readings.
QueueSetHandle_t queue_set_control = NULL;            // Control listens to config + sensors.
QueueSetHandle_t queue_set_sensors = NULL;            // Sensors task waits on either forward/backward.



/* -------------------------------------------------------------------------- */
/* Task Implementations                                                       */
/* -------------------------------------------------------------------------- */

// TODO: Read user inputs (buttons, rotary encoder position) and push to queue
static void input_task(void *arg)
{
    (void)arg;
    TickType_t last_button_tick = 0;
    const TickType_t button_debounce_ticks = pdMS_TO_TICKS(50); /* 50 ms software debounce */
    bool adjusting_value = false;

    /* Current configuration state derived from user input. */
    oled_update_t oled_state = {
        .set_speed_percent = 0U,
        .hl_on = false,
        .rev_on = false,
        .emr_br_active = false,
        .bar_graph_level = 0U,
        .act_speed_percent = 0U,
        .selected_option = OLED_CONFIG_OPTION_SPEED
    };

    control_cmd_t control_state = {
        .speed_percent = oled_state.set_speed_percent,
        .reverse_mode = oled_state.rev_on,
        .headlights_on = oled_state.hl_on
    };

    /* Publish defaults so UI and control tasks know initial state. */
    (void)xQueueSend(queue_oled_updates_from_input, &oled_state, 0);
    (void)xQueueSend(queue_control_cmd, &control_state, 0);

    ESP_LOGI(TAG, "Input task started");

    for (;;)
    {
        int value_from_queue = 0;
        /* Wait indefinitely for encoder events pushed from the ISR callback. */
        if (xQueueReceive(queue_encoder_events, &value_from_queue, portMAX_DELAY) == pdPASS)
        {
            /* Work on copies so we can detect changes at the end. */
            oled_update_t prev_oled_state = oled_state;
            int8_t step = 0;

            switch (value_from_queue)
            {
                case ROTARY_MENU_NAV_SELECT:
                {
                    TickType_t now = xTaskGetTickCount();
                    if ((now - last_button_tick) >= button_debounce_ticks)
                    {
                        adjusting_value = !adjusting_value;
                        last_button_tick = now;
                    }
                    break;
                }
                case ROTARY_MENU_NAV_STEP_NEXT:
                case ROTARY_MENU_NAV_STEP_PREV:
                {
                    int8_t step = (value_from_queue == ROTARY_MENU_NAV_STEP_NEXT) ? 1 : -1;
                    if (!adjusting_value)
                    {
                        int8_t next_option = (int8_t)oled_state.selected_option + step;
                        if (next_option >= (int8_t)OLED_CONFIG_OPTION_SPEED && next_option <= (int8_t)OLED_CONFIG_OPTION_REVERSE)
                        {
                            oled_state.selected_option = (oled_option_select_t)next_option;
                        }
                    }
                    else
                    {
                        switch (oled_state.selected_option)
                        {
                            case OLED_CONFIG_OPTION_SPEED:
                            {
                                int16_t new_speed = (int16_t)oled_state.set_speed_percent + (int16_t)(step);
                                if (new_speed < 0) { new_speed = 0; }
                                if (new_speed > 100) { new_speed = 100; }
                                oled_state.set_speed_percent = (uint8_t)new_speed;
                                break;
                            }
                            case OLED_CONFIG_OPTION_HEADLIGHTS:
                                oled_state.hl_on = !oled_state.hl_on;
                                break;
                            case OLED_CONFIG_OPTION_REVERSE:
                                oled_state.rev_on = !oled_state.rev_on;
                                break;
                            default:
                                break;
                        }
                    }
                    break;
                }
                default:
                    /* Ignore unexpected events. */
                    break;
            }

            /* Detect changes and send updates as needed. */
            bool oled_changed =
                (prev_oled_state.set_speed_percent != oled_state.set_speed_percent) ||
                (prev_oled_state.hl_on != oled_state.hl_on) ||
                (prev_oled_state.rev_on != oled_state.rev_on) ||
                (prev_oled_state.emr_br_active != oled_state.emr_br_active) ||
                (prev_oled_state.bar_graph_level != oled_state.bar_graph_level) ||
                (prev_oled_state.act_speed_percent != oled_state.act_speed_percent) ||
                (prev_oled_state.selected_option != oled_state.selected_option);

            if (oled_changed)
            {
                if (xQueueSend(queue_oled_updates_from_input, &oled_state, portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW(TAG, "Failed to send input update to OLED queue");
                }
            }

            bool control_changed =
                (prev_oled_state.set_speed_percent != oled_state.set_speed_percent) ||
                (prev_oled_state.hl_on != oled_state.hl_on) ||
                (prev_oled_state.rev_on != oled_state.rev_on);

            if (control_changed)
            {
                control_state.speed_percent = oled_state.set_speed_percent;
                control_state.reverse_mode = oled_state.rev_on;
                control_state.headlights_on = oled_state.hl_on;

                if (xQueueSend(queue_control_cmd, &control_state, portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW(TAG, "Failed to send control command");
                }
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
        QueueSetMemberHandle_t active = xQueueSelectFromSet(queue_set_sensors, portMAX_DELAY);
        sensor_data_t data = {0}; // added emergency_brake field initialization

        if (active == (QueueSetMemberHandle_t)queue_sensor_forward &&
            xQueueReceive(queue_sensor_forward, &data, 0) == pdPASS)
        {
            // TODO : move emergency_brake logic here from control task in order to pass also to the oled the EMERGENCY BRAKE status
            (void)xQueueSend(queue_oled_updates_from_sensors, &data, 0);
            (void)xQueueSend(queue_sensor_events, &data, 0);
        }
        else if (active == (QueueSetMemberHandle_t)queue_sensor_backward &&
                 xQueueReceive(queue_sensor_backward, &data, 0) == pdPASS)
        {
            // TODO : move emergency_brake logic here from control task in order to pass also to the oled the EMERGENCY BRAKE status
            (void)xQueueSend(queue_oled_updates_from_sensors, &data, 0);
            (void)xQueueSend(queue_sensor_events, &data, 0);
        }
    }
}

// TODO: Decide motor speed / LED status based on inputs & sensors
static void control_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Control task started");
    control_cmd_t cmd = {0};
    sensor_data_t sensor = {0};
    QueueSetMemberHandle_t activated_member;
    bool emergency_brake_active = false;
    bool previous_state_fwd_led = false;
    rgb_backward_mode_t prev_bwd_led = RGB_BWD_HALF_BRIGHT_RED;
    bool cmd_initialized = false;
    for (;;)
    {
        activated_member = xQueueSelectFromSet(queue_set_control, portMAX_DELAY);

        if (activated_member == (QueueSetMemberHandle_t)queue_control_cmd &&
            xQueueReceive(queue_control_cmd, &cmd, 0) == pdPASS)
        {
            cmd_initialized = true;
            // ESP_LOGI(TAG, "Control update: speed=%u%%, hl=%d, rev=%d",
            //          (unsigned)cmd.speed_percent,
            //          cmd.headlights_on,
            //          cmd.reverse_mode);

            if(cmd.headlights_on && previous_state_fwd_led == false)
            {
                rgb_led_set_forward(RGB_FWD_BRIGHT_WHITE);
                previous_state_fwd_led = true;
            }
            else if(!cmd.headlights_on && previous_state_fwd_led == true)
            {
                 rgb_led_set_forward(RGB_FWD_OFF);
                previous_state_fwd_led = false;
            }
            // Guard with emergency brake status, in order to see if we can send motor commands
            // TODO : Prepare motor commands based on speed_percent and reverse_mode + send to motor_task
        }
        else if (activated_member == (QueueSetMemberHandle_t)queue_sensor_events &&
                 xQueueReceive(queue_sensor_events, &sensor, 0) == pdPASS)
        {
            if (!cmd_initialized)
            {
                /* Wait for a config command before acting on sensors. */
                continue;
            }

            bool hazard_forward = sensor.forward && !cmd.reverse_mode && (sensor.distance < HCSR04_DISTANCE_TO_STOP_CM);
            bool hazard_backward = sensor.backward && cmd.reverse_mode && (sensor.distance < HCSR04_DISTANCE_TO_STOP_CM);

            bool emergency_now = hazard_forward || hazard_backward;
            bool safe_for_direction = (!cmd.reverse_mode && sensor.forward && sensor.distance >= HCSR04_DISTANCE_TO_STOP_CM) ||
                                      (cmd.reverse_mode && sensor.backward && sensor.distance >= HCSR04_DISTANCE_TO_STOP_CM);

            if (emergency_now && !emergency_brake_active)
            {
                emergency_brake_active = true;
                if (prev_bwd_led != RGB_BWD_BRIGHT_RED)
                {
                    rgb_led_set_backward(RGB_BWD_BRIGHT_RED);
                    prev_bwd_led = RGB_BWD_BRIGHT_RED;
                }
                // TODO: Prepare motor command to stop + send to motor_task
            }
            else if (!emergency_now && emergency_brake_active && safe_for_direction)
            {
                emergency_brake_active = false;
                if (prev_bwd_led != RGB_BWD_HALF_BRIGHT_RED)
                {
                    rgb_led_set_backward(RGB_BWD_HALF_BRIGHT_RED);
                    prev_bwd_led = RGB_BWD_HALF_BRIGHT_RED;
                }
                // TODO: Prepare motor command to resume previous speed + send to motor_task
            }
        }
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
        // TODO: Implement motor control via I2C based on commands from control task
    }
}

// TODO: Refresh OLED display / RGB LED patterns
static void ui_task(void *arg)
{
    /* Parameters for drawing driven entirely by messages from input/sensors. */
    oled_update_t oled_state = {
        .set_speed_percent = 0U,
        .hl_on = false,
        .rev_on = false,
        .emr_br_active = false,
        .bar_graph_level = 0U,
        .act_speed_percent = 0U,
        .selected_option = OLED_CONFIG_OPTION_SPEED
    };

    /* Initial look of the display when the app just started. */
    oled_draw_debug_screen(oled_state.set_speed_percent,
                           oled_state.hl_on,
                           oled_state.rev_on,
                           oled_state.emr_br_active,
                           oled_state.bar_graph_level,
                           oled_state.act_speed_percent,
                           oled_state.selected_option);

    /* OLED queue set update handler */
    QueueSetMemberHandle_t activated_member;

    ESP_LOGI(TAG, "UI task started");
    for (;;)
    {
        /* Wait for any OLED update from input or sensors via the queue set. */
        activated_member = xQueueSelectFromSet(queue_set_oled_updates, portMAX_DELAY);

        /* Process input updates - config related only */
        if (activated_member == (QueueSetMemberHandle_t)queue_oled_updates_from_input)
        {
            oled_update_t input_update = {0};
            if (xQueueReceive(queue_oled_updates_from_input, &input_update, 0) == pdPASS)
            {
                oled_state = input_update;
            }

        }
        /* Process sensor updates - debug related only */
        else if (activated_member == (QueueSetMemberHandle_t)queue_oled_updates_from_sensors)
        {
            sensor_data_t sensor_value = {0};
            if (xQueueReceive(queue_oled_updates_from_sensors, &sensor_value, 0) == pdPASS)
            {
                /* Use distance to populate debug visuals for now. */
                if (sensor_value.distance > 50U)
                {
                    sensor_value.distance = 50U;
                }

                /* Update bar graph based on which mode is active. */
                if (oled_state.rev_on && sensor_value.backward)
                {
                    oled_state.bar_graph_level = 2U*sensor_value.distance;
                }
                else if (!oled_state.rev_on && sensor_value.forward)
                {
                    oled_state.bar_graph_level = 2U*sensor_value.distance;
                }
            }
        }

        /* Redraw the OLED display with updated parameters */
        oled_draw_debug_screen(oled_state.set_speed_percent,
                               oled_state.hl_on,
                               oled_state.rev_on,
                               oled_state.emr_br_active,
                               oled_state.bar_graph_level,
                               oled_state.act_speed_percent,
                               oled_state.selected_option);
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

    /* Control/config/state queues */
    queue_control_cmd = xQueueCreate(5, sizeof(control_cmd_t));
    queue_sensor_events = xQueueCreate(10, sizeof(sensor_data_t));

    /* Sensor queues  : Updated from ISRs */
    queue_sensor_forward = xQueueCreate(5, sizeof(sensor_data_t));
    queue_sensor_backward = xQueueCreate(5, sizeof(sensor_data_t));
    queue_set_sensors = xQueueCreateSet(10);
    xQueueAddToSet(queue_sensor_forward, queue_set_sensors);
    xQueueAddToSet(queue_sensor_backward, queue_set_sensors);

    /* Queues and queue set for OLED updates from multiple sources: input/sensors */
    queue_oled_updates_from_input = xQueueCreate(5, sizeof(oled_update_t));
    queue_oled_updates_from_sensors = xQueueCreate(5, sizeof(sensor_data_t));
    queue_set_oled_updates = xQueueCreateSet(10);
    xQueueAddToSet(queue_oled_updates_from_input, queue_set_oled_updates);
    xQueueAddToSet(queue_oled_updates_from_sensors, queue_set_oled_updates);

    /* Queue set for control (config + sensors) */
    queue_set_control = xQueueCreateSet(15);
    xQueueAddToSet(queue_control_cmd, queue_set_control);
    xQueueAddToSet(queue_sensor_events, queue_set_control);

    ESP_LOGI(TAG, "Creating application tasks");

    BaseType_t ok;

    /* Initialize I2C bus. Set ESP32 as master. */
    hal_i2c_init();

    /* Initialize OLED screen. Set as I2C slave. */
    oled_init();

    /* Initialize rotary encoder */
    rotary_encoder_init();

    /* Initialize HC-SR04 sensors */
    hcsr04_init();
    hcsr04_start_periodic_trigger();

    /* Initialize RGB leds: FWD and BWD */
    rgb_led_init();
    rgb_led_set_backward(RGB_BWD_HALF_BRIGHT_RED); // Initial state for backward LED
    rgb_led_set_forward(RGB_FWD_OFF);              // Initial state for forward LED
 
    /* Create application tasks pinned to specific cores. */
    ok = xTaskCreatePinnedToCore(input_task, "input_task", INPUT_TASK_STACK_SIZE, NULL, INPUT_TASK_PRIORITY, NULL, CORE_IO);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create input_task");
        return ESP_FAIL;
    }

    ok = xTaskCreatePinnedToCore(sensors_task, "sensors_task", SENSORS_TASK_STACK_SIZE, NULL, SENSORS_TASK_PRIORITY, NULL, CORE_HIGH_PERF);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensors_task");
        return ESP_FAIL;
    }

    ok = xTaskCreatePinnedToCore(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL, CONTROL_TASK_PRIORITY, NULL, CORE_HIGH_PERF);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create control_task");
        return ESP_FAIL;
    }

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
