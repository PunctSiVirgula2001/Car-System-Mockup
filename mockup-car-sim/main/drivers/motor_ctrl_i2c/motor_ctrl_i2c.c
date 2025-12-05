/*
 * Motor Control over I2C - Source
 * Purpose: Implement I2C messaging to set motor speeds and read telemetry.
 * Notes: Define your protocol and error handling here.
 */

#include <string.h>
#include <stdbool.h>
#include "project_config.h"
#include "motor_ctrl_i2c.h"
#include "hal_i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "motor_ctrl_i2c";
static i2c_master_dev_handle_t s_motor_dev_handle = NULL;
static QueueHandle_t s_response_queue = NULL;

static bool command_requires_single_arg(motor_ctrl_command_t command)
{
    switch (command)
    {
        case MOTOR_CMD_SET_SPEED:
        case MOTOR_CMD_SET_TORQUE:
        case MOTOR_CMD_SET_DIRECTION:
        case MOTOR_CMD_SET_ACCELERATION:
            return true;
        default:
            return false;
    }
}

esp_err_t motor_ctrl_init(void)
{
    esp_err_t err = hal_i2c_add_device(MOTOR_CTRL_I2C_ADDRESS, &s_motor_dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add motor controller at addr 0x%02X: %s", MOTOR_CTRL_I2C_ADDRESS, esp_err_to_name(err));
        return err;
    }

    if (s_response_queue == NULL)
    {
        s_response_queue = xQueueCreate(MOTOR_CTRL_RESPONSE_QUEUE_LEN, sizeof(motor_ctrl_response_t));
        if (s_response_queue == NULL)
        {
            ESP_LOGE(TAG, "Failed to create motor response queue");
            return ESP_FAIL;
        }
    }

    /* Small delay to allow peripheral to settle before first command. */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Enable motor driver on startup. */
    err = motor_ctrl_send_command(MOTOR_CMD_ENABLE_DRIVER, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable motor driver: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "Motor controller initialized");
    return ESP_OK;
}

esp_err_t motor_ctrl_send_command(motor_ctrl_command_t command, const uint8_t *data, size_t data_length)
{
    bool needs_arg = command_requires_single_arg(command);
    if ((needs_arg && data_length != 1U) || (!needs_arg && data_length != 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[1U + MOTOR_CTRL_MAX_DATA_LEN];
    buffer[0] = (uint8_t)command;

    if (data_length > 0U)
    {
        memcpy(&buffer[1], data, data_length);
    }

    size_t total_length = 1U + data_length;
    TickType_t timeout_ticks = pdMS_TO_TICKS(MOTOR_CTRL_I2C_TIMEOUT_MS);

    return hal_i2c_master_write(s_motor_dev_handle, buffer, total_length, timeout_ticks);
}

esp_err_t motor_ctrl_receive_response(motor_ctrl_command_t command_tag,
                                      uint8_t *rx_data,
                                      size_t rx_length)
{
    TickType_t timeout_ticks = pdMS_TO_TICKS(MOTOR_CTRL_I2C_TIMEOUT_MS);

    esp_err_t err = hal_i2c_master_read(s_motor_dev_handle,
                                        rx_data,
                                        rx_length,
                                        timeout_ticks);

    if (err == ESP_OK && s_response_queue != NULL)
    {
        motor_ctrl_response_t resp = {
            .command = command_tag,
            .length = rx_length
        };
        memcpy(resp.data, rx_data, rx_length);

        if (xQueueSend(s_response_queue, &resp, 0) != pdPASS)
        {
            /* Keep latest by resetting queue when full. */
            (void)xQueueSend(s_response_queue, &resp, 0);
        }
    }

    return err;
}

esp_err_t motor_ctrl_set_speed(uint8_t speed_percent)
{
    return motor_ctrl_send_command(MOTOR_CMD_SET_SPEED, &speed_percent, 1U);
}

esp_err_t motor_ctrl_set_torque(uint8_t torque_percent)
{
    return motor_ctrl_send_command(MOTOR_CMD_SET_TORQUE, &torque_percent, 1U);
}

esp_err_t motor_ctrl_set_direction(uint8_t direction)
{
    return motor_ctrl_send_command(MOTOR_CMD_SET_DIRECTION, &direction, 1U);
}

esp_err_t motor_ctrl_set_acceleration(uint8_t accel)
{
    return motor_ctrl_send_command(MOTOR_CMD_SET_ACCELERATION, &accel, 1U);
}

QueueHandle_t motor_ctrl_get_response_queue(void)
{
    return s_response_queue;
}
