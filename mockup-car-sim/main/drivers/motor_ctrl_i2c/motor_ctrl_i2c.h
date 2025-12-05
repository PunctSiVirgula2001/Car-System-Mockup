#pragma once
/*
 * Motor Control over I2C - Header
 * Purpose: Declare interfaces to communicate with dsPIC motor controller over I2C.
 * Notes: Add command IDs, message formats, and function prototypes here.
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "motor_ctrl_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/* Maximum payload bytes that can follow a command byte when sending. */
#define MOTOR_CTRL_MAX_DATA_LEN      (8U)

/* Maximum bytes expected back from the motor controller for queries. */
#define MOTOR_CTRL_MAX_RESP_LEN      (8U)

/* Timeout for each blocking I2C transaction with the motor controller. */
#define MOTOR_CTRL_I2C_TIMEOUT_MS    (100U)

/* Response queue length created by default in motor_ctrl_init (single-slot overwrite). */
#define MOTOR_CTRL_RESPONSE_QUEUE_LEN (5U)

typedef struct
{
    motor_ctrl_command_t command;
    size_t length;
    uint8_t data[MOTOR_CTRL_MAX_RESP_LEN];
} motor_ctrl_response_t;

/* -------------------------------------------------------------------------- */
/* Initialization                                                            */
/* -------------------------------------------------------------------------- */

/* Initialize motor control channel on the shared I2C bus and enable the driver. */
esp_err_t motor_ctrl_init(void);

/* -------------------------------------------------------------------------- */
/* Generic command helpers                                                   */
/* -------------------------------------------------------------------------- */

/* Send a command byte followed by an optional payload. */
esp_err_t motor_ctrl_send_command(motor_ctrl_command_t command, const uint8_t *data, size_t data_length);

/* Trigger a read to receive a response; callback/queue will be used on completion. */
esp_err_t motor_ctrl_receive_response(motor_ctrl_command_t command_tag,
                                      uint8_t *rx_data,
                                      size_t rx_length);

/* Get the response queue handle created/used by the driver. */
QueueHandle_t motor_ctrl_get_response_queue(void);

/* -------------------------------------------------------------------------- */
/* Convenience setters                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t motor_ctrl_set_speed(uint8_t speed_percent);
esp_err_t motor_ctrl_set_torque(uint8_t torque_percent);
esp_err_t motor_ctrl_set_direction(uint8_t direction);
esp_err_t motor_ctrl_set_acceleration(uint8_t accel);
