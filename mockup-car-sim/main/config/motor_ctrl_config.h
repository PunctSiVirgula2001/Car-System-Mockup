#pragma once
/*
 * Motor Control (I2C) Config - Header
 * Purpose: Compile-time settings for dsPIC motor controller protocol (I2C address, command IDs, timeouts).
 * Notes: Put only constants/macros here; pins go to board_pins.h if needed.
 * Usage: Included indirectly via project_config.h in motor_ctrl_i2c.c.
 */

#define MOTOR_CTRL_I2C_ADDRESS        (0x69U)  /* I2C address of the motor controller dsPIC device */

typedef enum {
    MOTOR_CMD_ENABLE_DRIVER        = 0x01U,
    MOTOR_CMD_DISABLE_DRIVER       = 0x02U,
    MOTOR_CMD_SET_SPEED            = 0x03U,
    MOTOR_CMD_SET_TORQUE           = 0x04U,
    MOTOR_CMD_SET_DIRECTION        = 0x05U,
    MOTOR_CMD_SET_ACCELERATION     = 0x06U,
    MOTOR_CMD_ENABLE_BRAKE         = 0x07U,
    MOTOR_CMD_DISABLE_BRAKE        = 0x08U,
    
    MOTOR_CMD_QUERY_ENABLE_STATUS  = 0x11U,
    MOTOR_CMD_QUERY_SPEED          = 0x13U,
    MOTOR_CMD_QUERY_TORQUE         = 0x14U,
    MOTOR_CMD_QUERY_DIRECTION      = 0x15U,
    MOTOR_CMD_QUERY_ACCELERATION   = 0x16U,
    MOTOR_CMD_QUERY_BRAKE_STATUS   = 0x17U,
    MOTOR_CMD_QUERY_PWM_FREQUENCY  = 0x19U
} motor_ctrl_command_t;
