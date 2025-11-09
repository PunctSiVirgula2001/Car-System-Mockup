#pragma once
/*
 * Motor Control (I2C) Config - Header
 * Purpose: Compile-time settings for dsPIC motor controller protocol (I2C address, command IDs, timeouts).
 * Notes: Put only constants/macros here; pins go to board_pins.h if needed.
 * Usage: Included indirectly via project_config.h in motor_ctrl_i2c.c.
 */

/* I2C GPIOs used for DSPIC */
#define GPIO_DSPIC_I2C_SCL (22U)
#define GPIO_DSPIC_I2C_SDA (21U)