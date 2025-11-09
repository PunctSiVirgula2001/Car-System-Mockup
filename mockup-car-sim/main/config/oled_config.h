#pragma once
/*
 * OLED Driver Config - Header
 * Purpose: Compile-time configuration for the OLED device (I2C address, width/height, rotation, timings).
 * Notes: Keep device-specific constants here; no code. Pins belong in board_pins.h.
 * Usage: Included indirectly via project_config.h in oled_i2c.c.
 */

/* I2C GPIOs used for OLED */
#define GPIO_OLED_I2C_SCL (22U)
#define GPIO_OLED_I2C_SDA (21U)

