/*
 * HAL I2C - Header
 * Purpose: Peripheral abstraction for I2C used by device drivers (e.g., OLED, motor controller).
 * Notes: Place your enums/structs/defines and function prototypes for init/deinit and transfers here.
 * Usage: Drivers include with: #include "hal_i2c.h"
 */

/* I2C communication speed */
#define I2C_FREQ_HZ  (400000U) /* 400kHz */