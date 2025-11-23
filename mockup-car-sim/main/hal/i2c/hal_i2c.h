/*
 * HAL I2C - Header
 * Purpose: Peripheral abstraction for I2C used by device drivers (e.g., OLED, motor controller).
 * Notes: Place your enums/structs/defines and function prototypes for init/deinit and transfers here.
 * Usage: Drivers include with: #include "hal_i2c.h"
 */

#include "driver/i2c_master.h"


/* I2C communication speed */
#define I2C_FREQ_HZ  (400000U) /* 400kHz */

/* I2C GPIOs used for OLED */
#define GPIO_I2C_SCL (22U)
#define GPIO_I2C_SDA (21U)

/* I2C Port */
#define I2C_BUS_PORT I2C_NUM_0

/* I2C bus handle */
extern i2c_master_bus_handle_t i2c_bus_handle; 


/* I2C bus initialisation. */
void hal_i2c_init(void);