#pragma once
/*
 * HAL I2C - Header
 * Purpose: Peripheral abstraction for I2C used by device drivers (e.g., OLED, motor controller).
 * Notes: Place your enums/structs/defines and function prototypes for init/deinit and transfers here.
 * Usage: Drivers include with: #include "hal_i2c.h"
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"


/* I2C communication speed */
#define I2C_FREQ_HZ  (400000U) /* 400kHz */

/* I2C GPIOs used for OLED */
#define GPIO_I2C_SCL (22U)
#define GPIO_I2C_SDA (21U)

/* I2C Port */
#define I2C_BUS_PORT I2C_NUM_0

/* Default transfer timeout (ms) for blocking transactions. */
#define HAL_I2C_TIMEOUT_MS (100U)

/* I2C bus handle */
extern i2c_master_bus_handle_t i2c_bus_handle; 


/* I2C bus initialisation. */
void hal_i2c_init(void);

/* Attach a 7-bit addressed device to the shared bus and return its handle. */
esp_err_t hal_i2c_add_device(uint8_t dev_address, i2c_master_dev_handle_t *dev_handle);

/* Transmit a raw buffer to a device with a configurable timeout. */
esp_err_t hal_i2c_master_write(i2c_master_dev_handle_t dev_handle, const uint8_t *data, size_t length, TickType_t timeout_ticks);

/* Receive a raw buffer from a device with a configurable timeout. */
esp_err_t hal_i2c_master_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t length, TickType_t timeout_ticks);

/* Combined write followed by read with a repeated start (no stop between). */
esp_err_t hal_i2c_master_write_read(i2c_master_dev_handle_t dev_handle,
                                    const uint8_t *write_buf,
                                    size_t write_len,
                                    uint8_t *read_buf,
                                    size_t read_len,
                                    TickType_t timeout_ticks);

/* Register master event callbacks (e.g., on_recv_done) for a given device. */
esp_err_t hal_i2c_master_register_callbacks(i2c_master_dev_handle_t dev_handle,
                                            const i2c_master_event_callbacks_t *cbs,
                                            void *user_data);
