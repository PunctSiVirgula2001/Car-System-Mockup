/*
 * HAL I2C - Source
 * Purpose: Implement shared I2C operations; wraps ESP-IDF I2C or other MCU drivers.
 * Notes: Add initialization, write/read helpers, optional scan, and concurrency handling.
 */

#include "project_config.h"
#include "hal_i2c.h"

/* I2C bus handle. */
i2c_master_bus_handle_t i2c_bus_handle = NULL;

void hal_i2c_init(void)
{

    i2c_master_bus_config_t bus_config = 
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
}