/*
 * HAL I2C - Source
 * Purpose: Implement shared I2C operations; wraps ESP-IDF I2C or other MCU drivers.
 * Notes: Add initialization, write/read helpers, optional scan, and concurrency handling.
 */

#include "project_config.h"
#include "hal_i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "hal_i2c";

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
        .trans_queue_depth = 0, /* Use synchronous transactions; async queue not needed */
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));
    ESP_LOGI(TAG, "I2C master bus created on port %d", I2C_BUS_PORT);
}

esp_err_t hal_i2c_add_device(uint8_t dev_address, i2c_master_dev_handle_t *dev_handle)
{
    if (dev_handle == NULL || i2c_bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = 
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_address,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    return i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, dev_handle);
}

esp_err_t hal_i2c_master_write(i2c_master_dev_handle_t dev_handle, const uint8_t *data, size_t length, TickType_t timeout_ticks)
{
    if ((dev_handle == NULL) || (data == NULL) || (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit(dev_handle, data, length, timeout_ticks);
}

esp_err_t hal_i2c_master_read(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t length, TickType_t timeout_ticks)
{
    if ((dev_handle == NULL) || (data == NULL) || (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_receive(dev_handle, data, length, timeout_ticks);
}

esp_err_t hal_i2c_master_write_read(i2c_master_dev_handle_t dev_handle,
                                    const uint8_t *write_buf,
                                    size_t write_len,
                                    uint8_t *read_buf,
                                    size_t read_len,
                                    TickType_t timeout_ticks)
{
    if ((dev_handle == NULL) || (write_buf == NULL) || (write_len == 0U) || (read_buf == NULL) || (read_len == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(dev_handle, write_buf, write_len, read_buf, read_len, timeout_ticks);
}

esp_err_t hal_i2c_master_register_callbacks(i2c_master_dev_handle_t dev_handle,
                                            const i2c_master_event_callbacks_t *cbs,
                                            void *user_data)
{
    if (dev_handle == NULL || cbs == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_register_event_callbacks(dev_handle, cbs, user_data);
}
