/**
 * @file xl9555.h
 * XL9555 I2C GPIO Expander Driver
 *
 * The XL9555 is a 16-bit I/O expander with I2C interface.
 * It has two 8-bit ports (Port 0 and Port 1), each pin configurable
 * as input or output. The T-LoRa Pager uses it to control power
 * enables and reset lines for multiple peripherals.
 *
 * Pin numbering: 0-7 = Port 0, 8-15 = Port 1
 * (matches the XL9555 datasheet GPIO numbering used in the board header)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* XL9555 register addresses */
#define XL9555_REG_INPUT_PORT0      0x00
#define XL9555_REG_INPUT_PORT1      0x01
#define XL9555_REG_OUTPUT_PORT0     0x02
#define XL9555_REG_OUTPUT_PORT1     0x03
#define XL9555_REG_POLARITY_PORT0   0x04
#define XL9555_REG_POLARITY_PORT1   0x05
#define XL9555_REG_CONFIG_PORT0     0x06    /* 1=input, 0=output */
#define XL9555_REG_CONFIG_PORT1     0x07

/**
 * Initialize the XL9555 GPIO expander.
 *
 * Sets up the I2C bus (if not already initialized) and configures
 * pin directions for the T-LoRa Pager's peripheral power control.
 *
 * @param port    I2C port number
 * @param addr    7-bit I2C address (typically 0x20)
 * @param sda_pin GPIO number for SDA
 * @param scl_pin GPIO number for SCL
 * @param freq_hz I2C bus frequency
 * @return ESP_OK on success
 */
esp_err_t xl9555_init(i2c_port_t port, uint8_t addr,
                      int sda_pin, int scl_pin, uint32_t freq_hz);

/**
 * Set an output pin HIGH or LOW.
 *
 * @param pin   Pin number (0-15)
 * @param level true = HIGH, false = LOW
 * @return ESP_OK on success
 */
esp_err_t xl9555_write_pin(uint8_t pin, bool level);

/**
 * Read the current state of a pin.
 *
 * @param pin    Pin number (0-15)
 * @param level  Output: true if pin is HIGH
 * @return ESP_OK on success
 */
esp_err_t xl9555_read_pin(uint8_t pin, bool* level);

/**
 * Configure a pin as input or output.
 *
 * @param pin       Pin number (0-15)
 * @param is_input  true = input, false = output
 * @return ESP_OK on success
 */
esp_err_t xl9555_config_pin(uint8_t pin, bool is_input);

/**
 * Write a full 8-bit value to an output port register.
 *
 * @param port_reg  Register address (XL9555_REG_OUTPUT_PORT0 or _PORT1)
 * @param value     8-bit value to write
 * @return ESP_OK on success
 */
esp_err_t xl9555_write_port(uint8_t port_reg, uint8_t value);

/**
 * Read a full 8-bit value from an input/output port register.
 *
 * @param port_reg  Register address
 * @param value     Output: 8-bit value read
 * @return ESP_OK on success
 */
esp_err_t xl9555_read_port(uint8_t port_reg, uint8_t* value);

/**
 * Check if the XL9555 has been successfully initialized.
 *
 * @return true if initialized
 */
bool xl9555_is_initialized(void);

#ifdef __cplusplus
}
#endif
