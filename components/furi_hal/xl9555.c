/**
 * @file xl9555.c
 * XL9555 I2C GPIO Expander Driver Implementation
 *
 * Provides basic pin-level I/O for the XL9555 16-bit GPIO expander.
 * Used on the T-LoRa Pager for peripheral power control.
 *
 * Thread safety: all public functions use a mutex to protect I2C transactions.
 */

#include "xl9555.h"

#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static const char* TAG = "XL9555";

/* Module state */
static bool xl9555_initialized = false;
static i2c_port_t xl9555_port = I2C_NUM_0;
static uint8_t xl9555_addr = 0x20;
static SemaphoreHandle_t xl9555_mutex = NULL;

/* Shadow registers for output ports (read-modify-write) */
static uint8_t xl9555_output_shadow[2] = {0xFF, 0xFF}; /* Default: all HIGH */
static uint8_t xl9555_config_shadow[2] = {0xFF, 0xFF};  /* Default: all inputs */

/* I2C timeout */
#define XL9555_I2C_TIMEOUT_MS   100

static esp_err_t xl9555_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(
        xl9555_port, xl9555_addr, buf, sizeof(buf),
        pdMS_TO_TICKS(XL9555_I2C_TIMEOUT_MS));
}

static esp_err_t xl9555_read_reg(uint8_t reg, uint8_t* value) {
    return i2c_master_write_read_device(
        xl9555_port, xl9555_addr, &reg, 1, value, 1,
        pdMS_TO_TICKS(XL9555_I2C_TIMEOUT_MS));
}

static void xl9555_lock(void) {
    if(xl9555_mutex) {
        xSemaphoreTake(xl9555_mutex, portMAX_DELAY);
    }
}

static void xl9555_unlock(void) {
    if(xl9555_mutex) {
        xSemaphoreGive(xl9555_mutex);
    }
}

esp_err_t xl9555_init(i2c_port_t port, uint8_t addr,
                      int sda_pin, int scl_pin, uint32_t freq_hz) {
    if(xl9555_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    xl9555_port = port;
    xl9555_addr = addr;

    /* Initialize I2C bus */
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t err = i2c_param_config(port, &i2c_conf);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE = driver already installed (shared bus) */
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Create mutex */
    if(!xl9555_mutex) {
        xl9555_mutex = xSemaphoreCreateMutex();
        if(!xl9555_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* Verify device is present by reading input port 0 */
    uint8_t test_val = 0;
    err = xl9555_read_reg(XL9555_REG_INPUT_PORT0, &test_val);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Device not responding at 0x%02X: %s", addr, esp_err_to_name(err));
        return err;
    }

    /* Read current config and output registers to sync shadows */
    xl9555_read_reg(XL9555_REG_CONFIG_PORT0, &xl9555_config_shadow[0]);
    xl9555_read_reg(XL9555_REG_CONFIG_PORT1, &xl9555_config_shadow[1]);
    xl9555_read_reg(XL9555_REG_OUTPUT_PORT0, &xl9555_output_shadow[0]);
    xl9555_read_reg(XL9555_REG_OUTPUT_PORT1, &xl9555_output_shadow[1]);

    xl9555_initialized = true;
    ESP_LOGI(TAG, "Initialized at I2C addr 0x%02X (SDA=%d, SCL=%d, %lu Hz)",
             addr, sda_pin, scl_pin, (unsigned long)freq_hz);
    ESP_LOGI(TAG, "Config: Port0=0x%02X Port1=0x%02X, Output: Port0=0x%02X Port1=0x%02X",
             xl9555_config_shadow[0], xl9555_config_shadow[1],
             xl9555_output_shadow[0], xl9555_output_shadow[1]);

    return ESP_OK;
}

esp_err_t xl9555_config_pin(uint8_t pin, bool is_input) {
    if(!xl9555_initialized) return ESP_ERR_INVALID_STATE;
    if(pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t port_idx = pin / 8;
    uint8_t bit_pos = pin % 8;
    uint8_t config_reg = (port_idx == 0) ? XL9555_REG_CONFIG_PORT0 : XL9555_REG_CONFIG_PORT1;

    xl9555_lock();

    if(is_input) {
        xl9555_config_shadow[port_idx] |= (1U << bit_pos);
    } else {
        xl9555_config_shadow[port_idx] &= ~(1U << bit_pos);
    }

    esp_err_t err = xl9555_write_reg(config_reg, xl9555_config_shadow[port_idx]);

    xl9555_unlock();

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Config pin %d failed: %s", pin, esp_err_to_name(err));
    }
    return err;
}

esp_err_t xl9555_write_pin(uint8_t pin, bool level) {
    if(!xl9555_initialized) return ESP_ERR_INVALID_STATE;
    if(pin > 15) return ESP_ERR_INVALID_ARG;

    uint8_t port_idx = pin / 8;
    uint8_t bit_pos = pin % 8;
    uint8_t output_reg = (port_idx == 0) ? XL9555_REG_OUTPUT_PORT0 : XL9555_REG_OUTPUT_PORT1;

    xl9555_lock();

    if(level) {
        xl9555_output_shadow[port_idx] |= (1U << bit_pos);
    } else {
        xl9555_output_shadow[port_idx] &= ~(1U << bit_pos);
    }

    esp_err_t err = xl9555_write_reg(output_reg, xl9555_output_shadow[port_idx]);

    xl9555_unlock();

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Write pin %d failed: %s", pin, esp_err_to_name(err));
    }
    return err;
}

esp_err_t xl9555_read_pin(uint8_t pin, bool* level) {
    if(!xl9555_initialized) return ESP_ERR_INVALID_STATE;
    if(pin > 15 || !level) return ESP_ERR_INVALID_ARG;

    uint8_t port_idx = pin / 8;
    uint8_t bit_pos = pin % 8;
    uint8_t input_reg = (port_idx == 0) ? XL9555_REG_INPUT_PORT0 : XL9555_REG_INPUT_PORT1;

    xl9555_lock();

    uint8_t reg_val = 0;
    esp_err_t err = xl9555_read_reg(input_reg, &reg_val);
    if(err == ESP_OK) {
        *level = (reg_val & (1U << bit_pos)) != 0;
    }

    xl9555_unlock();

    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Read pin %d failed: %s", pin, esp_err_to_name(err));
    }
    return err;
}

esp_err_t xl9555_write_port(uint8_t port_reg, uint8_t value) {
    if(!xl9555_initialized) return ESP_ERR_INVALID_STATE;

    xl9555_lock();

    esp_err_t err = xl9555_write_reg(port_reg, value);
    if(err == ESP_OK) {
        if(port_reg == XL9555_REG_OUTPUT_PORT0) {
            xl9555_output_shadow[0] = value;
        } else if(port_reg == XL9555_REG_OUTPUT_PORT1) {
            xl9555_output_shadow[1] = value;
        }
    }

    xl9555_unlock();
    return err;
}

esp_err_t xl9555_read_port(uint8_t port_reg, uint8_t* value) {
    if(!xl9555_initialized || !value) return ESP_ERR_INVALID_STATE;

    xl9555_lock();
    esp_err_t err = xl9555_read_reg(port_reg, value);
    xl9555_unlock();

    return err;
}

bool xl9555_is_initialized(void) {
    return xl9555_initialized;
}
