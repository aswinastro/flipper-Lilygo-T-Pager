#include "furi_hal.h"
#include "boards/board.h"
#include <furi_hal_gpio.h>
#include <esp_log.h>
#include <nvs_flash.h>

#if defined(BOARD_HAS_XL9555) && BOARD_HAS_XL9555
#include "xl9555.h"
#endif

static const char* TAG = "FuriHal";

void furi_hal_init_early(void) {
    furi_hal_cortex_init_early();

#if defined(BOARD_HAS_XL9555) && BOARD_HAS_XL9555
    /* Initialize XL9555 GPIO expander on shared I2C bus */
    esp_err_t xl_err = xl9555_init(
        BOARD_I2C_PORT, BOARD_XL9555_I2C_ADDR,
        BOARD_I2C_SDA, BOARD_I2C_SCL, BOARD_I2C_FREQ_HZ);
    if(xl_err == ESP_OK) {
        /* Configure SD Card power pin as output and enable SD power */
        xl9555_config_pin(XL9555_PIN_SD_PWR_EN, false);
        xl9555_write_pin(XL9555_PIN_SD_PWR_EN, true);
        ESP_LOGI(TAG, "XL9555 initialized: SD card power enabled (GPIO%d=HIGH)", XL9555_PIN_SD_PWR_EN);

        /* Configure SD card detect as input */
        xl9555_config_pin(XL9555_PIN_SD_DETECT, true);
    } else {
        ESP_LOGE(TAG, "XL9555 init failed: %s", esp_err_to_name(xl_err));
    }
#endif

#if defined(BOARD_SPI_SCK)
    /* Shared SPI bus on T-LoRa Pager: ensure all peripheral CS pins are initialized HIGH (inactive) */
    static const GpioPin cs_display = {.port = NULL, .pin = BOARD_PIN_LCD_CS};
    static const GpioPin cs_sd = {.port = NULL, .pin = BOARD_PIN_SD_CS};
    furi_hal_gpio_init_simple(&cs_display, GpioModeOutputPushPull);
    furi_hal_gpio_write(&cs_display, true);
    furi_hal_gpio_init_simple(&cs_sd, GpioModeOutputPushPull);
    furi_hal_gpio_write(&cs_sd, true);

#ifdef BOARD_PIN_LORA_CS
    static const GpioPin cs_lora = {.port = NULL, .pin = BOARD_PIN_LORA_CS};
    furi_hal_gpio_init_simple(&cs_lora, GpioModeOutputPushPull);
    furi_hal_gpio_write(&cs_lora, true);
#endif

#ifdef BOARD_PIN_NFC_CS
    static const GpioPin cs_nfc = {.port = NULL, .pin = BOARD_PIN_NFC_CS};
    furi_hal_gpio_init_simple(&cs_nfc, GpioModeOutputPushPull);
    furi_hal_gpio_write(&cs_nfc, true);
#endif
    ESP_LOGI(TAG, "Shared SPI CS lines initialized HIGH (Display=%d, SD=%d, LoRa=%d, NFC=%d)",
             BOARD_PIN_LCD_CS, BOARD_PIN_SD_CS, BOARD_PIN_LORA_CS, BOARD_PIN_NFC_CS);
#endif

#ifdef BOARD_PIN_PWR_EN
    /* Power-enable must be set early — powers CC1101, BQ27220 fuel gauge, WS2812 */
    static const GpioPin pwr_en = {.port = NULL, .pin = BOARD_PIN_PWR_EN};
    furi_hal_gpio_init_simple(&pwr_en, GpioModeOutputPushPull);
    furi_hal_gpio_write(&pwr_en, true);
    ESP_LOGI(TAG, "PWR_EN GPIO%d set HIGH", BOARD_PIN_PWR_EN);
#endif

#ifdef BOARD_PIN_NRF24_CSN
    /* T-Embed Plus shares SPI2 between CC1101 and NRF24. Drive NRF24 CSN HIGH
     * (deselected) and CE LOW (standby) at boot, before any CC1101 SPI traffic.
     * Without this, NRF24 sees CC1101 traffic and corrupts the bus, manifesting
     * as a stuck ~312 MHz reading in the Frequency Analyzer and total RX failure. */
    static const GpioPin nrf24_csn = {.port = NULL, .pin = BOARD_PIN_NRF24_CSN};
    furi_hal_gpio_init_simple(&nrf24_csn, GpioModeOutputPushPull);
    furi_hal_gpio_write(&nrf24_csn, true);
    ESP_LOGI(TAG, "NRF24_CSN GPIO%d set HIGH (deselect)", BOARD_PIN_NRF24_CSN);
#endif

#ifdef BOARD_PIN_NRF24_CE
    static const GpioPin nrf24_ce = {.port = NULL, .pin = BOARD_PIN_NRF24_CE};
    furi_hal_gpio_init_simple(&nrf24_ce, GpioModeOutputPushPull);
    furi_hal_gpio_write(&nrf24_ce, false);
    ESP_LOGI(TAG, "NRF24_CE GPIO%d set LOW (standby)", BOARD_PIN_NRF24_CE);
#endif

    ESP_LOGI(TAG, "Early init complete");
}

void furi_hal_deinit_early(void) {
}

void furi_hal_init(void) {
    /* NVS is required by WiFi and BLE — init once at boot */
    esp_err_t nvs_err = nvs_flash_init();
    if(nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    furi_hal_rtc_init();
    furi_hal_version_init();
    furi_hal_power_init();
    furi_hal_crypto_init();
    furi_hal_subghz_init();
    furi_hal_usb_init();
    furi_hal_light_init();
    furi_hal_display_init();
    furi_hal_speaker_init();
    furi_hal_nfc_init();
    ESP_LOGI(TAG, "Init complete");
}
