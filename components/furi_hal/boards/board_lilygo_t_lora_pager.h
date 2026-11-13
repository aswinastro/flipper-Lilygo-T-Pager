/**
 * @file board_lilygo_t_lora_pager.h
 * Board definition: LilyGo T-LoRa Pager
 *
 * MCU:      ESP32-S3 (dual-core Xtensa LX7)
 * Flash:    16 MB
 * PSRAM:    8 MB
 * Display:  ST7796U 480x222 RGB565 via SPI
 * Input:    Rotary Encoder (A/B/Button) + TCA8418 keyboard (optional)
 * Radio:    SX1262/SX1280 via SPI (NOT CC1101 — disabled for now)
 * NFC:      ST25R3916 via SPI (NOT PN532 — disabled for now)
 * SD Card:  SPI (shared bus with LCD + LoRa + NFC)
 * Audio:    ES8311 I2S codec
 * Battery:  BQ25896 charger + BQ27220 gauge (I2C)
 * Haptic:   DRV2605 (I2C)
 * GNSS:     MIA-M10Q (UART)
 * GPIO Exp: XL9555 (I2C) — controls power enables for most peripherals
 */

#pragma once

#include <driver/i2c.h>
#include <driver/spi_master.h>

/* ---- Board metadata ---- */
#define BOARD_LILYGO_T_LORA_PAGER 1
#define BOARD_NAME        "LilyGo T-LoRa Pager"
#define BOARD_TARGET      "esp32s3"

/* ---- Hardware Button / Encoder Pins ---- */
#define BOARD_PIN_BUTTON_BOOT   0   /* Boot/User button (GPIO0, active low) */
#define BOARD_PIN_BUTTON_KEY    UINT16_MAX  /* No dedicated side key — keyboard replaces it */
#define BOARD_PIN_BATTERY_ADC   UINT16_MAX  /* Battery via I2C gauge, no ADC pin */

/* ---- Rotary Encoder ---- */
#define BOARD_PIN_ENCODER_A     40
#define BOARD_PIN_ENCODER_B     41
#define BOARD_PIN_ENCODER_BTN   7   /* Encoder center press */

/* ---- LCD Pins (ST7796U via SPI) ---- */
#define BOARD_PIN_LCD_MOSI      34  /* Shared SPI MOSI */
#define BOARD_PIN_LCD_SCLK      35  /* Shared SPI SCK */
#define BOARD_PIN_LCD_DC        37  /* Data/Command */
#define BOARD_PIN_LCD_CS        38  /* Display Chip Select */
#define BOARD_PIN_LCD_RST       UINT16_MAX  /* NOT CONNECTED — use software reset only */
#define BOARD_PIN_LCD_BL        42  /* Backlight PWM */

/* ---- LCD Display Configuration ---- */
#define BOARD_LCD_H_RES         480     /* Native width (landscape orientation) */
#define BOARD_LCD_V_RES         222     /* Visible height of the 2.33" panel */
#define BOARD_LCD_SPI_HOST      SPI2_HOST
#define BOARD_LCD_SPI_FREQ_HZ   (40 * 1000 * 1000)
#define BOARD_LCD_CMD_BITS      8
#define BOARD_LCD_PARAM_BITS    8
#define BOARD_LCD_SWAP_XY       true    /* Landscape orientation */
#define BOARD_LCD_MIRROR_X      true    /* 180° rotation (X inverted) */
#define BOARD_LCD_MIRROR_Y      true    /* 180° rotation (Y inverted) */
#define BOARD_LCD_INVERT_COLOR  false   /* ST7796U IPS panel (inversion off) */
#define BOARD_LCD_GAP_X         0       /* 480 columns are fully active */
#define BOARD_LCD_GAP_Y         49      /* Fixed: ST7796 320-line RAM offset: (320 - 222) / 2 = 49 */
#define BOARD_LCD_BL_ACTIVE_LOW false   /* Backlight is active-high */
#define BOARD_LCD_COLOR_ORDER_BGR false /* RGB order */
#define BOARD_LCD_SIDE_MARGIN   6       /* Side margin for 480px width */

/* The ST7796U needs its own panel driver — NOT st7789 */
#define BOARD_LCD_PANEL_ST7796  1

/* Flipper framebuffer → display color mapping (RGB565, native byte order) */
#define BOARD_LCD_FG_COLOR      0xA0FD  /* Flipper Orange 0xFDA0 byte-swapped for S3 SPI */
#define BOARD_LCD_FG_COLOR_RB   0x5F03  /* Same orange with R/B swapped */
#define BOARD_LCD_BG_COLOR      0x0000  /* Black */

/* ---- Shared SPI Bus Pins ---- */
/* Display, SD Card, LoRa, NFC all share this SPI bus */
#define BOARD_SPI_SCK           35
#define BOARD_SPI_MOSI          34
#define BOARD_SPI_MISO          33

/* ---- SD Card Pins (shared SPI bus) ---- */
#define BOARD_PIN_SD_CS         21
#define BOARD_PIN_SD_MISO       33  /* Shared MISO */

/* ---- Touch Controller — NOT PRESENT (encoder + keyboard input) ---- */
#define BOARD_PIN_TOUCH_SCL     UINT16_MAX
#define BOARD_PIN_TOUCH_SDA     UINT16_MAX
#define BOARD_PIN_TOUCH_RST     UINT16_MAX
#define BOARD_PIN_TOUCH_INT     UINT16_MAX
#define BOARD_TOUCH_I2C_ADDR    0x00
#define BOARD_TOUCH_I2C_PORT    I2C_NUM_0
#define BOARD_TOUCH_I2C_FREQ_HZ 0
#define BOARD_TOUCH_I2C_TIMEOUT 0

/* ---- SubGHz / External CC1101 Module ---- */
/* The T-LoRa Pager has a built-in SX1262 LoRa radio, but an EXTERNAL CC1101
 * module can be connected to the expansion header for Sub-GHz support.
 *
 * The CC1101 shares the main SPI bus (SPI2_HOST) with the display, SD card,
 * LoRa, and NFC via CS-based muxing. The firmware's SPI bus lock serializes
 * all transactions automatically.
 *
 * Wiring for external CC1101 module:
 *   VCC  → 3.3V ONLY (do NOT use 5V)
 *   GND  → GND
 *   SCK  → GPIO35 (shared SPI SCK on expansion header)
 *   MOSI → GPIO34 (shared SPI MOSI on expansion header)
 *   MISO → GPIO33 (shared SPI MISO on expansion header)
 *   CSN  → GPIO9  (expansion header "custom" pin)
 *   GDO0 → GPIO43 (expansion header TX pin — safe: console uses USB-Serial-JTAG)
 */
#define BOARD_PIN_CC1101_SCK    BOARD_SPI_SCK   /* 35 — shared SPI bus */
#define BOARD_PIN_CC1101_CSN    9               /* Expansion header "custom" pin */
#define BOARD_PIN_CC1101_MISO   BOARD_SPI_MISO  /* 33 — shared SPI bus */
#define BOARD_PIN_CC1101_MOSI   BOARD_SPI_MOSI  /* 34 — shared SPI bus */
#define BOARD_PIN_CC1101_GDO0   43              /* Expansion header TX — CC1101 interrupt */
#define BOARD_PIN_CC1101_GDO2   UINT16_MAX      /* Not connected */
#define BOARD_PIN_CC1101_SW1    UINT16_MAX      /* No RF switch on external module */
#define BOARD_PIN_CC1101_SW0    UINT16_MAX      /* No RF switch on external module */
#define BOARD_CC1101_SPI_SHARED 1               /* CC1101 shares SPI2_HOST with LCD+SD+LoRa */

/* LoRa radio pins (SX1262/SX1280) — for future driver */
#define BOARD_PIN_LORA_CS       36
#define BOARD_PIN_LORA_DIO1     14  /* Interrupt */
#define BOARD_PIN_LORA_RESET    47
#define BOARD_PIN_LORA_BUSY     48
/* LoRa power enable: XL9555 GPIO3 */

/* ---- NFC ---- */
/* ST25R3916 via SPI — NOT PN532.
 * PN532 driver must NOT be initialized on this board.
 * NFC is disabled (BOARD_HAS_NFC=0) until a proper ST25R3916 driver is added. */
#define BOARD_PIN_NFC_SCL       UINT16_MAX  /* Not I2C on this board */
#define BOARD_PIN_NFC_SDA       UINT16_MAX
#define BOARD_PIN_NFC_IRQ       1           /* NFC interrupt GPIO */
#define BOARD_PIN_NFC_RST       UINT16_MAX
#define BOARD_NFC_I2C_PORT      I2C_NUM_0

/* NFC SPI pins (ST25R3916) — for future driver */
#define BOARD_PIN_NFC_CS        39
/* NFC power enable: XL9555 GPIO5 */

/* ---- Power Enable ---- */
/* No single power-enable pin like T-Embed's BOARD_PIN_PWR_EN.
 * Individual peripheral power is controlled through the XL9555 GPIO expander.
 * Do NOT define BOARD_PIN_PWR_EN so the init code skips the PWR_EN path. */

/* ---- IR — NOT PRESENT ---- */
#define BOARD_PIN_IR_TX         UINT16_MAX
#define BOARD_PIN_IR_RX         UINT16_MAX

/* ---- Speaker / Audio (ES8311 I2S) ---- */
#define BOARD_PIN_SPEAKER_BCLK  11
#define BOARD_PIN_SPEAKER_WCLK  18  /* WS / LRCK */
#define BOARD_PIN_SPEAKER_DOUT  45  /* I2S Data Out */
/* Additional audio pins:
 * MCLK = GPIO10
 * DATA IN = GPIO17
 * Amplifier enable: XL9555 GPIO1 */

/* ---- WS2812 RGB LED Strip — NOT PRESENT ---- */
/* Do NOT define BOARD_PIN_WS2812_DATA — guards in furi_hal_light.c skip WS2812 init */

/* ---- Microphone — NOT PRESENT (audio codec handles it) ---- */
/* Do NOT define BOARD_PIN_MIC_DATA / BOARD_PIN_MIC_CLK */

/* ---- I2C Bus (shared by XL9555, keyboard, battery, haptic) ---- */
#define BOARD_I2C_SCL           2
#define BOARD_I2C_SDA           3
#define BOARD_I2C_PORT          I2C_NUM_0
#define BOARD_I2C_FREQ_HZ       (400 * 1000)    /* 400 kHz Fast Mode */

/* ---- XL9555 GPIO Expander ---- */
#define BOARD_XL9555_I2C_ADDR   0x20
/* XL9555 pin assignments:
 * GPIO0  = Haptic driver enable (DRV2605)
 * GPIO1  = Audio amplifier enable
 * GPIO2  = Keyboard reset
 * GPIO3  = LoRa power enable
 * GPIO4  = GNSS power enable
 * GPIO5  = NFC power enable
 * GPIO7  = GNSS reset
 * GPIO9  = External CE
 * GPIO10 = Keyboard power enable
 * GPIO12 = SD card insert detect (input)
 * GPIO14 = SD card power enable
 */
#define XL9555_PIN_HAPTIC_EN    0
#define XL9555_PIN_AMP_EN       1
#define XL9555_PIN_KBD_RESET    2
#define XL9555_PIN_LORA_PWR_EN  3
#define XL9555_PIN_GNSS_PWR_EN  4
#define XL9555_PIN_NFC_PWR_EN   5
#define XL9555_PIN_GNSS_RESET   7
#define XL9555_PIN_EXT_CE       9
#define XL9555_PIN_KBD_PWR_EN   10
#define XL9555_PIN_SD_DETECT    12  /* Input: SD card insert detect */
#define XL9555_PIN_SD_PWR_EN    14

/* ---- Keyboard (TCA8418 I2C) ---- */
#define BOARD_PIN_KBD_INT       6   /* Keyboard interrupt GPIO */
/* Keyboard power/reset via XL9555 GPIO10/GPIO2 */

/* ---- GNSS (MIA-M10Q UART) ---- */
#define BOARD_PIN_GPS_RX        12
#define BOARD_PIN_GPS_TX        4
#define BOARD_PIN_GPS_PPS       13
/* GNSS power/reset via XL9555 GPIO4/GPIO7 */

/* ---- Qwiic / External I2C (shared with main I2C bus) ---- */
#define BOARD_PIN_QWIIC_SDA     3
#define BOARD_PIN_QWIIC_SCL     2

/* ---- External GPIO Header ---- */
/* Expansion header pins: TX=GPIO43, RX=GPIO44, custom=GPIO9
 * Also: I2C (SCL=2, SDA=3) and SPI (33/34/35) are broken out.
 *
 * Console uses USB-Serial-JTAG (internal PHY on GPIO19/20), so
 * GPIO43/44 are free for external module use.
 *
 * External RF module allocation (CC1101 and NRF24 share the same pins —
 * only one module is physically connected at a time):
 *   GPIO9  → CS / CSN (chip select for whichever module is connected)
 *   GPIO43 → GDO0 (CC1101 interrupt) / CE (NRF24 chip enable)
 *   GPIO44 → FREE (available for future use)
 */

/* ---- External NRF24L01 Module ---- */
/* Shares the main SPI bus AND the same CS/control pins as CC1101.
 * This works because CC1101 and NRF24 are NEVER connected simultaneously —
 * the user physically swaps modules on the same expansion header.
 *
 * Wiring for external NRF24L01(+) module (SAME pins as CC1101):
 *   VCC  → 3.3V ONLY (do NOT use 5V)
 *   GND  → GND
 *   SCK  → GPIO35 (shared SPI SCK)
 *   MOSI → GPIO34 (shared SPI MOSI)
 *   MISO → GPIO33 (shared SPI MISO)
 *   CSN  → GPIO9  (expansion header "custom" — same as CC1101 CS)
 *   CE   → GPIO43 (expansion header TX — same as CC1101 GDO0)
 *   IRQ  → not connected (not required)
 */
#define BOARD_PIN_NRF24_SCK     BOARD_SPI_SCK           /* 35 — shared */
#define BOARD_PIN_NRF24_MISO    BOARD_SPI_MISO          /* 33 — shared */
#define BOARD_PIN_NRF24_MOSI    BOARD_SPI_MOSI          /* 34 — shared */
#define BOARD_PIN_NRF24_CSN     BOARD_PIN_CC1101_CSN    /* 9 — shared CS (one module at a time) */
#define BOARD_PIN_NRF24_CE      BOARD_PIN_CC1101_GDO0   /* 43 — shared (one module at a time) */
#define BOARD_HAS_NRF24         1

/* ---- RFID — NOT PRESENT ---- */
#define BOARD_PIN_RFID_RX       UINT16_MAX
#define BOARD_PIN_RFID_TX       UINT16_MAX
#define BOARD_RFID_UART_NUM     1

/* ---- Features ---- */
#define BOARD_HAS_TOUCH         0
#define BOARD_HAS_ENCODER       1
#define BOARD_HAS_SD_CARD       1
#define BOARD_HAS_BLE           1
#define BOARD_HAS_RGB_LED       0   /* No WS2812 */
#define BOARD_HAS_VIBRO         0   /* DRV2605 present but driver not implemented yet */
#define BOARD_HAS_SPEAKER       1
#define BOARD_HAS_IR            0   /* No IR hardware */
#define BOARD_HAS_IBUTTON       0
#define BOARD_HAS_RFID          0
#define BOARD_HAS_NFC           0   /* ST25R3916 present but driver not implemented yet */
#define BOARD_HAS_SUBGHZ        1   /* External CC1101 via expansion header */
#define BOARD_HAS_MIC           0
#define BOARD_HAS_KEYBOARD      0   /* TCA8418 present but driver not implemented yet */
#define BOARD_HAS_XL9555        1   /* XL9555 GPIO expander present */

/* ---- Battery Charger / Gauge ---- */
#define BQ27220_ADDR            0x55
#define BQ_I2C_PORT             I2C_NUM_0
#define BQ_I2C_SDA              BOARD_I2C_SDA
#define BQ_I2C_SCL              BOARD_I2C_SCL
#define HIGH_DRAIN_CURRENT_THRESHOLD    (-200)
#define FURI_HAL_POWER_VIRTUAL_CAPACITY_MAH (1300U)
#define BQ25896_CHARGE_LIMIT    1280
