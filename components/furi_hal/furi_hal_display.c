/**
 * @file furi_hal_display.c
 * Display HAL — board-driven configuration via boards/board.h
 *
 * Flipper GUI: 128x64 mono → aspect-fit scaled and centered on display
 *
 * Performance optimisations (2024):
 *  - Double-buffered DMA pipelining: CPU renders into buf A while DMA
 *    sends buf B, then swap. Eliminates CPU-idle-during-DMA stalls.
 *  - Margin repaint cached: only repaint margins when fg_color changes.
 *  - STRIPE_HEIGHT=16: halves the number of SPI transactions per frame.
 *  - Scaling loop: row deduplication (memcpy identical rows) + branchless
 *    pixel colour selection.
 */

#include "furi_hal_display.h"
#include "furi_hal_light.h"
#include "furi_hal_resources.h"
#include "furi_hal_spi_bus.h"
#include "boards/board.h"

#if defined(BOARD_LCD_PANEL_ST7796) && BOARD_LCD_PANEL_ST7796
#include <esp_lcd_st7796.h>
#endif

#include <string.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <freertos/semphr.h>

static const char* TAG = "FuriHalDisplay";

/* Display dimensions from board config */
#define LCD_H_RES BOARD_LCD_H_RES
#define LCD_V_RES BOARD_LCD_V_RES

/* Flipper framebuffer dimensions */
#define FB_WIDTH  128
#define FB_HEIGHT 64

/* Always-on left/right inset (in physical LCD pixels) kept as plain background
 * color so the UI never sits flush against the screen's left/right edges. A
 * board may override this in its board.h; otherwise this default applies
 * everywhere. Set to 0 to disable. */
#ifndef BOARD_LCD_SIDE_MARGIN
#define BOARD_LCD_SIDE_MARGIN 4
#endif
#define DISPLAY_SIDE_MARGIN BOARD_LCD_SIDE_MARGIN

/* Horizontal resolution actually available to the UI after reserving the
 * left/right margins. The aspect-fit below scales into this, then the result
 * is centered across the full LCD resolution — so the leftover on each side
 * is always >= the margin and is painted with the background color. */
#define USABLE_H_RES (LCD_H_RES - 2 * DISPLAY_SIDE_MARGIN)

/* Scale the 128x64 framebuffer to the largest centered size that keeps aspect ratio. */
#if (USABLE_H_RES * FB_HEIGHT) <= (LCD_V_RES * FB_WIDTH)
#define SCALED_WIDTH  USABLE_H_RES
#define SCALED_HEIGHT ((USABLE_H_RES * FB_HEIGHT) / FB_WIDTH)
#else
#define SCALED_HEIGHT LCD_V_RES
#define SCALED_WIDTH  ((LCD_V_RES * FB_WIDTH) / FB_HEIGHT)
#endif

/* Centering margins */
#define MARGIN_X ((LCD_H_RES - SCALED_WIDTH) / 2)
#define MARGIN_Y ((LCD_V_RES - SCALED_HEIGHT) / 2)

/* Colors from board config — both are set at runtime so the user can pick
 * UI Background (fg_color, the field that fills "unset" mono pixels) and
 * UI Foreground (bg_color, what fills "set" pixels = drawn UI elements). */
static uint16_t fg_color;
static uint16_t bg_color;

/* SPI configuration from board config */
#define LCD_SPI_HOST   BOARD_LCD_SPI_HOST
#define LCD_SPI_FREQ   BOARD_LCD_SPI_FREQ_HZ
#define LCD_CMD_BITS   BOARD_LCD_CMD_BITS
#define LCD_PARAM_BITS BOARD_LCD_PARAM_BITS

/* Stripe-based rendering: render & DMA-send N lines at a time.
 * 16 lines gives a good balance between DMA transaction count and buffer size.
 * With double-buffering each buffer is SCALED_WIDTH*16*2 = ~15KB (DMA-capable). */
#define STRIPE_HEIGHT 16

/* Number of double-buffer slots for DMA pipelining */
#define NUM_STRIPE_BUFS 2

static esp_lcd_panel_handle_t panel_handle = NULL;
static uint16_t* stripe_bufs[NUM_STRIPE_BUFS] = {NULL, NULL};
static SemaphoreHandle_t lcd_flush_done = NULL;
static uint8_t x_scale_lut[SCALED_WIDTH];
static uint8_t y_scale_lut[SCALED_HEIGHT];

/* Margin colour cache — only repaint margins when the colour actually changes */
static uint16_t margin_painted_color = 0xFFFF; /* impossible initial value forces first paint */

static bool furi_hal_display_flush_done_callback(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t* edata,
    void* user_ctx) {
    UNUSED(panel_io);
    UNUSED(edata);
    UNUSED(user_ctx);

    BaseType_t yield = pdFALSE;
    if(lcd_flush_done) {
        xSemaphoreGiveFromISR(lcd_flush_done, &yield);
    }

    return yield == pdTRUE;
}

static void furi_hal_display_prepare_flush(void) {
    if(!lcd_flush_done) return;

    xSemaphoreTake(lcd_flush_done, 0);
}

static void furi_hal_display_wait_flush(void) {
    if(!lcd_flush_done) return;

    if(xSemaphoreTake(lcd_flush_done, pdMS_TO_TICKS(250)) != pdTRUE) {
        ESP_LOGW(TAG, "Timed out waiting for LCD flush");
    }
}

static void furi_hal_display_init_scale_lut(void) {
    for(size_t x = 0; x < SCALED_WIDTH; x++) {
        x_scale_lut[x] = (x * FB_WIDTH) / SCALED_WIDTH;
    }

    for(size_t y = 0; y < SCALED_HEIGHT; y++) {
        y_scale_lut[y] = (y * FB_HEIGHT) / SCALED_HEIGHT;
    }
}

/* Fill the entire screen with a solid colour, using stripe-sized DMA chunks
 * instead of one-line-at-a-time. Used only during display init. */
static void display_fill_color(uint16_t color) {
    if(!stripe_bufs[0]) return;
    uint16_t* buf = stripe_bufs[0];

    const size_t fill_w = LCD_H_RES;
    const size_t fill_stride = fill_w * STRIPE_HEIGHT;

    /* Pre-fill the stripe buffer */
    for(size_t i = 0; i < fill_stride; i++) buf[i] = color;

    furi_hal_spi_bus_lock();
    for(int y = 0; y < LCD_V_RES; y += STRIPE_HEIGHT) {
        int chunk_h = STRIPE_HEIGHT;
        if(y + chunk_h > LCD_V_RES) chunk_h = LCD_V_RES - y;

        furi_hal_display_prepare_flush();
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, fill_w, y + chunk_h, buf);
        furi_hal_display_wait_flush();
    }
    furi_hal_spi_bus_unlock();
}

/* Paint a solid-color rectangle using stripe_bufs[0] as scratch. Caller must
 * hold the SPI bus lock. Chunks vertically at STRIPE_HEIGHT to stay within the
 * buffer's capacity. Used by the commit path to repaint margins on colour
 * change. */
static void display_paint_rect(
    size_t x, size_t y, size_t w, size_t h, uint16_t color) {
    if(w == 0 || h == 0 || w > LCD_H_RES) return;
    uint16_t* buf = stripe_bufs[0];

    for(size_t y_off = 0; y_off < h; y_off += STRIPE_HEIGHT) {
        size_t chunk_h = h - y_off;
        if(chunk_h > STRIPE_HEIGHT) chunk_h = STRIPE_HEIGHT;

        const size_t px_count = w * chunk_h;
        for(size_t i = 0; i < px_count; i++) buf[i] = color;

        furi_hal_display_prepare_flush();
        esp_lcd_panel_draw_bitmap(
            panel_handle, x, y + y_off, x + w, y + y_off + chunk_h, buf);
        furi_hal_display_wait_flush();
    }
}

void furi_hal_display_init(void) {
    ESP_LOGI(TAG, "Initializing display for %s", BOARD_NAME);
    furi_hal_spi_bus_init();
    if(!lcd_flush_done) {
        lcd_flush_done = xSemaphoreCreateBinary();
        ESP_ERROR_CHECK(lcd_flush_done ? ESP_OK : ESP_ERR_NO_MEM);
    }

    /* Initialize SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = gpio_lcd_din.pin,
        .miso_io_num = gpio_sdcard_miso.pin,
        .sclk_io_num = gpio_lcd_clk.pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * STRIPE_HEIGHT * sizeof(uint16_t),
    };
    esp_err_t spi_err = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    /* ESP_ERR_INVALID_STATE is OK — bus may already be initialized by SubGHz on shared-bus boards */
    if(spi_err != ESP_OK && spi_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(spi_err);
    }

    /* Create panel IO (SPI) — queue depth 2 enables DMA pipelining:
     * we can submit stripe N+1 while stripe N is still in flight. */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = gpio_lcd_dc.pin,
        .cs_gpio_num = gpio_lcd_cs.pin,
        .pclk_hz = LCD_SPI_FREQ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = NUM_STRIPE_BUFS,
        .on_color_trans_done = furi_hal_display_flush_done_callback,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &io_handle));

    /* Create panel */
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = (gpio_lcd_rst.pin != UINT16_MAX) ? (int)gpio_lcd_rst.pin : -1,
#if defined(BOARD_LCD_COLOR_ORDER_BGR) && BOARD_LCD_COLOR_ORDER_BGR
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
#else
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
#endif
        .bits_per_pixel = 16,
    };

    /* 1) Hardware reset pulse on RESX if connected */
    if(gpio_lcd_rst.pin != UINT16_MAX) {
        gpio_config_t rst_cfg = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << gpio_lcd_rst.pin,
        };
        gpio_config(&rst_cfg);
        gpio_set_level((gpio_num_t)gpio_lcd_rst.pin, 1);   /* ensure a clean falling edge */
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)gpio_lcd_rst.pin, 0);   /* assert reset (active low) */
        vTaskDelay(pdMS_TO_TICKS(20));                      /* RESX min low is 10us; be generous */
        gpio_set_level((gpio_num_t)gpio_lcd_rst.pin, 1);   /* release reset */
        vTaskDelay(pdMS_TO_TICKS(150));                     /* wait ≥120ms after reset */
    }

    /* 2) Software reset (0x01) — re-loads all registers to factory defaults */
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x01 /* SWRESET */, NULL, 0));
    vTaskDelay(pdMS_TO_TICKS(150));

#if defined(BOARD_LCD_PANEL_ST7796) && BOARD_LCD_PANEL_ST7796
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
#else
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
#endif

    /* 3) esp_lcd does another RESX pulse, then SLPOUT + COLMOD + MADCTL. */
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    /* Display orientation and corrections from board config (these (re)write
     * MADCTL with our colour order + mirror/swap bits, and INVON/INVOFF). */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, BOARD_LCD_INVERT_COLOR));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, BOARD_LCD_SWAP_XY));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, BOARD_LCD_MIRROR_X, BOARD_LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, BOARD_LCD_GAP_X, BOARD_LCD_GAP_Y));

    /* 4) Belt-and-suspenders: pin down pixel format + normal display mode.
     *    COLMOD 0x55 = 16 bits/pixel (RGB565) — matches bits_per_pixel=16 above;
     *    NORON (0x13) = normal display mode (not partial/idle). Both are no-ops
     *    when the state is already right and cost nothing. */
    {
        const uint8_t colmod = 0x55;
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x3A /* COLMOD */, &colmod, 1));
    }
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, 0x13 /* NORON */, NULL, 0));

    /* Turn on display */
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    fg_color = BOARD_LCD_FG_COLOR;
    bg_color = BOARD_LCD_BG_COLOR;

    furi_hal_display_init_scale_lut();

    /* Allocate DMA-capable RGB565 double-buffer stripe buffers.
     * Each buffer holds STRIPE_HEIGHT lines at SCALED_WIDTH. Sized to the
     * larger of SCALED_WIDTH and LCD_H_RES so the same buffer can paint
     * full-width margin strips as well as the scaled UI stripes. */
    const size_t buf_width = (SCALED_WIDTH > LCD_H_RES) ? SCALED_WIDTH : LCD_H_RES;
    const size_t stripe_bytes = buf_width * STRIPE_HEIGHT * sizeof(uint16_t);
    for(int i = 0; i < NUM_STRIPE_BUFS; i++) {
        stripe_bufs[i] = heap_caps_malloc(stripe_bytes, MALLOC_CAP_DMA);
        if(!stripe_bufs[i]) {
            ESP_LOGE(TAG, "Failed to allocate RGB565 stripe buffer %d (%d bytes)",
                     i, (int)stripe_bytes);
            return;
        }
    }

    /* Clear entire screen to background (unset pixels = FG) */
    display_fill_color(fg_color);
    margin_painted_color = fg_color;

    ESP_LOGI(TAG, "Display initialized (%dx%d, scaled %dx%d, stripe=%d lines, %d bufs, buf=%d bytes each)",
             FB_WIDTH, FB_HEIGHT, SCALED_WIDTH, SCALED_HEIGHT, STRIPE_HEIGHT,
             NUM_STRIPE_BUFS, (int)stripe_bytes);
}

/* Render a single stripe of the scaled framebuffer into the given RGB565 buffer.
 * Optimised: detects duplicate source rows and uses memcpy instead of recomputing.
 * Uses branchless colour selection to avoid branch mispredictions in the inner loop. */
static void render_stripe(
    const uint8_t* data,
    uint16_t* buf,
    size_t stripe_y,
    size_t stripe_h) {

    /* Pre-compute colour table for branchless selection:
     * color_lut[0] = fg_color (pixel not set), color_lut[1] = bg_color (pixel set) */
    const uint16_t color_lut[2] = { fg_color, bg_color };

    uint8_t prev_mono_y = 0xFF; /* impossible value — forces first row to compute */

    for(size_t row = 0; row < stripe_h; row++) {
        const size_t sy = stripe_y + row;
        const uint8_t mono_y = y_scale_lut[sy];
        uint16_t* dst = &buf[row * SCALED_WIDTH];

        /* Row deduplication: if this row maps to the same mono row as the
         * previous one, just memcpy instead of recomputing every pixel. */
        if(mono_y == prev_mono_y && row > 0) {
            memcpy(dst, dst - SCALED_WIDTH, SCALED_WIDTH * sizeof(uint16_t));
            continue;
        }
        prev_mono_y = mono_y;

        const size_t page = mono_y >> 3;
        const uint8_t bit_shift = mono_y & 0x07;
        const uint8_t* page_base = &data[page * FB_WIDTH];

        for(size_t sx = 0; sx < SCALED_WIDTH; sx++) {
            /* Branchless: extract the bit, use it as LUT index */
            const uint8_t pixel_set = (page_base[x_scale_lut[sx]] >> bit_shift) & 1U;
            dst[sx] = color_lut[pixel_set];
        }
    }
}

/* Framebuffer cache for dirty-region tracking */
static uint8_t prev_fb[FB_WIDTH * FB_HEIGHT / 8];
static bool prev_fb_valid = false;
static uint16_t prev_fg_color = 0;
static uint16_t prev_bg_color = 0;

void furi_hal_display_commit(const uint8_t* data, uint32_t size) {
    UNUSED(size);
    if(!panel_handle || !stripe_bufs[0] || !stripe_bufs[1] || !data) return;

    /* If the framebuffer and colours are completely unchanged and margins are
     * up to date, skip the entire commit. Zero SPI overhead for static screens. */
    if(prev_fb_valid &&
       fg_color == prev_fg_color &&
       bg_color == prev_bg_color &&
       margin_painted_color == fg_color) {
        if(memcmp(data, prev_fb, sizeof(prev_fb)) == 0) {
            return;
        }
    }

    /*
     * Double-buffered stripe rendering with DMA pipelining and dirty-stripe tracking:
     *
     * While DMA sends stripe N from buf[cur], CPU renders stripe N+1 into
     * buf[next]. Stripes whose underlying mono pages have not changed since the
     * last frame are skipped entirely, saving both CPU scaling and SPI transfer time.
     *
     * u8g2 tile buffer layout:
     * 8 pages of 128 bytes each (128 columns × 8 pages = 1024 bytes)
     * Byte at [page * 128 + x]: bit n = pixel at (x, page*8 + n)
     * Bit 0 = top pixel in tile, bit 7 = bottom pixel in tile
     */
    furi_hal_spi_bus_lock();

    /* Repaint margin strips ONLY when fg_color has changed. This saves ~4ms
     * per frame in the common case where the colour is constant. */
    if(margin_painted_color != fg_color) {
        if(MARGIN_Y > 0) {
            display_paint_rect(0, 0, LCD_H_RES, MARGIN_Y, fg_color);
            display_paint_rect(
                0, MARGIN_Y + SCALED_HEIGHT,
                LCD_H_RES, LCD_V_RES - MARGIN_Y - SCALED_HEIGHT, fg_color);
        }
        if(MARGIN_X > 0) {
            display_paint_rect(0, MARGIN_Y, MARGIN_X, SCALED_HEIGHT, fg_color);
            display_paint_rect(
                MARGIN_X + SCALED_WIDTH, MARGIN_Y,
                LCD_H_RES - MARGIN_X - SCALED_WIDTH, SCALED_HEIGHT, fg_color);
        }
        margin_painted_color = fg_color;
    }

    const bool force_all_stripes = (!prev_fb_valid ||
                                    fg_color != prev_fg_color ||
                                    bg_color != prev_bg_color);

    /* --- Double-buffered pipelined stripe loop --- */
    int cur_buf = 0;
    bool dma_in_flight = false;

    for(size_t stripe_y = 0; stripe_y < SCALED_HEIGHT; stripe_y += STRIPE_HEIGHT) {
        size_t stripe_h = STRIPE_HEIGHT;
        if(stripe_y + stripe_h > SCALED_HEIGHT) {
            stripe_h = SCALED_HEIGHT - stripe_y;
        }

        /* Check if any mono pages corresponding to this stripe have changed */
        if(!force_all_stripes) {
            const size_t mono_y_start = y_scale_lut[stripe_y];
            const size_t mono_y_end = y_scale_lut[stripe_y + stripe_h - 1];
            const size_t byte_start = (mono_y_start >> 3) * FB_WIDTH;
            const size_t byte_end = ((mono_y_end >> 3) + 1) * FB_WIDTH;
            const size_t check_len = (byte_end <= sizeof(prev_fb)) ?
                                     (byte_end - byte_start) :
                                     (sizeof(prev_fb) - byte_start);

            if(memcmp(&data[byte_start], &prev_fb[byte_start], check_len) == 0) {
                /* Stripe unchanged — skip rendering and DMA */
                continue;
            }
        }

        /* Render this stripe into the current buffer */
        render_stripe(data, stripe_bufs[cur_buf], stripe_y, stripe_h);

        /* Wait for the PREVIOUS DMA to finish (if any) before we submit a new one */
        if(dma_in_flight) {
            furi_hal_display_wait_flush();
        }

        /* Submit this stripe for DMA transfer */
        furi_hal_display_prepare_flush();
        esp_lcd_panel_draw_bitmap(
            panel_handle,
            MARGIN_X, MARGIN_Y + stripe_y,
            MARGIN_X + SCALED_WIDTH, MARGIN_Y + stripe_y + stripe_h,
            stripe_bufs[cur_buf]);
        dma_in_flight = true;

        /* Swap to the other buffer for the next stripe's CPU rendering */
        cur_buf ^= 1;
    }

    /* Wait for the last DMA transfer to complete */
    if(dma_in_flight) {
        furi_hal_display_wait_flush();
    }

    /* Update cache */
    memcpy(prev_fb, data, sizeof(prev_fb));
    prev_fg_color = fg_color;
    prev_bg_color = bg_color;
    prev_fb_valid = true;

    furi_hal_spi_bus_unlock();
}

void furi_hal_display_set_backlight(uint8_t brightness) {
    furi_hal_light_set(LightBacklight, brightness);
}

void furi_hal_display_sleep(void) {
    if(!panel_handle) return;
    furi_hal_spi_bus_lock();
    /* Invalidate frame cache so wake-up redrawn completely */
    prev_fb_valid = false;
    /* SLPIN: stop the panel's internal oscillator/booster to cut idle current */
    esp_lcd_panel_disp_on_off(panel_handle, false);
    furi_hal_spi_bus_unlock();
}

uint16_t furi_hal_display_get_h_res(void) {
    return LCD_H_RES;
}

uint16_t furi_hal_display_get_v_res(void) {
    return LCD_V_RES;
}

esp_lcd_panel_handle_t furi_hal_display_get_panel_handle(void) {
    return panel_handle;
}

void furi_hal_display_set_fg_color(uint16_t color) {
    fg_color = color;
}

uint16_t furi_hal_display_get_fg_color(void) {
    return fg_color;
}

void furi_hal_display_set_bg_color(uint16_t color) {
    bg_color = color;
}

uint16_t furi_hal_display_get_bg_color(void) {
    return bg_color;
}
