/**
 * @file target_input.c
 * Input driver for LilyGo T-LoRa Pager: Rotary Encoder + Boot button
 *
 * Encoder rotation:
 *   CW  (clockwise)        -> InputKeyDown  (scroll down / next)
 *   CCW (counter-clockwise) -> InputKeyUp   (scroll up / prev)
 *
 * Encoder button held + rotation (only after INPUT_ENCODER_MODIFIER_GRACE_MS):
 *   CW  -> InputKeyRight  (tab switch / value adjust)
 *   CCW -> InputKeyLeft   (tab switch / value adjust)
 *
 * Encoder button (GPIO7):
 *   Short press (without rotation) -> InputKeyOk
 *   Long press  -> InputKeyBack
 *
 * Boot button (GPIO0):
 *   Short press -> InputKeyBack
 *
 * This is adapted from the T-Embed CC1101 input driver with different
 * pin assignments. The encoder logic is identical — proven debouncing,
 * state-table quadrature decoding, jostle/guard handling.
 */

#include "target_input.h"

#include <furi_hal_resources.h>
#include <boards/board.h>
#include <driver/gpio.h>
#include <esp_err.h>

#define TAG "InputEncoderPager"

/* Timing constants */
#define INPUT_DEBOUNCE_POLLS        2U
#define INPUT_LONG_PRESS_MS         500U
#define INPUT_REPEAT_MS             200U

/* Pushing the encoder shaft mechanically jostles the A/B contacts, which the
 * quadrature decoder happily reads as a real detent. That would emit Left/Right
 * and veto the Ok on release, so a normal click got swallowed unless it was
 * very short. Rotation therefore only counts as the Left/Right modifier once
 * the press has been held this long. */
#define INPUT_ENCODER_MODIFIER_GRACE_MS 80U

/* The shaft is both the knob and the Ok button, so turning it tends to nudge
 * the switch. A press that lands inside an ongoing turn is such a nudge, not a
 * deliberate click, and must not emit Ok Short. */
#define INPUT_ROTATION_GUARD_MS 150U

/* Leftover quarter-steps from the previous turn stay in the accumulator and let
 * a single contact wobble complete a detent. Drop them once the encoder has
 * been still for a while. */
#define INPUT_ENCODER_ACCUM_DECAY_MS 200U

/* Encoder state — proper quadrature decoder with accumulator. */
typedef struct {
    uint8_t ab_state;    // last AB reading (2 bits)
    int8_t  accum;       // accumulated steps (positive = CW)
    uint32_t last_step_at;   // tick of the last quarter-step (drives accum decay)
    uint32_t last_detent_at; // tick of the last real detent (drives the rotation guard)
} EncoderState;

/* Quadrature state transition table: indexed by (old_AB << 2 | new_AB)
 *  0 = invalid/no movement, +1 = CW quarter-step, -1 = CCW quarter-step */
static const int8_t enc_table[16] = {
    /*          new: 00  01  10  11  */
    /* old 00 */  0, -1, +1,  0,
    /* old 01 */ +1,  0,  0, -1,
    /* old 10 */ -1,  0,  0, +1,
    /* old 11 */  0, +1, -1,  0,
};

/* Steps per detent — most mechanical encoders produce 4 state changes per
 * click (full-step). Some produce 2 (half-step). The T-LoRa Pager encoder
 * uses 2 state changes per detent (same as T-Embed). */
#define ENCODER_STEPS_PER_DETENT 2

/* Button state (reusable for any GPIO button) */
typedef struct {
    gpio_num_t gpio;
    bool inverted;          /* true = active-low */
    bool raw_pressed;
    bool debounced_pressed;
    uint8_t debounce_polls;
    uint32_t press_started_at;
    bool long_press_sent;
    uint32_t last_repeat_at;
    bool had_encoder_rotation; /* encoder rotated while this button was held */
    bool rotation_guarded;     /* press landed inside a turn — emit nothing on release */
} ButtonState;

static EncoderState encoder;
static ButtonState encoder_btn;
static ButtonState boot_btn;

/* --- helpers --- */

static void input_publish(FuriPubSub* pubsub, InputKey key, InputType type, uint32_t sequence) {
    InputEvent event = {
        .sequence_source = INPUT_SEQUENCE_SOURCE_HARDWARE,
        .sequence_counter = sequence,
        .key = key,
        .type = type,
    };
    furi_pubsub_publish(pubsub, &event);
}

static void input_emit_short(FuriPubSub* pubsub, InputKey key, uint32_t sequence) {
    input_publish(pubsub, key, InputTypePress, sequence);
    input_publish(pubsub, key, InputTypeShort, sequence);
    input_publish(pubsub, key, InputTypeRelease, sequence);
}

static bool button_is_pressed(ButtonState* btn) {
    int level = gpio_get_level(btn->gpio);
    return btn->inverted ? (level == 0) : (level != 0);
}

static void button_init_gpio(gpio_num_t pin, bool pull_up) {
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pull_up ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_up ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if(err != ESP_OK) {
        FURI_LOG_E(TAG, "GPIO %d config failed: %s", pin, esp_err_to_name(err));
    }
}

static void button_init_state(ButtonState* btn, gpio_num_t gpio, bool inverted) {
    btn->gpio = gpio;
    btn->inverted = inverted;
    btn->raw_pressed = button_is_pressed(btn);
    btn->debounced_pressed = btn->raw_pressed;
    btn->debounce_polls = INPUT_DEBOUNCE_POLLS;
    btn->press_started_at = 0;
    btn->long_press_sent = false;
    btn->last_repeat_at = 0;
    btn->had_encoder_rotation = false;
    btn->rotation_guarded = false;
}

static void button_poll(
    ButtonState* btn,
    FuriPubSub* pubsub,
    InputKey short_key,
    InputKey long_key,
    uint32_t now,
    uint32_t long_press_ticks,
    uint32_t repeat_ticks,
    uint32_t* sequence_counter) {
    bool raw = button_is_pressed(btn);

    /* Debounce */
    if(raw == btn->raw_pressed) {
        if(btn->debounce_polls < INPUT_DEBOUNCE_POLLS) {
            btn->debounce_polls++;
        }
    } else {
        btn->raw_pressed = raw;
        btn->debounce_polls = 1;
    }

    if(btn->debounce_polls < INPUT_DEBOUNCE_POLLS) return;
    if(btn->debounced_pressed == btn->raw_pressed) {
        /* Held-down handling: long press + repeat */
        if(btn->debounced_pressed) {
            uint32_t held = now - btn->press_started_at;
            if(!btn->long_press_sent && held >= long_press_ticks) {
                btn->long_press_sent = true;
                btn->last_repeat_at = now;
                input_publish(pubsub, long_key, InputTypePress, ++(*sequence_counter));
                input_publish(pubsub, long_key, InputTypeLong, *sequence_counter);
            } else if(btn->long_press_sent && (now - btn->last_repeat_at) >= repeat_ticks) {
                btn->last_repeat_at = now;
                input_publish(pubsub, long_key, InputTypeRepeat, *sequence_counter);
            }
        }
        return;
    }

    /* State changed */
    btn->debounced_pressed = btn->raw_pressed;

    if(btn->debounced_pressed) {
        /* Press */
        btn->press_started_at = now;
        btn->long_press_sent = false;
        btn->had_encoder_rotation = false;
    } else {
        /* Release */
        if(!btn->long_press_sent) {
            if(!btn->had_encoder_rotation) {
                input_emit_short(pubsub, short_key, ++(*sequence_counter));
            }
        } else {
            input_publish(pubsub, long_key, InputTypeRelease, *sequence_counter);
        }
        btn->had_encoder_rotation = false;
    }
}

/* --- Encoder button: Ok on short click, modifier for Left/Right when held --- */

static void encoder_button_poll(
    ButtonState* btn,
    FuriPubSub* pubsub,
    uint32_t now,
    uint32_t long_press_ticks,
    uint32_t* sequence_counter) {
    bool raw = button_is_pressed(btn);

    /* Debounce */
    if(raw == btn->raw_pressed) {
        if(btn->debounce_polls < INPUT_DEBOUNCE_POLLS) {
            btn->debounce_polls++;
        }
    } else {
        btn->raw_pressed = raw;
        btn->debounce_polls = 1;
    }

    if(btn->debounce_polls < INPUT_DEBOUNCE_POLLS) return;

    if(btn->debounced_pressed == btn->raw_pressed) {
        if(btn->debounced_pressed && !btn->long_press_sent &&
           (now - btn->press_started_at) >= long_press_ticks) {
            btn->long_press_sent = true;
            input_publish(pubsub, InputKeyOk, InputTypePress, ++(*sequence_counter));
            input_publish(pubsub, InputKeyOk, InputTypeLong, *sequence_counter);
        }
        return;
    }

    /* State changed */
    btn->debounced_pressed = btn->raw_pressed;

    if(btn->debounced_pressed) {
        btn->rotation_guarded =
            (now - encoder.last_detent_at) < furi_ms_to_ticks(INPUT_ROTATION_GUARD_MS);
        btn->press_started_at = now;
        btn->had_encoder_rotation = false;
        btn->long_press_sent = false;
    } else {
        if(btn->long_press_sent || btn->had_encoder_rotation) {
            input_publish(pubsub, InputKeyOk, InputTypeRelease, ++(*sequence_counter));
        } else if(!btn->rotation_guarded) {
            input_emit_short(pubsub, InputKeyOk, ++(*sequence_counter));
        }
        btn->had_encoder_rotation = false;
        btn->rotation_guarded = false;
    }
}

/* --- Encoder reading --- */

static void encoder_init(void) {
    button_init_gpio((gpio_num_t)BOARD_PIN_ENCODER_A, true);
    button_init_gpio((gpio_num_t)BOARD_PIN_ENCODER_B, true);

    bool a = gpio_get_level((gpio_num_t)BOARD_PIN_ENCODER_A);
    bool b = gpio_get_level((gpio_num_t)BOARD_PIN_ENCODER_B);
    encoder.ab_state = (a << 1) | b;
    encoder.accum = 0;
    encoder.last_step_at = 0;
    encoder.last_detent_at = 0;
}

typedef enum {
    EncoderRotationScroll,
    EncoderRotationModifier,
    EncoderRotationJostle,
} EncoderRotationMode;

static EncoderRotationMode encoder_rotation_mode(uint32_t now) {
    if(!encoder_btn.raw_pressed && !encoder_btn.debounced_pressed) return EncoderRotationScroll;
    if(encoder_btn.long_press_sent) return EncoderRotationScroll;

    if(encoder_btn.raw_pressed != encoder_btn.debounced_pressed) return EncoderRotationJostle;

    if((now - encoder_btn.press_started_at) < furi_ms_to_ticks(INPUT_ENCODER_MODIFIER_GRACE_MS)) {
        return EncoderRotationJostle;
    }

    return EncoderRotationModifier;
}

static void
    encoder_emit_rotation(FuriPubSub* pubsub, bool cw, uint32_t now, uint32_t* sequence_counter) {
    EncoderRotationMode mode = encoder_rotation_mode(now);

    if(mode != EncoderRotationJostle) {
        encoder.last_detent_at = now;
    }

    switch(mode) {
    case EncoderRotationModifier:
        input_publish(pubsub, InputKeyOk, InputTypePress, ++(*sequence_counter));
        input_emit_short(pubsub, cw ? InputKeyRight : InputKeyLeft, ++(*sequence_counter));
        encoder_btn.had_encoder_rotation = true;
        break;
    case EncoderRotationScroll:
        input_emit_short(pubsub, cw ? InputKeyDown : InputKeyUp, ++(*sequence_counter));
        break;
    case EncoderRotationJostle:
        break;
    }
}

static void encoder_poll(FuriPubSub* pubsub, uint32_t now, uint32_t* sequence_counter) {
    if(encoder.accum != 0 &&
       (now - encoder.last_step_at) >= furi_ms_to_ticks(INPUT_ENCODER_ACCUM_DECAY_MS)) {
        encoder.accum = 0;
    }

    bool a = gpio_get_level((gpio_num_t)BOARD_PIN_ENCODER_A);
    bool b = gpio_get_level((gpio_num_t)BOARD_PIN_ENCODER_B);
    uint8_t new_ab = (a << 1) | b;

    if(new_ab == encoder.ab_state) return;

    int8_t delta = enc_table[(encoder.ab_state << 2) | new_ab];
    encoder.ab_state = new_ab;

    if(delta == 0) return;

    encoder.accum += delta;
    encoder.last_step_at = now;

    if(encoder.accum >= ENCODER_STEPS_PER_DETENT) {
        encoder.accum = 0;
        encoder_emit_rotation(pubsub, true, now, sequence_counter);
    } else if(encoder.accum <= -ENCODER_STEPS_PER_DETENT) {
        encoder.accum = 0;
        encoder_emit_rotation(pubsub, false, now, sequence_counter);
    }
}

/* --- target_input interface --- */

void target_input_init(void) {
    /* Encoder A/B */
    encoder_init();

    /* Encoder button (GPIO7, active low) */
    button_init_gpio((gpio_num_t)BOARD_PIN_ENCODER_BTN, true);
    button_init_state(&encoder_btn, (gpio_num_t)BOARD_PIN_ENCODER_BTN, true);

    /* Boot button (GPIO0, active low) — serves as Back button */
    button_init_gpio((gpio_num_t)BOARD_PIN_BUTTON_BOOT, true);
    button_init_state(&boot_btn, (gpio_num_t)BOARD_PIN_BUTTON_BOOT, true);

    FURI_LOG_I(TAG, "T-LoRa Pager input initialized: Encoder A=%d B=%d BTN=%d, Boot=%d",
               BOARD_PIN_ENCODER_A, BOARD_PIN_ENCODER_B,
               BOARD_PIN_ENCODER_BTN, BOARD_PIN_BUTTON_BOOT);
}

void target_input_poll(FuriPubSub* pubsub, uint32_t* sequence_counter) {
    uint32_t now = furi_get_tick();
    uint32_t long_press_ticks = furi_ms_to_ticks(INPUT_LONG_PRESS_MS);
    uint32_t repeat_ticks = furi_ms_to_ticks(INPUT_REPEAT_MS);

    /* Rotary encoder (must poll before buttons to detect held+rotate) */
    encoder_poll(pubsub, now, sequence_counter);

    /* Encoder button: short=Ok (if no rotation), long-hold emits Ok Long+Release */
    encoder_button_poll(
        &encoder_btn, pubsub,
        now, long_press_ticks, sequence_counter);

    /* Boot button: short=Back (dedicated back button) */
    button_poll(
        &boot_btn, pubsub,
        InputKeyBack, InputKeyBack,
        now, long_press_ticks, repeat_ticks,
        sequence_counter);
}
