#include "led.h"

#include <string.h>

#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "link.h"
#include "nvs.h"
#include "sdkconfig.h"

#if CONFIG_ROLE_SENDER
#include "battery.h"
#include "powerload.h"
#include "sender.h"
#else
#include "receiver.h"
#endif

static const char *TAG = "led";

// With the LED switched off in menuconfig these are not defined; every call below
// still works and does nothing, since no task or queue is ever created.
#ifndef CONFIG_LED_LEVEL
#define CONFIG_LED_LEVEL 0
#endif
#ifndef CONFIG_LED_LOW_BATT_MV
#define CONFIG_LED_LOW_BATT_MV 0
#endif
#ifndef CONFIG_LED_GPIO
#define CONFIG_LED_GPIO -1
#endif
#if CONFIG_LED_ENABLE
#define LED_ENABLED 1
#else
#define LED_ENABLED 0
#endif

#define NVS_NS        "led"
#define NVS_KEY_LEVEL "level"

// 10 MHz: 0.1 us per RMT tick, fine enough for the WS2812's 0.3/0.9 us bits.
#define RMT_RES_HZ 10000000
#define TICK_MS    10    // render period
#define BG_POLL_MS 200   // how often the background state is re-read
#define REFRESH_MS 1000  // an unchanged colour is rewritten this often, in case of a glitch
#define QUEUE_LEN  16
// A gap after every flash, so two in a row read as two.
#define FLASH_GAP_MS 60

// Warnings (fault, max drain armed) show even with the status level at 0, and never
// dimmer than this, so turning the indicator off cannot hide them.
#define WARN_LEVEL 48

// Sender: this many exchanges in a row without an answer is "link down".
#define LINK_DOWN_MISSES 3

#if CONFIG_ROLE_SENDER
// The battery is re-read this often for the low-battery pattern. One read is 32
// ADC samples, and the cell does not change faster than this.
#define BATT_POLL_MS 10000
#endif

typedef struct {
    uint8_t r, g, b;
} rgb_t;

static const rgb_t BLACK  = { 0, 0, 0 };
static const rgb_t WHITE  = { 255, 255, 255 };
static const rgb_t RED    = { 255, 0, 0 };
static const rgb_t GREEN  = { 0, 255, 0 };
static const rgb_t BLUE   = { 0, 0, 255 };
static const rgb_t CYAN   = { 0, 255, 255 };
static const rgb_t VIOLET = { 160, 0, 255 };
static const rgb_t PINK   = { 255, 0, 160 };
static const rgb_t AMBER  = { 255, 140, 0 };
static const rgb_t ORANGE = { 255, 50, 0 };

// One lit-then-dark step of a flash, already scaled to its brightness.
typedef struct {
    rgb_t    c;
    uint16_t on_ms;
    uint16_t off_ms;
} step_t;

// A background pattern: `blinks` flashes of on_ms each, an on_ms apart, at the start
// of every period_ms. period_ms 0 is solid.
typedef struct {
    const char *name;
    rgb_t       c;
    uint16_t    period_ms;
    uint8_t     blinks;
    uint16_t    on_ms;
} pattern_t;

static const pattern_t P_FATAL       = { "radio fault",         { 255, 0, 0 },     200,  1, 100 };
static const pattern_t P_LINK_DOWN   = { "link down",           { 255, 0, 0 },     2000, 1, 100 };
static const pattern_t P_PROVISIONAL = { "provisional profile", { 0, 255, 255 },   2000, 2, 60 };
static const pattern_t P_STOPPED     = { "stopped",             { 255, 140, 0 },   3000, 1, 80 };
static const pattern_t P_IDLE        = { "ok",                  { 0, 255, 0 },     5000, 1, 30 };
static const pattern_t P_OFF         = { "off",                 { 0, 0, 0 },       0,    0, 0 };
#if CONFIG_ROLE_SENDER
static const pattern_t P_DRAIN       = { "max drain",           { 255, 255, 255 }, 0,    0, 0 };
static const pattern_t P_ARMING      = { "max drain arming",    { 255, 255, 255 }, 500,  1, 250 };
static const pattern_t P_LOW_BATT    = { "low battery",         { 255, 50, 0 },    4000, 3, 60 };
#else
static const pattern_t P_LISTENING   = { "listening",           { 0, 0, 255 },     3000, 1, 60 };
static const pattern_t P_CMD_WAIT    = { "command waiting",     { 160, 0, 255 },   1000, 1, 60 };
#endif

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static QueueHandle_t        s_queue;
static volatile uint8_t     s_level = CONFIG_LED_LEVEL;
static volatile bool        s_fatal;
static volatile bool        s_status_ready;  // the role's state is safe to read
static const char *volatile s_pattern = "off";
static volatile uint32_t    s_rgb;

static led_trace_t  s_trace[LED_TRACE_LEN];
static uint32_t     s_trace_n;  // changes ever recorded
static portMUX_TYPE s_trace_mux = portMUX_INITIALIZER_UNLOCKED;

static rgb_t scale(rgb_t c, uint8_t level)
{
    return (rgb_t){
        .r = (uint8_t)((c.r * level + 127) / 255),
        .g = (uint8_t)((c.g * level + 127) / 255),
        .b = (uint8_t)((c.b * level + 127) / 255),
    };
}

static uint8_t warn_level(void)
{
    return s_level > WARN_LEVEL ? s_level : WARN_LEVEL;
}

static void write_rgb(rgb_t c)
{
    // The WS2812 takes green first, MSB first. All 24 bits fit in one RMT memory
    // block, so the frame plays out of hardware with no refill interrupt for WiFi's
    // ISRs to delay - the timing cannot be stretched mid-frame.
    const uint8_t grb[3] = { c.g, c.r, c.b };
    const rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &tx) == ESP_OK) {
        // The line then idles low far longer than the 280 us reset the newer
        // WS2812B needs: frames are at least TICK_MS apart.
        rmt_tx_wait_all_done(s_chan, 20);
    }
    s_rgb = ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | c.b;
}

static void push(rgb_t c, uint16_t on_ms, uint16_t off_ms)
{
    if (s_queue == NULL) {
        return;
    }
    const step_t s = { .c = c, .on_ms = on_ms, .off_ms = off_ms };
    (void)xQueueSend(s_queue, &s, 0);  // full: drop it, never block the radio
}

// `count` flashes of one colour at the status level.
static void flash(rgb_t c, uint16_t on_ms, uint8_t count)
{
    const uint8_t level = s_level;
    if (level == 0) {
        return;
    }
    const rgb_t sc = scale(c, level);
    for (uint8_t i = 0; i < count; i++) {
        push(sc, on_ms, (i + 1 < count) ? on_ms : FLASH_GAP_MS);
    }
}

void led_event(led_event_t ev)
{
    switch (ev) {
        case LED_EV_TX:        flash(BLUE, 20, 1);    break;
        case LED_EV_NO_ACK:    flash(RED, 200, 1);    break;
        case LED_EV_RADIO_ERR: flash(RED, 60, 2);     break;
        case LED_EV_LOST:      flash(RED, 200, 1);    break;
        case LED_EV_CRC:       flash(PINK, 30, 1);    break;
        case LED_EV_COMMAND:   flash(VIOLET, 60, 2);  break;
        case LED_EV_PROFILE:   flash(CYAN, 60, 3);    break;
        case LED_EV_REVERT:    flash(RED, 150, 1);    flash(CYAN, 150, 1); break;
        case LED_EV_COMMIT:    flash(GREEN, 150, 1);  flash(CYAN, 150, 1); break;
    }
}

void led_link_quality(float m)
{
    const rgb_t c = (m >= 10.0f) ? GREEN
                  : (m >= 5.0f)  ? (rgb_t){ 128, 255, 0 }
                  : (m >= 0.0f)  ? AMBER
                                 : ORANGE;
    flash(c, 60, 1);
}

void led_set_fatal(void)
{
    s_fatal = true;
}

void led_status_start(void)
{
    s_status_ready = true;
}

uint8_t led_level(void)
{
    return s_level;
}

esp_err_t led_set_level(int level)
{
    if (level < 0 || level > 255) {
        return ESP_ERR_INVALID_ARG;
    }
    s_level = (uint8_t)level;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u8(h, NVS_KEY_LEVEL, (uint8_t)level);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    ESP_LOGI(TAG, "Status level %d/255", level);
    return err;
}

void led_self_test(void)
{
    // Full brightness and unscaled, so a dim or dead die is obvious.
    push(RED, 400, 100);
    push(GREEN, 400, 100);
    push(BLUE, 400, 100);
    push(WHITE, 400, 100);
}

const char *led_pattern(void)
{
    return s_pattern;
}

uint32_t led_rgb(void)
{
    return s_rgb;
}

static void trace_add(int64_t at_ms, rgb_t c, const char *pattern)
{
    taskENTER_CRITICAL(&s_trace_mux);
    s_trace[s_trace_n % LED_TRACE_LEN] = (led_trace_t){
        .at_ms   = at_ms,
        .rgb     = ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | c.b,
        .pattern = pattern,
    };
    s_trace_n++;
    taskEXIT_CRITICAL(&s_trace_mux);
}

int led_trace(led_trace_t *out, int max)
{
    taskENTER_CRITICAL(&s_trace_mux);
    const uint32_t n     = s_trace_n;
    const uint32_t first = n > LED_TRACE_LEN ? n - LED_TRACE_LEN : 0;
    int k = 0;
    for (uint32_t i = first; i < n && k < max; i++) {
        out[k++] = s_trace[i % LED_TRACE_LEN];
    }
    taskEXIT_CRITICAL(&s_trace_mux);
    return k;
}

#if CONFIG_ROLE_SENDER
static bool battery_low(int64_t now_ms)
{
    static int64_t next_ms;
    static bool    low;
    if (now_ms >= next_ms) {
        next_ms = now_ms + BATT_POLL_MS;
        const int pin = battery_read_pin_mv();
        // A pin near 0 V means no cell (or no divider), not a flat one.
        low = pin >= 100 && battery_mv_from_pin(pin) < CONFIG_LED_LOW_BATT_MV;
    }
    return low;
}
#endif

// The lowest layer: what the board has to say when nothing is happening, in order of
// how much it matters.
static const pattern_t *background(int64_t now_ms)
{
    // Before the radio task exists its mutex does not either, and taking a NULL
    // one asserts.
    if (s_level == 0 || !s_status_ready) {
        return &P_OFF;
    }
#if CONFIG_ROLE_SENDER
    sender_status_t st;
    sender_get_status(&st);
    if (st.consec_miss >= LINK_DOWN_MISSES) {
        return &P_LINK_DOWN;
    }
    if (link_profile_is_provisional()) {
        return &P_PROVISIONAL;
    }
    if (battery_low(now_ms)) {
        return &P_LOW_BATT;
    }
    if (!st.running) {
        return &P_STOPPED;
    }
    return &P_IDLE;
#else
    (void)now_ms;
    // Static: the status struct is large for this task's stack, and only this task
    // reads it.
    static receiver_status_t st;
    receiver_get_status(&st);
    if (!st.contact_seen) {
        return &P_LISTENING;
    }
    if (st.link_lost) {
        return &P_LINK_DOWN;
    }
    if (st.cmd_queued > 0) {
        return &P_CMD_WAIT;
    }
    if (link_profile_is_provisional()) {
        return &P_PROVISIONAL;
    }
    if (st.sender_stopped) {
        return &P_STOPPED;
    }
    return &P_IDLE;
#endif
}

// Blinks sit at the start of each period, so a warning shows the moment it applies.
static bool pattern_lit(const pattern_t *p, int64_t elapsed_ms)
{
    if (p->period_ms == 0) {
        return p->c.r | p->c.g | p->c.b;
    }
    const int64_t phase = elapsed_ms % p->period_ms;
    const int64_t span  = 2 * (int64_t)p->on_ms;  // one blink and the gap after it
    return phase < span * p->blinks && (phase % span) < p->on_ms;
}

// The background's blinks sit at the end of each period instead, counted from the
// last flash: the board has nothing new to say until a full period has passed with
// nothing happening. So a heartbeat never follows straight on from a packet flash
// (which already says the board is alive), and a link carrying a packet more often
// than the heartbeat period shows only its packets.
static bool background_lit(const pattern_t *p, int64_t elapsed_ms)
{
    if (p->period_ms == 0) {
        return p->c.r | p->c.g | p->c.b;
    }
    const int64_t span   = 2 * (int64_t)p->on_ms;
    const int64_t offset = p->period_ms - span * p->blinks;
    const int64_t phase  = elapsed_ms % p->period_ms;
    return phase >= offset && ((phase - offset) % span) < p->on_ms;
}

static void led_task(void *arg)
{
    const pattern_t *bg       = &P_OFF;
    const pattern_t *shown    = NULL;  // pattern currently on, for phase alignment
    int64_t          shown_at = 0;
    int64_t          next_bg  = 0;
    int64_t          quiet_at = 0;     // when the last flash ended

    step_t  step;
    bool    in_step  = false;
    int64_t step_end = 0, step_off = 0;  // lit until step_off, dark until step_end

    rgb_t   last      = BLACK;
    int64_t last_tx   = 0;

    for (;;) {
        const int64_t now = esp_timer_get_time() / 1000;
        rgb_t       c;
        const char *name;

        const pattern_t *top  = NULL;
        bool             full = false;  // max drain: 255 whatever the status level
#if CONFIG_ROLE_SENDER
        if (powerload_is_active()) {
            top  = &P_DRAIN;
            full = true;
        } else if (powerload_is_on()) {
            top = &P_ARMING;  // on, but still inside the boot delay
        }
#endif
        if (s_fatal) {
            top  = &P_FATAL;
            full = false;
        }

        if (top != NULL) {
            // Warnings own the LED outright. Flashes queued meanwhile are stale
            // by the time it frees up, so they are dropped rather than replayed.
            xQueueReset(s_queue);
            in_step = false;
            if (shown != top) {
                shown    = top;
                shown_at = now;
            }
            const uint8_t lv = full ? 255 : warn_level();
            c    = pattern_lit(top, now - shown_at) ? scale(top->c, lv) : BLACK;
            name = top->name;
        } else {
            if (!in_step && xQueueReceive(s_queue, &step, 0) == pdTRUE) {
                in_step  = true;
                step_off = now + step.on_ms;
                step_end = step_off + step.off_ms;
            }
            if (in_step && now >= step_end) {
                in_step = false;
                // Straight on to the next flash if one is waiting.
                if (xQueueReceive(s_queue, &step, 0) == pdTRUE) {
                    in_step  = true;
                    step_off = now + step.on_ms;
                    step_end = step_off + step.off_ms;
                }
            }

            if (in_step) {
                c        = (now < step_off) ? step.c : BLACK;
                name     = "event";
                quiet_at = step_end;
            } else {
                if (now >= next_bg) {
                    bg      = background(now);
                    next_bg = now + BG_POLL_MS;
                }
                if (shown != bg) {
                    shown    = bg;
                    shown_at = now;
                }
                const int64_t since = shown_at > quiet_at ? shown_at : quiet_at;
                c    = background_lit(bg, now - since) ? scale(bg->c, s_level) : BLACK;
                name = bg->name;
            }
        }

        s_pattern = name;
        const bool changed = memcmp(&c, &last, sizeof(c)) != 0;
        if (changed) {
            trace_add(now, c, name);
        }
        if (changed || now - last_tx >= REFRESH_MS) {
            write_rgb(c);
            last    = c;
            last_tx = now;
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t led_init(void)
{
    if (!LED_ENABLED) {
        return ESP_OK;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v;
        if (nvs_get_u8(h, NVS_KEY_LEVEL, &v) == ESP_OK) {
            s_level = v;
        }
        nvs_close(h);
    }

    const rmt_tx_channel_config_t ch = {
        .gpio_num          = CONFIG_LED_GPIO,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 48,  // one C3 block; a frame is 24
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&ch, &s_chan);
    if (err == ESP_OK) {
        // T0H 0.3 / T0L 0.9 us, T1H 0.9 / T1L 0.3 us: inside both the WS2812 and
        // WS2812B windows.
        const rmt_bytes_encoder_config_t enc = {
            .bit0  = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
            .bit1  = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
            .flags = { .msb_first = 1 },
        };
        err = rmt_new_bytes_encoder(&enc, &s_enc);
    }
    if (err == ESP_OK) {
        err = rmt_enable(s_chan);
    }
    s_queue = xQueueCreate(QUEUE_LEN, sizeof(step_t));
    if (err != ESP_OK || s_queue == NULL) {
        ESP_LOGE(TAG, "Status LED unavailable (%s)", esp_err_to_name(err));
        return err != ESP_OK ? err : ESP_ERR_NO_MEM;
    }

    // Black first: the LED runs off BAT+, so it keeps whatever it last showed
    // through a reset of the ESP - including max drain's full white.
    write_rgb(BLACK);
    if (xTaskCreate(led_task, "led", 3072, NULL, 3, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Status LED on GPIO%d, level %u/255", CONFIG_LED_GPIO, s_level);
    return ESP_OK;
}
