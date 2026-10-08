#pragma once

// The Repeater PCB's WS2812 status LED (U8, DIN on IO4, VDD straight off BAT+).
//
// One task owns the LED and renders, highest priority first:
//
//   fatal fault      red, 5 Hz                       the radio never came up
//   max drain        solid white, full brightness    the load is running
//   max drain armed  white, 2 Hz                     the boot delay before it starts
//   event flashes    short, one per packet / event   see led_event()
//   background       a blink every few seconds       link down, provisional, ...
//
// White is the maximum-power colour on purpose: the WS2812 sinks a constant
// current per die, so all three dies at 255 is the most it can draw, and during
// max drain that is the point. Everything else runs at the configurable status
// level (a tenth of full by default) and is lit for tens of milliseconds at a
// time, so the indicator does not bend a normal battery run.
//
// Because VDD is BAT+, not 3V3, the LED stays dark with no cell fitted however
// correct the firmware is, and it keeps its last colour through an ESP reset.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    LED_EV_TX,         // sender: a test packet went out              blue tick
    LED_EV_NO_ACK,     // sender: its acknowledgement never came      long red
    LED_EV_RADIO_ERR,  // the radio refused a transmit outright       red double
    LED_EV_LOST,       // receiver: packets missing before this one   long red
    LED_EV_CRC,        // receiver: a frame failed CRC/header check   magenta tick
    LED_EV_COMMAND,    // a remote command was carried out / answered  violet double
    LED_EV_PROFILE,    // a new radio profile went live               cyan triple
    LED_EV_REVERT,     // a provisional profile was rolled back       red-cyan
    LED_EV_COMMIT,     // a provisional profile was saved for good    green-cyan
} led_event_t;

// Set the LED up and black it out. Call early - before the radio, so a radio that
// never comes up can still be shown. Not fatal: without it nothing lights.
esp_err_t led_init(void);

// Queue a flash. Never blocks, so it is safe on the radio path; a full queue
// drops the flash rather than delay an acknowledgement.
void led_event(led_event_t ev);

// A packet that came through, flashed in a colour for how much room the link had:
// its SNR above the demodulator floor for this spreading factor.
//   >= 10 dB green   5..10 yellow-green   0..5 amber   below 0 orange-red
void led_link_quality(float snr_margin_db);

// The radio failed to start; the LED blinks red for good.
void led_set_fatal(void);

// The radio task is running, so its state can be read for the background patterns.
// Until then only the fault and max-drain patterns and flashes show.
void led_status_start(void);

// Status brightness 0..255, saved. 0 turns the indicator off; the fault, max-drain
// and max-drain-armed patterns still show, since they are warnings, not status.
uint8_t   led_level(void);
esp_err_t led_set_level(int level);

// Red, green, blue, white at full brightness, 400 ms each, to check the hardware.
void led_self_test(void);

// What the LED is showing now ("max drain", "link down", ...) and its colour this
// instant as 0xRRGGBB, for the dashboards.
const char *led_pattern(void);
uint32_t    led_rgb(void);

// The last LED_TRACE_LEN colour changes, oldest first, so the flash timing can be
// checked from /led?trace=1 without anyone watching the board.
#define LED_TRACE_LEN 48
typedef struct {
    int64_t     at_ms;    // esp_timer milliseconds
    uint32_t    rgb;      // 0xRRGGBB
    const char *pattern;
} led_trace_t;
int led_trace(led_trace_t *out, int max);
