#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define RECEIVER_LOG_SIZE 128

typedef struct {
    uint32_t seq;
    uint32_t at_uptime_s;   // sender's clock when known, else the receiver's
    float    rssi;
    float    snr;
    float    signal_rssi;
    uint16_t gap;           // sequence numbers missing before this one
    uint8_t  len;
    uint8_t  type;
} receiver_log_t;

typedef struct {
    // Counters
    uint32_t rx_count;      // packets accepted
    uint32_t lost;          // inferred from sequence gaps
    uint32_t dup;
    uint32_t out_of_order;
    uint16_t crc_err;       // from the chip's own GetStats
    uint16_t hdr_err;
    float    pdr;           // delivery ratio, percent

    // Last packet
    bool  have_packet;
    float last_rssi;
    float last_snr;
    float last_signal_rssi;
    uint32_t last_seq;
    uint32_t since_last_s;

    // Derived link quality
    float noise_floor;      // instantaneous RSSI sampled while idle in RX
    float snr_margin;       // how far the last SNR sits above the demodulator floor
    float link_budget;      // configured TX power minus received RSSI
    float fade_margin;      // received RSSI minus estimated sensitivity

    // Remote battery, from the sender's BATT packets. Reported as measured at the
    // sense pin - no divider is applied anywhere on either board.
    bool     have_batt;
    uint16_t batt_pin_mv;
    uint32_t batt_uptime_s;

    uint32_t log_seq;
} receiver_status_t;

esp_err_t receiver_start(void);
void      receiver_get_status(receiver_status_t *out);
int       receiver_copy_log(receiver_log_t *out, int max, uint32_t since, uint32_t *next);
void      receiver_reset_counters(void);
