#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sx126x.h"

#define RECEIVER_LOG_SIZE 128

// One accepted packet, with every link figure as it stood when it arrived: the
// same ones the console line prints, so the dashboard table and the serial log
// always agree. Kept for battery packets as much as for pings.
typedef struct {
    uint32_t seq;
    uint32_t at_uptime_s;   // sender's discharge clock when known, else 0
    float    rssi;          // packet RSSI, dBm
    float    signal_rssi;   // after despreading, dBm
    float    snr;
    float    snr_margin;    // SNR above the demodulator floor for this SF
    float    fade_margin;   // signal RSSI above the estimated sensitivity
    float    path_loss;     // TX power minus signal RSSI
    float    freq_err_hz;   // carrier offset between the two radios
    float    noise_floor;   // latest idle-channel RSSI
    float    pdr;           // delivery ratio so far, percent
    uint32_t lost, dup, out_of_order;  // running totals
    uint16_t crc_err, hdr_err;         // running totals
    uint16_t gap;           // sequence numbers missing before this one
    uint16_t batt_mv;       // battery packets: the sender's battery; 0 otherwise
    uint8_t  len;
    uint8_t  type;
    uint8_t  charge;        // sender's solar charger, charge_state_t
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
    float link_budget;      // configured TX power minus signal RSSI: the path loss
    float fade_margin;      // signal RSSI minus estimated sensitivity

    // Remote battery, from the sender's BATT packets: the sense pin as measured, and
    // the battery voltage from the divider ratio the sender sends alongside it.
    bool     have_batt;
    uint16_t batt_pin_mv;
    uint32_t batt_uptime_s;
    uint16_t batt_ratio_x1000;  // 0 until a battery packet has carried one
    uint32_t batt_mv;           // 0 while the ratio is unknown
    uint8_t  peer_charge;       // sender's solar charger (charge_state_t), from its packets

    // Commands for the sender, and what it last reported about itself.
    int      cmd_queued;        // waiting to be confirmed by the sender
    char     cmd_waiting[56];   // the one going out now, described; "" if none
    char     cmd_state[96];     // the last outcome
    bool     sender_known;      // the sender has answered a command since boot
    uint32_t sender_age_s;      // since that answer
    bool     sender_running, sender_load, sender_wifi;
    uint8_t  sender_mode;       // link_mode_t
    uint32_t sender_interval_ms;
    uint8_t  sender_payload;
    uint16_t sender_ratio_x1000;
    bool     contact_seen;      // heard any frame from the sender since boot
    uint32_t contact_age_s;
    bool     sender_stopped;    // as of the last frame heard from it
    bool     link_lost;         // contact_seen, and silent for longer than its cadence explains

    uint32_t log_seq;
} receiver_status_t;

esp_err_t receiver_start(void);

// Queue a command for the sender (link_cmd_op_t). It rides on this board's ACKs
// until the sender confirms it; cfg is for LINK_CMD_PUSH_CFG, NULL otherwise.
// ESP_ERR_NO_MEM when the queue is full.
esp_err_t receiver_queue_command(uint8_t op, uint32_t arg, const sx126x_cfg_t *cfg);
// Drop everything still waiting. One already delivered may have been acted on.
void      receiver_cancel_commands(void);
void      receiver_get_status(receiver_status_t *out);
int       receiver_copy_log(receiver_log_t *out, int max, uint32_t since, uint32_t *next);
void      receiver_reset_counters(void);
