#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sx126x.h"

#define SENDER_LOG_SIZE 128

// One transmitted packet and what came back for it.
typedef struct {
    uint32_t seq;
    uint32_t at_uptime_s;
    float    airtime_ms;
    uint32_t rtt_ms;       // can exceed 65 s: SF12/BW7.8 is ~163 s per exchange
    uint8_t  len;
    uint8_t  type;         // link_pkt_type_t
    bool     acked;
    float    ack_rssi;     // how the receiver heard us
    float    ack_snr;
    float    local_rssi;   // how we heard the acknowledgement
    float    local_snr;
} sender_log_t;

typedef struct {
    bool     running;
    uint32_t next_seq;
    uint32_t tx_count;
    uint32_t ack_count;
    uint32_t timeout_count;
    uint32_t interval_ms;
    uint8_t  payload_len;
    float    airtime_ms;      // for the current profile + payload size
    float    ack_airtime_ms;
    float    duty_cycle;      // fraction of wall time the PA is keyed
    uint32_t last_rtt_ms;
    float    last_ack_rssi, last_ack_snr;
    float    last_local_rssi, last_local_snr;
    uint32_t log_seq;         // total log entries ever written; a cursor for the UI
    uint8_t  peer_charge;     // receiver's solar charger (charge_state_t), from its ACKs
    uint32_t consec_miss;     // exchanges in a row with no answer; 0 while the link is up
} sender_status_t;

esp_err_t sender_start(void);

// Cut the radio task's wait between packets short so it acts on a change at once:
// a queued profile, a push, a stop or a new interval. Pacing is unaffected - the
// task goes back to waiting out whatever remains of the interval.
void sender_wake(void);

void sender_get_status(sender_status_t *out);

// Copy log entries newer than the `since` cursor into out. Returns how many, and
// writes the new cursor to *next.
int sender_copy_log(sender_log_t *out, int max, uint32_t since, uint32_t *next);

void      sender_set_running(bool run);
esp_err_t sender_set_interval(uint32_t ms);
esp_err_t sender_set_payload_len(int len);
void      sender_reset_counters(void);
// Start a fresh discharge cycle at zero, saved at once.
void      sender_reset_discharge_clock(void);

// Ask the radio task to negotiate `cfg` with the receiver and adopt it here too.
// Returns immediately; poll sender_cfg_push_state() for the outcome.
//
// The sender applies the profile whether or not the receiver confirms, because an
// unanswered push usually means only the CFGACK was lost - the receiver has already
// switched, and the sender staying behind is what breaks the link. The switch is
// provisional either way, so a profile that genuinely does not work reverts on both
// ends once the provisional silence window expires.
esp_err_t sender_request_cfg_push(const sx126x_cfg_t *cfg);
bool      sender_cfg_push_busy(void);
// Copies the state string out under the lock; the radio task rewrites it as the
// handshake progresses, so callers must not hold a pointer into it.
void      sender_cfg_push_state(char *out, size_t n);

// The last command the receiver's dashboard sent, and its outcome, e.g.
// "battery test: done". Empty until one arrives.
void      sender_remote_command_state(char *out, size_t n);
