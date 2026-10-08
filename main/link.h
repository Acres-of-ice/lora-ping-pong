#pragma once

// Everything the sender and receiver must agree on: the over-the-air packet
// format, the active radio profile (persisted to NVS), the test mode, and the
// auto-rollback timer that protects an over-the-air config change.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sx126x.h"

#define LINK_MAGIC       0x524C  // 'LR'
// v3 reported the raw sense-pin voltage instead of a divider-corrected one. v4
// kept that raw reading and added the sender's calibrated divider ratio alongside,
// so the receiver can show the battery voltage too. v5 has each board report its
// solar charger state to the other: the sender in every PING and BATT, the receiver
// in every ACK. v6 lets the receiver's dashboard command the sender: commands ride
// on its ACKs, and a stopped sender POLLs so it can still be reached.
//
// Every bump drops frames from the other version rather than misreading them: a
// half-flashed pair goes quiet rather than plotting a wrong graph.
#define LINK_VERSION     6
#define LINK_MAX_PAYLOAD 255

// A newly applied profile is provisional: it reverts to the previous one after a
// spell of silence, and only becomes permanent after carrying traffic for a while.
//
// Those windows are DERIVED from the profile, not fixed, because one exchange can
// legitimately take minutes at the slow end - SF12 at BW 7.81 kHz is ~163 s for a
// 255-byte packet plus its acknowledgement. Any constant short enough to be useful
// at BW 125 kHz would expire mid-packet down there and revert a perfectly good
// profile. See link_profile_silence_s().
//
// This is the floor for that derivation, covering the fast profiles where three
// exchanges pass in a couple of seconds and reverting that eagerly would be twitchy.
#define LINK_PROV_SILENCE_MIN_S 90

// Silence window as a multiple of the expected exchange cadence: enough missed
// exchanges to be convincing rather than one unlucky packet.
#define LINK_PROV_SILENCE_CYCLES 3

// The commit window is this multiple of the silence window. Keeping it a multiple
// rather than a separate constant preserves the invariant the convergence argument
// depends on: silence must always expire BEFORE commit. If a profile works in one
// direction only, the receiver keeps hearing pings and would otherwise commit,
// while the sender - getting no acks - reverts. Silence firing first makes the
// receiver follow the sender back instead of stranding itself on a profile nobody
// is transmitting.
#define LINK_PROV_COMMIT_MULTIPLE 2

typedef enum {
    PKT_PING   = 1,
    PKT_ACK    = 2,
    PKT_BATT   = 3,
    PKT_CFG    = 4,
    PKT_CFGACK = 5,
    PKT_POLL   = 6,  // sender asking for commands; see link_poll_t
    PKT_CMDACK = 7,  // sender: done with a command from the receiver
} link_pkt_type_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  type;
    uint8_t  ver;
    uint32_t seq;
} link_hdr_t;

// A range-test packet. Anything past these fields is filler out to the configured
// payload size.
typedef struct __attribute__((packed)) {
    link_hdr_t hdr;
    uint8_t    charge;  // sender's solar charger state, charge_state_t
} link_ping_t;

// Sent back for every PING and BATT. Carries the receiver's own view of the link so
// the sender can show both directions without a second dashboard.
typedef struct __attribute__((packed)) {
    link_hdr_t hdr;
    int16_t    rssi_x10;
    int16_t    snr_x10;
    uint8_t    rx_len;
    uint8_t    charge;  // receiver's solar charger state, charge_state_t
} link_ack_t;

// One battery sample, sent as-measured.
//
// pin_mv is the calibrated voltage at the sense pin and nothing else: no divider,
// no scaling, no correction. The divider ratio travels beside it rather than baked
// into it, so the receiver's CSV keeps the raw reading and a later recalibration
// still converts the whole run, not just the samples taken after it.
typedef struct __attribute__((packed)) {
    link_hdr_t hdr;
    uint8_t    charge;       // sender's solar charger state, charge_state_t
    uint16_t   pin_mv;       // ADC millivolts at the sense pin, unscaled
    uint32_t   uptime_s;     // sender's discharge-cycle clock
    uint16_t   ratio_x1000;  // sender's divider ratio: battery mV = pin_mv * this / 1000
} link_batt_t;

// Radio profile in wire form. Fixed width and packed so it survives both NVS
// round-trips and the air.
typedef struct __attribute__((packed)) {
    uint32_t freq_hz;
    uint8_t  sf;
    uint8_t  bw;
    uint8_t  cr;
    uint16_t preamble;
    int8_t   tx_dbm;
    uint8_t  sync_word;
    uint8_t  crc_on;
} link_cfg_wire_t;

typedef struct __attribute__((packed)) {
    link_hdr_t      hdr;
    link_cfg_wire_t cfg;
    uint16_t        rollback_s;
} link_cfgpkt_t;

// ---- Commands from the receiver's dashboard to the sender ----
//
// The link is half-duplex and the sender leads: it transmits, then listens only
// for its acknowledgement. So a command rides on an ACK, appended after link_ack_t,
// and is repeated on every ACK until the sender's CMDACK for it arrives. The
// sender acts on each id once, so a repeat after a lost CMDACK only repeats the
// reply. A stopped sender sends a short POLL every few seconds so there is still
// an ACK to carry "start", and any sender POLLs once after each command so a
// queue of them drains in seconds even at a long interval.
typedef enum {
    LINK_CMD_STATUS         = 1,   // do nothing; just report the settings
    LINK_CMD_RUN            = 2,   // arg: 0 stop, 1 start
    LINK_CMD_MODE           = 3,   // arg: link_mode_t
    LINK_CMD_INTERVAL       = 4,   // arg: ms between packets
    LINK_CMD_PAYLOAD        = 5,   // arg: bytes per packet
    LINK_CMD_RESET_COUNTERS = 6,
    LINK_CMD_PUSH_CFG       = 7,   // cfg: negotiate it with the receiver, as from the sender
    LINK_CMD_LOAD           = 8,   // arg: max drain 0 off, 1 on
    LINK_CMD_RESET_CLOCK    = 9,
    LINK_CMD_CALIBRATE      = 10,  // arg: battery mV, read off a multimeter
    LINK_CMD_RATIO_RESET    = 11,
    LINK_CMD_WIFI_OFF       = 12,
    LINK_CMD_REBOOT         = 13,
} link_cmd_op_t;

typedef struct __attribute__((packed)) {
    uint8_t         id;   // 1..255, never 0
    uint8_t         op;   // link_cmd_op_t
    uint32_t        arg;
    link_cfg_wire_t cfg;  // LINK_CMD_PUSH_CFG only
} link_cmd_t;

// The longest ACK, one carrying a command. The sender's listening window is sized
// for it.
#define LINK_ACK_MAX_LEN (sizeof(link_ack_t) + sizeof(link_cmd_t))

// A stopped sender's check-in every few seconds. A running one sends one too, straight
// after acting on a command, to collect the next without waiting out its interval.
// Answered with an ACK like a ping, but counted nowhere: it is not test traffic.
typedef struct __attribute__((packed)) {
    link_hdr_t hdr;
    uint8_t    charge;   // sender's solar charger state, as in a ping
    uint8_t    running;  // so the receiver can tell the two apart
} link_poll_t;

enum { LINK_CMD_DONE = 0, LINK_CMD_REFUSED = 1 };

// The sender's reply to a command, with its settings as they stand afterwards so
// the receiver's dashboard can show them.
typedef struct __attribute__((packed)) {
    link_hdr_t hdr;
    uint8_t    id;           // the command this answers
    uint8_t    result;       // LINK_CMD_DONE / LINK_CMD_REFUSED
    uint8_t    running;
    uint8_t    mode;         // link_mode_t
    uint8_t    load;         // max drain on
    uint8_t    wifi;         // dashboard WiFi on
    uint32_t   interval_ms;
    uint8_t    payload_len;
    uint16_t   ratio_x1000;  // battery divider
} link_cmdack_t;

// Short name for logs and the dashboards, e.g. "mode".
const char *link_cmd_name(uint8_t op);
// A command with its argument, e.g. "interval 2000 ms" or "battery test".
void link_cmd_describe(const link_cmd_t *c, char *out, size_t n);

typedef enum {
    LINK_MODE_RANGE   = 0,
    LINK_MODE_BATTERY = 1,
} link_mode_t;

// Load the saved profile (or the Kconfig defaults) and push it to the radio.
esp_err_t link_init(void);

void      link_get_cfg(sx126x_cfg_t *out);
esp_err_t link_set_cfg(const sx126x_cfg_t *cfg, bool persist);
esp_err_t link_persist_cfg(void);

link_mode_t link_get_mode(void);
esp_err_t   link_set_mode(link_mode_t mode);
const char *link_mode_str(link_mode_t mode);

void link_cfg_to_wire(const sx126x_cfg_t *cfg, link_cfg_wire_t *out);
void link_cfg_from_wire(const link_cfg_wire_t *w, sx126x_cfg_t *out);

// True when two profiles are identical on the air. Used to skip a redundant
// reconfigure, which would otherwise re-arm the provisional timer for no reason.
bool link_cfg_equal(const sx126x_cfg_t *a, const sx126x_cfg_t *b);

// The SX1262's whole tuning range. The Ra-01SH's antenna matching is built for
// 803-930 MHz: outside that the radio still works, but range drops sharply.
#define LINK_FREQ_MIN_HZ 150000000u
#define LINK_FREQ_MAX_HZ 960000000u

// True when every field of a profile is something this radio can run. Checked on
// profiles arriving over the air: CRC only proves the frame was not corrupted, not
// that the sender (or someone else's device on the same sync word) sent sense.
bool link_cfg_valid(const sx126x_cfg_t *cfg);

// Fill a header in place and return the header size.
int  link_put_hdr(void *buf, link_pkt_type_t type, uint32_t seq);
// Validate magic/version/type and that len covers need_len bytes.
bool link_check(const void *buf, int len, link_pkt_type_t type, int need_len);

// ---- Bandwidth table, shared with the dashboards ----
#define LINK_BW_COUNT 10
extern const uint32_t LINK_BW_HZ[LINK_BW_COUNT];
extern const uint8_t  LINK_BW_CODE[LINK_BW_COUNT];
uint8_t link_bw_from_hz(uint32_t hz);

// ---- Applying a profile ----
// Queue a profile for the radio task to apply at a safe point. HTTP handlers MUST
// use this rather than link_set_cfg(): sx126x_tx() releases the driver lock while
// it waits for TxDone, so applying from the HTTP task lands a SetStandby in the
// middle of a transmission and silently kills it.
esp_err_t link_request_apply(const sx126x_cfg_t *cfg, bool persist);
// True while a queued profile is still waiting for the radio task. The dashboards
// use it to hold the form on the submitted values until they are live.
bool      link_apply_queued(void);
// Called by the radio task; applies any queued profile. True if it applied one.
bool      link_apply_pending(void);

// ---- Provisional profiles / auto-rollback ----

// How long to tolerate silence under `cfg` before reverting, in seconds:
// LINK_PROV_SILENCE_CYCLES worth of (one full-size exchange + cadence_ms), floored
// at LINK_PROV_SILENCE_MIN_S. Pass the sender's inter-packet interval as
// cadence_ms; pass 0 if unknown.
//
// Sized off a 255-byte payload regardless of the configured size, so the estimate
// stays valid if the payload is raised later, and so the receiver - which knows
// neither the payload size nor the interval - can compute the same figure.
uint32_t link_profile_silence_s(const sx126x_cfg_t *cfg, uint32_t cadence_ms);

// Mark the current profile provisional, falling back to `fallback` if it does not
// prove itself within `silence_s`. Both ends must use the same value, so the sender
// computes it and ships it in the CFG frame.
void     link_profile_provisional(const sx126x_cfg_t *fallback, uint32_t silence_s);
// Proof the link works under the current profile: an ack on the sender, any valid
// packet on the receiver. Extends the deadline rather than committing outright.
void     link_profile_traffic_ok(void);
bool     link_profile_is_provisional(void);
// Seconds of continued silence before the profile reverts.
uint32_t link_profile_revert_in_s(void);
// Call periodically from the radio task; commits or reverts as the timers expire.
// True when it reverted: that reconfigures the radio and leaves it in standby, so
// a receiver has to go back to listening.
bool     link_profile_tick(void);

// Shared JSON fragment: the radio profile plus everything derived from it
// (airtime, bitrate, sensitivity, LDRO) for a given payload size. No braces, so
// callers can embed it. Returns bytes written.
int link_cfg_json(char *buf, size_t n, const sx126x_cfg_t *cfg, uint8_t payload_len);
