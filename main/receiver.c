#include "receiver.h"

#include <string.h>

#include <stddef.h>

#include "battery.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link.h"
#include "nvs.h"
#include "runstate.h"
#include "solar.h"
#include "storage.h"

// Both packet types carry the sender's charger state at the same place, so one
// read covers them; this keeps the two layouts from drifting apart.
_Static_assert(offsetof(link_ping_t, charge) == offsetof(link_batt_t, charge),
               "PING and BATT must carry the charger state at the same offset");

static const char *TAG = "receiver";

#define NVS_NS        "receiver"
#define NVS_KEY_RATIO "ratio"

// A sequence number this far below the last one means the sender rebooted rather
// than a packet arriving out of order.
#define RESTART_THRESHOLD 100

// How often to refresh the noise floor and the chip's error counters.
#define HOUSEKEEPING_MS 2000

static SemaphoreHandle_t s_mutex;

static uint32_t s_rx_count, s_lost, s_dup, s_out_of_order;
static uint32_t s_first_seq, s_last_seq;
static bool     s_have_packet;
static float    s_last_rssi, s_last_snr, s_last_signal_rssi;
static int64_t  s_last_rx_us;
static float    s_noise_floor;
static uint16_t s_crc_err, s_hdr_err;
// The chip's own counters start from zero at every boot; these carry the totals
// saved before it, so the dashboard's counts survive a reset too.
static uint16_t s_crc_base, s_hdr_base;

static bool     s_have_batt;
static uint16_t s_batt_pin_mv;
static uint32_t s_batt_uptime_s;
static uint16_t s_batt_ratio_x1000;  // the sender's divider ratio; 0 until one arrives
static uint8_t  s_peer_charge = CHARGE_UNKNOWN;  // the sender's charger, from its packets

// Commands for the sender from this board's dashboard. They ride on our ACKs: the
// head of the queue goes out on every one until the sender's CMDACK for it arrives.
#define CMD_QUEUE_LEN    8
#define RUNSTATE_KEY_CMD "cmdid"
static link_cmd_t s_cmdq[CMD_QUEUE_LEN];
static int        s_cmdq_len;
static uint8_t    s_cmd_next_id = 1;  // saved, so a rebooted receiver never reuses an id
static char       s_cmd_state[96];    // the last outcome, for the dashboard

// What the sender last said about itself (in a CMDACK), and when we last heard any
// frame from it. Whether it is running is tracked from every frame: test packets
// mean it is, and a POLL or CMDACK says so either way.
static bool          s_sender_known;
static link_cmdack_t s_sender;
static int64_t       s_sender_at_us;
static bool          s_contact_seen;
static int64_t       s_contact_us;
static bool          s_sender_stopped;

static receiver_log_t s_log[RECEIVER_LOG_SIZE];
static uint32_t       s_log_seq;
static uint32_t       s_log_floor;  // entries below this were dropped by a counter reset

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

// Delivery statistics, saved with every packet so a receiver reset carries on from
// them. Packets the sender sent while this board was down then count as lost,
// which is what they were, rather than the statistics silently starting over.
#define RUNSTATE_KEY "receiver"

typedef struct {
    uint32_t rx, lost, dup, ooo;
    uint32_t first_seq, last_seq;
    uint16_t crc, hdr;
    uint8_t  have_packet;
    uint8_t  reserved[3];
} run_state_t;

static void checkpoint(void)
{
    // Held across the write, so a reset from the dashboard cannot land between
    // the snapshot and the save and then be overwritten by the older numbers.
    lock();
    const run_state_t st = {
        .rx = s_rx_count, .lost = s_lost, .dup = s_dup, .ooo = s_out_of_order,
        .first_seq = s_first_seq, .last_seq = s_last_seq,
        .crc = s_crc_err, .hdr = s_hdr_err, .have_packet = s_have_packet,
    };
    runstate_save(RUNSTATE_KEY, &st, sizeof(st));
    unlock();
}

static void restore_run_state(void)
{
    // Command ids must not repeat one the sender has already acted on, or it would
    // only re-answer instead of acting. With nothing saved, start somewhere random.
    if (!runstate_load(RUNSTATE_KEY_CMD, &s_cmd_next_id, sizeof(s_cmd_next_id)) ||
        s_cmd_next_id == 0) {
        s_cmd_next_id = (uint8_t)(1 + esp_random() % 255);
    }

    run_state_t st;
    if (runstate_load(RUNSTATE_KEY, &st, sizeof(st))) {
        s_rx_count     = st.rx;
        s_lost         = st.lost;
        s_dup          = st.dup;
        s_out_of_order = st.ooo;
        s_first_seq    = st.first_seq;
        s_last_seq     = st.last_seq;
        s_crc_base     = s_crc_err = st.crc;
        s_hdr_base     = s_hdr_err = st.hdr;
        s_have_packet  = st.have_packet != 0;
        if (s_have_packet) {
            ESP_LOGW(TAG, "Resuming run: %lu received, %lu lost, last packet #%lu",
                     (unsigned long)st.rx, (unsigned long)st.lost, (unsigned long)st.last_seq);
        }
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint16_t r = 0;
        if (nvs_get_u16(h, NVS_KEY_RATIO, &r) == ESP_OK &&
            r >= BATTERY_RATIO_MIN_X1000 && r <= BATTERY_RATIO_MAX_X1000) {
            s_batt_ratio_x1000 = r;
        }
        nvs_close(h);
    }
}

// Kept so the dashboard can show volts, and convert the CSV's raw pin readings,
// straight after a reboot rather than waiting for the next battery packet.
static void save_ratio(uint16_t ratio_x1000)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_set_u16(h, NVS_KEY_RATIO, ratio_x1000) == ESP_OK) {
            nvs_commit(h);
        }
        nvs_close(h);
    }
}

// Fold one accepted packet into the sequence statistics. Returns the number of
// sequence numbers missing immediately before it.
static uint16_t account_sequence(uint32_t seq)
{
    uint16_t gap = 0;

    if (!s_have_packet) {
        s_first_seq = seq;
        s_last_seq  = seq;
        s_have_packet = true;
    } else if (seq > s_last_seq) {
        // Counted in full; only the per-packet figure is capped to its field. With
        // the statistics surviving a receiver reset, a gap spanning a long power-off
        // is real now and must not wrap.
        const uint32_t missing = seq - s_last_seq - 1;
        s_lost += missing;
        gap = (missing > UINT16_MAX) ? UINT16_MAX : (uint16_t)missing;
        s_last_seq = seq;
    } else if (seq == s_last_seq) {
        s_dup++;
    } else if (s_last_seq - seq > RESTART_THRESHOLD) {
        // The sender restarted its numbering; begin a fresh run rather than
        // reporting a mountain of losses.
        ESP_LOGW(TAG, "Sender restarted (seq %lu after %lu); resetting statistics",
                 (unsigned long)seq, (unsigned long)s_last_seq);
        s_rx_count = s_lost = s_dup = s_out_of_order = 0;
        s_first_seq = s_last_seq = seq;
    } else {
        s_out_of_order++;
        // It arrived late, so it was previously counted as lost.
        if (s_lost > 0) {
            s_lost--;
        }
    }

    s_rx_count++;
    s_last_rx_us = esp_timer_get_time();
    return gap;
}

// Delivery ratio in percent: packets accepted over sequence numbers spanned. Must
// be called with the lock held.
static float pdr_locked(void)
{
    const uint32_t expected = s_have_packet ? (s_last_seq - s_first_seq + 1) : 0;
    if (expected == 0) {
        return 0.0f;
    }
    const float pdr = 100.0f * (float)s_rx_count / (float)expected;
    return (pdr > 100.0f) ? 100.0f : pdr;  // duplicates can push the raw ratio past 100
}

// Record one accepted packet with every link figure as it stands now, and return a
// copy for the console line. The margins and path loss use signal RSSI, which
// keeps tracking the signal below the noise floor, where packet RSSI flattens out
// at the noise.
static receiver_log_t log_add(uint32_t seq, uint8_t type, uint8_t len, uint16_t gap,
                              const sx126x_rxinfo_t *info)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    receiver_log_t e = {
        .seq         = seq,
        .len         = len,
        .type        = type,
        .gap         = gap,
        .rssi        = info->rssi,
        .signal_rssi = info->signal_rssi,
        .snr         = info->snr,
        .snr_margin  = info->snr - sx126x_snr_floor_db(cfg.sf),
        .fade_margin = info->signal_rssi - sx126x_sensitivity_dbm(&cfg),
        .path_loss   = (float)cfg.tx_dbm - info->signal_rssi,
        .freq_err_hz = info->freq_err_hz,
    };

    lock();
    e.at_uptime_s  = s_have_batt ? s_batt_uptime_s : 0;
    e.noise_floor  = s_noise_floor;
    e.pdr          = pdr_locked();
    e.lost         = s_lost;
    e.dup          = s_dup;
    e.out_of_order = s_out_of_order;
    e.crc_err      = s_crc_err;
    e.hdr_err      = s_hdr_err;
    e.charge       = s_peer_charge;
    if (type == PKT_BATT && s_batt_ratio_x1000 != 0) {
        e.batt_mv = (uint16_t)((uint32_t)s_batt_pin_mv * s_batt_ratio_x1000 / 1000);
    }
    s_log[s_log_seq % RECEIVER_LOG_SIZE] = e;
    s_log_seq++;
    unlock();
    return e;
}

// Acknowledge a packet, reporting how we heard it so the sender can display the
// reverse direction without a second dashboard.
static void send_ack(uint32_t seq, const sx126x_rxinfo_t *info, uint8_t rx_len)
{
    uint8_t buf[LINK_ACK_MAX_LEN];
    link_ack_t *a = (link_ack_t *)buf;
    link_put_hdr(buf, PKT_ACK, seq);
    a->rssi_x10 = (int16_t)(info->rssi * 10.0f);
    a->snr_x10  = (int16_t)(info->snr * 10.0f);
    a->rx_len   = rx_len;
    a->charge   = (uint8_t)charge_state();

    // The oldest waiting command for the sender rides along, every time, until
    // the sender confirms it. Plain ACKs stay short when nothing is waiting.
    uint8_t len = sizeof(link_ack_t);
    lock();
    if (s_cmdq_len > 0) {
        memcpy(buf + sizeof(link_ack_t), &s_cmdq[0], sizeof(link_cmd_t));
        len = LINK_ACK_MAX_LEN;
    }
    unlock();

    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);
    const uint32_t to = (uint32_t)(sx126x_airtime_ms(&cfg, len) * 2.0f) + 500;
    sx126x_tx(buf, len, to);
}

// Append a command for the sender; the lock must be held. False when the queue is
// full. The next id is saved here, under the lock, so two callers cannot save theirs
// out of order and leave an id behind that a rebooted receiver would then reuse.
static bool queue_locked(uint8_t op, uint32_t arg, const sx126x_cfg_t *cfg)
{
    if (s_cmdq_len >= CMD_QUEUE_LEN) {
        return false;
    }
    link_cmd_t *c = &s_cmdq[s_cmdq_len++];
    memset(c, 0, sizeof(*c));
    c->id  = s_cmd_next_id;
    c->op  = op;
    c->arg = arg;
    if (cfg != NULL) {
        link_cfg_to_wire(cfg, &c->cfg);
    }
    s_cmd_next_id = (s_cmd_next_id == 255) ? 1 : s_cmd_next_id + 1;
    runstate_save(RUNSTATE_KEY_CMD, &s_cmd_next_id, sizeof(s_cmd_next_id));
    return true;
}

// The sender's answer to a command: take it off the queue if it is the one at the
// head (a repeat after a lost answer may not be), and keep the settings it reports.
static void handle_cmdack(const uint8_t *rx, int n)
{
    if (!link_check(rx, n, PKT_CMDACK, sizeof(link_cmdack_t))) {
        return;
    }
    const link_cmdack_t *a = (const link_cmdack_t *)rx;
    const uint16_t ratio   = a->ratio_x1000;
    const bool     ratio_ok = ratio >= BATTERY_RATIO_MIN_X1000 && ratio <= BATTERY_RATIO_MAX_X1000;

    lock();
    s_sender         = *a;
    s_sender_known   = true;
    s_sender_at_us   = esp_timer_get_time();
    s_sender_stopped = !a->running;
    // The same ratio a battery packet carries, so a calibration sent from this
    // dashboard shows at once rather than with the next battery sample.
    const bool ratio_changed = ratio_ok && ratio != s_batt_ratio_x1000;
    if (ratio_changed) {
        s_batt_ratio_x1000 = ratio;
    }
    if (s_cmdq_len > 0 && s_cmdq[0].id == a->id) {
        char what[56];
        link_cmd_describe(&s_cmdq[0], what, sizeof(what));
        snprintf(s_cmd_state, sizeof(s_cmd_state), "%s: %s", what,
                 a->result == LINK_CMD_DONE ? "done" : "refused by the sender");
        const uint8_t op = s_cmdq[0].op;
        memmove(&s_cmdq[0], &s_cmdq[1], (size_t)(s_cmdq_len - 1) * sizeof(link_cmd_t));
        s_cmdq_len--;
        ESP_LOGI(TAG, "Sender: %s", s_cmd_state);
        // A reboot is answered before the restart, so that answer still carries the
        // old settings (WiFi off, say). Ask again once it is back up.
        if (op == LINK_CMD_REBOOT) {
            queue_locked(LINK_CMD_STATUS, 0, NULL);
        }
    }
    unlock();

    if (ratio_changed) {
        save_ratio(ratio);
    }
    // The receiver's mode follows the sender's, as it does from the packet type,
    // but a stopped sender sends no packets to follow.
    const link_mode_t mode = (a->mode == LINK_MODE_BATTERY) ? LINK_MODE_BATTERY : LINK_MODE_RANGE;
    if (link_get_mode() != mode) {
        link_set_mode(mode);
    }
}

// Accept a profile pushed by the sender. The acknowledgement goes out on the old
// profile, which is where the sender is still listening, then we switch and arm our
// own provisional timer. The sender switches whether or not that CFGACK arrives, so
// a lost one costs nothing: both ends end up on the new profile, and both revert
// together if it turns out to carry no traffic.
static void handle_cfg(const uint8_t *rx, int n)
{
    if (!link_check(rx, n, PKT_CFG, sizeof(link_cfgpkt_t))) {
        return;
    }
    const link_cfgpkt_t *p = (const link_cfgpkt_t *)rx;

    sx126x_cfg_t old, want;
    link_get_cfg(&old);
    link_cfg_from_wire(&p->cfg, &want);
    if (!link_cfg_valid(&want)) {
        // Not acknowledged: the sender applies provisionally either way, finds
        // no traffic, and reverts - which is the right outcome for a bad profile.
        ESP_LOGE(TAG, "Ignoring pushed profile outside what this radio supports "
                      "(%lu Hz SF%u CR4/%u %+d dBm)",
                 (unsigned long)want.freq_hz, want.sf, want.cr, want.tx_dbm);
        return;
    }

    uint8_t ack[sizeof(link_hdr_t)];
    link_put_hdr(ack, PKT_CFGACK, p->hdr.seq);
    const uint32_t to = (uint32_t)(sx126x_airtime_ms(&old, sizeof(ack)) * 2.0f) + 500;
    sx126x_tx(ack, sizeof(ack), to);

    ESP_LOGW(TAG, "Accepting pushed profile: SF%u BW%lu CR4/%u %+d dBm",
             want.sf, (unsigned long)sx126x_bw_hz(want.bw), want.cr, want.tx_dbm);

    // Use the sender's window rather than deriving our own: it knows the interval
    // and we do not, and both ends must expire together. Fall back to a local
    // estimate if an older sender sends nothing usable.
    uint32_t silence_s = p->rollback_s;
    if (silence_s < LINK_PROV_SILENCE_MIN_S) {
        silence_s = link_profile_silence_s(&want, 0);
    }

    link_set_cfg(&want, false);
    link_profile_provisional(&old, silence_s);
}

// The sample is logged exactly as it arrived. No scaling happens here: the sender
// reports the sense pin as measured, and turning that into a battery voltage is a
// multiplication done against the CSV, off-device.
static void handle_batt(const uint8_t *rx, int n, const sx126x_rxinfo_t *info)
{
    if (!link_check(rx, n, PKT_BATT, sizeof(link_batt_t))) {
        return;
    }
    const link_batt_t *b = (const link_batt_t *)rx;
    const uint16_t ratio = b->ratio_x1000;
    const bool ratio_ok  = ratio >= BATTERY_RATIO_MIN_X1000 && ratio <= BATTERY_RATIO_MAX_X1000;

    lock();
    s_have_batt     = true;
    s_batt_pin_mv   = b->pin_mv;
    s_batt_uptime_s = b->uptime_s;
    const bool ratio_changed = ratio_ok && ratio != s_batt_ratio_x1000;
    if (ratio_changed) {
        s_batt_ratio_x1000 = ratio;
    }
    unlock();

    if (ratio_changed) {
        save_ratio(ratio);  // only when the sender is recalibrated, so rare
    }
    storage_append(b->uptime_s, b->pin_mv, info->rssi, info->snr, b->hdr.seq);

    // The packet type is the mode signal: the receiver follows the sender rather
    // than needing to be switched by hand.
    if (link_get_mode() != LINK_MODE_BATTERY) {
        link_set_mode(LINK_MODE_BATTERY);
    }
    if (ratio_ok) {
        ESP_LOGI(TAG, "batt %.3fV (pin %umV) uptime=%us",
                 b->pin_mv * ratio / 1e6, b->pin_mv, (unsigned)b->uptime_s);
    } else {
        ESP_LOGI(TAG, "batt pin=%umV uptime=%us", b->pin_mv, (unsigned)b->uptime_s);
    }
}

static void housekeeping(void)
{
    // Only meaningful while the chip is actually listening.
    const float rssi = sx126x_rssi_inst();

    sx126x_stats_t st;
    const bool ok = (sx126x_get_stats(&st) == ESP_OK);

    lock();
    if (rssi < 0.0f) {
        s_noise_floor = rssi;
    }
    if (ok) {
        s_crc_err = s_crc_base + st.crc_err;
        s_hdr_err = s_hdr_base + st.hdr_err;
    }
    unlock();
}

// One line per packet with everything known about the link: this packet's signal,
// its margins against what the profile can decode, and the running totals. Printed
// from the logged entry, so it matches the dashboard's table figure for figure.
static void log_packet(const receiver_log_t *e)
{
    ESP_LOGI(TAG, "#%lu %uB rssi=%.0f sig=%.0f snr=%.1f snr_margin=%+.1f fade_margin=%+.1f "
                  "pathloss=%.0f ferr=%+.0fHz noise=%.0f gap=%u pdr=%.1f%% lost=%lu dup=%lu "
                  "ooo=%lu crc=%u hdr=%u",
             (unsigned long)e->seq, e->len, e->rssi, e->signal_rssi, e->snr, e->snr_margin,
             e->fade_margin, e->path_loss, e->freq_err_hz, e->noise_floor, e->gap, e->pdr,
             (unsigned long)e->lost, (unsigned long)e->dup, (unsigned long)e->out_of_order,
             e->crc_err, e->hdr_err);
}

static void receiver_task(void *arg)
{
    uint8_t rx[LINK_MAX_PAYLOAD];
    sx126x_rxinfo_t info;

    sx126x_reset_stats();
    // False whenever the chip may have left continuous receive - anything that
    // reconfigures it ends in standby, as does a failed rx_start - and re-armed
    // at the top of every pass, so no path can leave the receiver deaf.
    bool listening = false;
    int64_t last_housekeeping = esp_timer_get_time();

    for (;;) {
        // A silence revert reconfigures the radio, leaving it in standby. Not
        // re-arming after one left the receiver deaf for good - just when the
        // rollback was meant to bring the link back.
        if (link_profile_tick()) {
            listening = false;
        }

        // Apply any profile queued by an HTTP handler here rather than in the HTTP
        // task, which could otherwise land mid-transmission of an acknowledgement.
        if (link_apply_pending()) {
            listening = false;
        }

        if (!listening) {
            listening = (sx126x_rx_start(0) == ESP_OK);  // continuous
        }

        const int n = sx126x_rx_wait(rx, sizeof(rx), &info, 500);

        // Anything shorter than a header is noise that happened to pass CRC, or a
        // stray packet from another network; it must not be read as a header.
        if (n >= (int)sizeof(link_hdr_t)) {
            const link_hdr_t *h = (const link_hdr_t *)rx;
            if (h->magic == LINK_MAGIC && h->ver == LINK_VERSION) {
                // Anything that decodes shows the profile works in this direction.
                // Not enough to commit it on its own - see link_profile_silence_s().
                link_profile_traffic_ok();

                switch (h->type) {
                    case PKT_PING:
                    case PKT_BATT: {
                        const uint32_t seq = h->seq;
                        lock();
                        const uint16_t gap = account_sequence(seq);
                        s_last_rssi        = info.rssi;
                        s_last_snr         = info.snr;
                        s_last_signal_rssi = info.signal_rssi;
                        if (n >= (int)sizeof(link_ping_t)) {
                            s_peer_charge = ((const link_ping_t *)rx)->charge;
                        }
                        unlock();

                        if (h->type == PKT_BATT) {
                            handle_batt(rx, n, &info);
                        } else if (link_get_mode() != LINK_MODE_RANGE) {
                            // Follow the sender back out of a battery test too.
                            // The receiver has no mode control of its own, so
                            // without this it stayed on "battery" for good.
                            link_set_mode(LINK_MODE_RANGE);
                        }

                        const receiver_log_t entry =
                            log_add(seq, h->type, (uint8_t)n, gap, &info);

                        send_ack(seq, &info, (uint8_t)n);
                        // Sending the acknowledgement dropped the chip to
                        // standby; go back to listening before logging, so the
                        // log line cannot eat into the next packet's preamble.
                        listening = (sx126x_rx_start(0) == ESP_OK);
                        log_packet(&entry);
                        checkpoint();
                        break;
                    }
                    case PKT_CFG:
                        handle_cfg(rx, n);   // also transmits, and reconfigures
                        listening = (sx126x_rx_start(0) == ESP_OK);
                        break;
                    case PKT_POLL:
                        // The sender asking for commands. Answered like a ping,
                        // so a waiting command rides along, but counted nowhere:
                        // it is not test traffic.
                        if (n >= (int)sizeof(link_poll_t)) {
                            const link_poll_t *p = (const link_poll_t *)rx;
                            lock();
                            s_peer_charge    = p->charge;
                            s_sender_stopped = !p->running;
                            unlock();
                        }
                        send_ack(h->seq, &info, (uint8_t)n);
                        listening = (sx126x_rx_start(0) == ESP_OK);
                        break;
                    case PKT_CMDACK:
                        handle_cmdack(rx, n);  // no reply, so still listening
                        break;
                    default:
                        break;  // ACK/CFGACK are ours, ignore any echo
                }

                // Any frame is contact, and test packets mean it is running.
                lock();
                s_contact_seen = true;
                s_contact_us   = esp_timer_get_time();
                if (h->type == PKT_PING || h->type == PKT_BATT) {
                    s_sender_stopped = false;
                }
                unlock();
            }
        }

        if (esp_timer_get_time() - last_housekeeping > HOUSEKEEPING_MS * 1000) {
            housekeeping();
            last_housekeeping = esp_timer_get_time();
        }
    }
}

esp_err_t receiver_queue_command(uint8_t op, uint32_t arg, const sx126x_cfg_t *cfg)
{
    lock();
    const bool queued = queue_locked(op, arg, cfg);
    unlock();
    return queued ? ESP_OK : ESP_ERR_NO_MEM;
}

void receiver_cancel_commands(void)
{
    lock();
    if (s_cmdq_len > 0) {
        snprintf(s_cmd_state, sizeof(s_cmd_state), "%d waiting command%s cancelled",
                 s_cmdq_len, s_cmdq_len == 1 ? "" : "s");
    }
    s_cmdq_len = 0;
    unlock();
}

esp_err_t receiver_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    restore_run_state();  // before the task starts, so the first packet continues it
    if (xTaskCreate(receiver_task, "receiver", 5120, NULL, 5, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void receiver_get_status(receiver_status_t *out)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    memset(out, 0, sizeof(*out));  // the sender's fields stay 0 until it has answered
    lock();
    out->rx_count         = s_rx_count;
    out->lost             = s_lost;
    out->dup              = s_dup;
    out->out_of_order     = s_out_of_order;
    out->crc_err          = s_crc_err;
    out->hdr_err          = s_hdr_err;
    out->have_packet      = s_have_packet;
    out->last_rssi        = s_last_rssi;
    out->last_snr         = s_last_snr;
    out->last_signal_rssi = s_last_signal_rssi;
    out->last_seq         = s_last_seq;
    out->noise_floor      = s_noise_floor;
    out->have_batt        = s_have_batt;
    out->batt_pin_mv      = s_batt_pin_mv;
    out->batt_uptime_s    = s_batt_uptime_s;
    out->batt_ratio_x1000 = s_batt_ratio_x1000;
    out->batt_mv          = (uint32_t)s_batt_pin_mv * s_batt_ratio_x1000 / 1000;
    out->peer_charge      = s_peer_charge;

    const int64_t now = esp_timer_get_time();
    out->cmd_queued = s_cmdq_len;
    out->cmd_waiting[0] = '\0';
    if (s_cmdq_len > 0) {
        link_cmd_describe(&s_cmdq[0], out->cmd_waiting, sizeof(out->cmd_waiting));
    }
    strncpy(out->cmd_state, s_cmd_state, sizeof(out->cmd_state) - 1);
    out->cmd_state[sizeof(out->cmd_state) - 1] = '\0';
    out->sender_known = s_sender_known;
    if (s_sender_known) {
        out->sender_age_s       = (uint32_t)((now - s_sender_at_us) / 1000000);
        out->sender_running     = s_sender.running != 0;
        out->sender_mode        = s_sender.mode;
        out->sender_load        = s_sender.load != 0;
        out->sender_wifi        = s_sender.wifi != 0;
        out->sender_interval_ms = s_sender.interval_ms;
        out->sender_payload     = s_sender.payload_len;
        out->sender_ratio_x1000 = s_sender.ratio_x1000;
    }
    out->contact_seen   = s_contact_seen;
    out->contact_age_s  = s_contact_seen ? (uint32_t)((now - s_contact_us) / 1000000) : 0;
    out->sender_stopped = s_sender_stopped;
    out->log_seq          = s_log_seq;
    out->pdr              = pdr_locked();
    const int64_t last_us = s_last_rx_us;
    unlock();

    out->since_last_s = out->have_packet
                            ? (uint32_t)((esp_timer_get_time() - last_us) / 1000000)
                            : 0;

    // Signal RSSI rather than packet RSSI: below the noise floor packet RSSI reads
    // the noise, which overstated the fade margin by up to the SNR deficit at the
    // edge of range, exactly where it matters.
    out->snr_margin  = out->last_snr - sx126x_snr_floor_db(cfg.sf);
    out->link_budget = (float)cfg.tx_dbm - out->last_signal_rssi;
    out->fade_margin = out->last_signal_rssi - sx126x_sensitivity_dbm(&cfg);
}

int receiver_copy_log(receiver_log_t *out, int max, uint32_t since, uint32_t *next)
{
    lock();
    const uint32_t head = s_log_seq;
    uint32_t oldest = (head > RECEIVER_LOG_SIZE) ? head - RECEIVER_LOG_SIZE : 0;
    if (oldest < s_log_floor) {
        oldest = s_log_floor;
    }
    uint32_t from = (since < oldest) ? oldest : since;

    int n = 0;
    for (uint32_t i = from; i < head && n < max; i++, n++) {
        out[n] = s_log[i % RECEIVER_LOG_SIZE];
    }
    unlock();

    *next = head;
    return n;
}

void receiver_reset_counters(void)
{
    sx126x_reset_stats();  // the chip's counts first, so no pass re-adds the old ones
    lock();
    s_rx_count = s_lost = s_dup = s_out_of_order = 0;
    s_have_packet = false;
    s_first_seq = s_last_seq = 0;
    s_last_rssi = s_last_snr = s_last_signal_rssi = 0.0f;
    // Cleared here too, not only in the chip: the cached copies otherwise showed
    // the old counts until the next housekeeping pass.
    s_crc_err = s_hdr_err = 0;
    s_crc_base = s_hdr_base = 0;
    // Without this the dashboard's log, which restarts from cursor 0 after a
    // reset, refilled at once with every packet still in the ring.
    s_log_floor = s_log_seq;
    unlock();
    checkpoint();  // or a reboot would bring the old counts back
    ESP_LOGW(TAG, "Counters reset");
}
