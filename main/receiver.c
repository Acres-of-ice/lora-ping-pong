#include "receiver.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link.h"
#include "storage.h"

static const char *TAG = "receiver";

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

static bool     s_have_batt;
static uint16_t s_batt_pin_mv;
static uint32_t s_batt_uptime_s;

static receiver_log_t s_log[RECEIVER_LOG_SIZE];
static uint32_t       s_log_seq;

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

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
        gap = (uint16_t)(seq - s_last_seq - 1);
        s_lost += gap;
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

static void log_add(uint32_t seq, uint8_t type, uint8_t len, uint16_t gap,
                    const sx126x_rxinfo_t *info, uint32_t uptime_s)
{
    receiver_log_t *e = &s_log[s_log_seq % RECEIVER_LOG_SIZE];
    e->seq         = seq;
    e->at_uptime_s = uptime_s;
    e->rssi        = info->rssi;
    e->snr         = info->snr;
    e->signal_rssi = info->signal_rssi;
    e->gap         = gap;
    e->len         = len;
    e->type        = type;
    s_log_seq++;
}

// Acknowledge a packet, reporting how we heard it so the sender can display the
// reverse direction without a second dashboard.
static void send_ack(uint32_t seq, const sx126x_rxinfo_t *info, uint8_t rx_len)
{
    uint8_t buf[sizeof(link_ack_t)];
    link_ack_t *a = (link_ack_t *)buf;
    link_put_hdr(buf, PKT_ACK, seq);
    a->rssi_x10 = (int16_t)(info->rssi * 10.0f);
    a->snr_x10  = (int16_t)(info->snr * 10.0f);
    a->rx_len   = rx_len;

    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);
    const uint32_t to = (uint32_t)(sx126x_airtime_ms(&cfg, sizeof(buf)) * 2.0f) + 500;
    sx126x_tx(buf, sizeof(buf), to);
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

    lock();
    s_have_batt     = true;
    s_batt_pin_mv   = b->pin_mv;
    s_batt_uptime_s = b->uptime_s;
    unlock();

    storage_append(b->uptime_s, b->pin_mv, info->rssi, info->snr, b->hdr.seq);

    // The packet type is the mode signal: the receiver follows the sender rather
    // than needing to be switched by hand.
    if (link_get_mode() != LINK_MODE_BATTERY) {
        link_set_mode(LINK_MODE_BATTERY);
    }
    ESP_LOGI(TAG, "batt pin=%umV uptime=%us rssi=%.0f snr=%.1f",
             b->pin_mv, (unsigned)b->uptime_s, info->rssi, info->snr);
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
        s_crc_err = st.crc_err;
        s_hdr_err = st.hdr_err;
    }
    unlock();
}

static void receiver_task(void *arg)
{
    uint8_t rx[LINK_MAX_PAYLOAD];
    sx126x_rxinfo_t info;

    sx126x_reset_stats();
    sx126x_rx_start(0);  // continuous
    int64_t last_housekeeping = esp_timer_get_time();

    for (;;) {
        link_profile_tick();

        // Apply any profile queued by an HTTP handler here rather than in the HTTP
        // task, which could otherwise land mid-transmission of an acknowledgement.
        if (link_apply_pending()) {
            sx126x_rx_start(0);
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
                        unlock();

                        if (h->type == PKT_BATT) {
                            handle_batt(rx, n, &info);
                        }

                        lock();
                        const uint32_t up = s_have_batt ? s_batt_uptime_s : 0;
                        log_add(seq, h->type, (uint8_t)n, gap, &info, up);
                        unlock();

                        send_ack(seq, &info, (uint8_t)n);
                        if (h->type == PKT_PING) {
                            ESP_LOGI(TAG, "#%lu %dB rssi=%.0f snr=%.1f sig=%.0f gap=%u",
                                     (unsigned long)seq, n, info.rssi, info.snr,
                                     info.signal_rssi, gap);
                        }
                        // Sending the acknowledgement dropped the chip to
                        // standby; go back to listening.
                        sx126x_rx_start(0);
                        break;
                    }
                    case PKT_CFG:
                        handle_cfg(rx, n);   // also transmits, and reconfigures
                        sx126x_rx_start(0);
                        break;
                    default:
                        break;  // ACK/CFGACK are ours, ignore any echo
                }
            }
        }

        if (esp_timer_get_time() - last_housekeeping > HOUSEKEEPING_MS * 1000) {
            housekeeping();
            last_housekeeping = esp_timer_get_time();
        }
    }
}

esp_err_t receiver_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(receiver_task, "receiver", 5120, NULL, 5, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void receiver_get_status(receiver_status_t *out)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

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
    out->log_seq          = s_log_seq;

    const uint32_t expected = s_have_packet ? (s_last_seq - s_first_seq + 1) : 0;
    const int64_t  last_us  = s_last_rx_us;
    unlock();

    out->pdr = (expected > 0) ? (100.0f * (float)out->rx_count / (float)expected) : 0.0f;
    if (out->pdr > 100.0f) {
        out->pdr = 100.0f;  // duplicates can push the raw ratio past 100
    }
    out->since_last_s = out->have_packet
                            ? (uint32_t)((esp_timer_get_time() - last_us) / 1000000)
                            : 0;

    out->snr_margin  = out->last_snr - sx126x_snr_floor_db(cfg.sf);
    out->link_budget = (float)cfg.tx_dbm - out->last_rssi;
    out->fade_margin = out->last_rssi - sx126x_sensitivity_dbm(&cfg);
}

int receiver_copy_log(receiver_log_t *out, int max, uint32_t since, uint32_t *next)
{
    lock();
    const uint32_t head   = s_log_seq;
    const uint32_t oldest = (head > RECEIVER_LOG_SIZE) ? head - RECEIVER_LOG_SIZE : 0;
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
    lock();
    s_rx_count = s_lost = s_dup = s_out_of_order = 0;
    s_have_packet = false;
    s_first_seq = s_last_seq = 0;
    s_last_rssi = s_last_snr = s_last_signal_rssi = 0.0f;
    unlock();
    sx126x_reset_stats();
    ESP_LOGW(TAG, "Counters reset");
}
