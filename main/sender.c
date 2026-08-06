#include "sender.h"

#include <string.h>

#include "battery.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link.h"
#include "nvs.h"
#include "powerload.h"
#include "sdkconfig.h"

static const char *TAG = "sender";

#define CFG_PUSH_RETRIES 3

#define NVS_NS           "sender"
#define NVS_KEY_INTERVAL "interval"
#define NVS_KEY_PAYLOAD  "payload"

static SemaphoreHandle_t s_mutex;

static bool     s_running = true;
static uint32_t s_interval_ms = CONFIG_SENDER_DEFAULT_INTERVAL_MS;
static uint8_t  s_payload_len = CONFIG_SENDER_DEFAULT_PAYLOAD;
static uint32_t s_seq = 1;
// Control packets get their own numbering. Sharing the ping counter punched holes
// in it (a 3-retry push burned 3 sequence numbers), and the receiver counts those
// gaps as lost packets - so a config push used to dent the delivery ratio.
static uint32_t s_ctrl_seq = 1;
static uint32_t s_tx_count, s_ack_count, s_timeout_count;
static uint32_t s_last_rtt_ms;
static float    s_last_ack_rssi, s_last_ack_snr;
static float    s_last_local_rssi, s_last_local_snr;

static sender_log_t s_log[SENDER_LOG_SIZE];
static uint32_t     s_log_seq;   // total entries ever written

// Config-push handshake, owned by the radio task but requested from HTTP.
static bool         s_push_req;
static sx126x_cfg_t s_push_cfg;
static char         s_push_state[96] = "idle";

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

static void push_state(const char *s)
{
    lock();
    strncpy(s_push_state, s, sizeof(s_push_state) - 1);
    s_push_state[sizeof(s_push_state) - 1] = '\0';
    unlock();
}

// Interval and payload size are settings, not scratch state: a multi-day battery
// run must come back from a brownout on the same cadence it started with, or the
// discharge curve changes slope for a reason that has nothing to do with the cell.
static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;  // never saved yet -> keep the Kconfig defaults
    }
    uint32_t interval = 0;
    if (nvs_get_u32(h, NVS_KEY_INTERVAL, &interval) == ESP_OK && interval <= 600000) {
        s_interval_ms = interval;  // 0 is valid: back-to-back
    }
    uint8_t payload = 0;
    if (nvs_get_u8(h, NVS_KEY_PAYLOAD, &payload) == ESP_OK &&
        payload >= sizeof(link_batt_t)) {
        s_payload_len = payload;
    }
    nvs_close(h);
    ESP_LOGI(TAG, "Settings from NVS: interval %ums, payload %uB",
             (unsigned)s_interval_ms, s_payload_len);
}

static void save_settings(void)
{
    lock();
    const uint32_t interval = s_interval_ms;
    const uint8_t  payload  = s_payload_len;
    unlock();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_u32(h, NVS_KEY_INTERVAL, interval) == ESP_OK &&
        nvs_set_u8(h, NVS_KEY_PAYLOAD, payload) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

static void log_add(const sender_log_t *e)
{
    lock();
    s_log[s_log_seq % SENDER_LOG_SIZE] = *e;
    s_log_seq++;
    unlock();
}

// Round-trip budget for one exchange. Scaled off the profile rather than fixed:
// at SF12 with a 255-byte payload a single packet is already ~9 s in the air.
static uint32_t tx_timeout_ms(const sx126x_cfg_t *cfg, uint8_t len)
{
    return (uint32_t)(sx126x_airtime_ms(cfg, len) * 2.0f) + 500;
}

static uint32_t ack_timeout_ms(const sx126x_cfg_t *cfg)
{
    return (uint32_t)(sx126x_airtime_ms(cfg, sizeof(link_ack_t)) * 2.0f) + 500;
}

// Transmit one packet and wait for its acknowledgement. Fills `e`.
static void exchange(link_pkt_type_t type, uint8_t *buf, uint8_t len, sender_log_t *e)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    const uint32_t seq = s_seq++;
    link_put_hdr(buf, type, seq);

    memset(e, 0, sizeof(*e));
    e->seq         = seq;
    e->len         = len;
    e->type        = (uint8_t)type;
    e->airtime_ms  = sx126x_airtime_ms(&cfg, len);
    e->at_uptime_s = battery_uptime_s();

    const int64_t t0 = esp_timer_get_time();
    if (sx126x_tx(buf, len, tx_timeout_ms(&cfg, len)) != ESP_OK) {
        lock();
        s_tx_count++;
        s_timeout_count++;
        unlock();
        return;
    }
    lock();
    s_tx_count++;
    unlock();

    // Listen for the acknowledgement.
    const uint32_t to = ack_timeout_ms(&cfg);
    sx126x_rx_start(to);

    uint8_t rx[LINK_MAX_PAYLOAD];
    sx126x_rxinfo_t info = { 0 };
    const int64_t deadline = esp_timer_get_time() + (int64_t)to * 1000;

    while (esp_timer_get_time() < deadline) {
        const int n = sx126x_rx_wait(rx, sizeof(rx), &info, 100);
        if (n <= 0) {
            continue;  // nothing yet, or a CRC error - keep listening
        }
        if (!link_check(rx, n, PKT_ACK, sizeof(link_ack_t))) {
            continue;  // someone else's traffic
        }
        const link_ack_t *a = (const link_ack_t *)rx;
        if (a->hdr.seq != seq) {
            continue;  // a late acknowledgement for an earlier packet
        }

        e->acked      = true;
        e->rtt_ms     = (uint16_t)((esp_timer_get_time() - t0) / 1000);
        e->ack_rssi   = a->rssi_x10 / 10.0f;
        e->ack_snr    = a->snr_x10 / 10.0f;
        e->local_rssi = info.rssi;
        e->local_snr  = info.snr;

        lock();
        s_ack_count++;
        s_last_rtt_ms     = e->rtt_ms;
        s_last_ack_rssi   = e->ack_rssi;
        s_last_ack_snr    = e->ack_snr;
        s_last_local_rssi = e->local_rssi;
        s_last_local_snr  = e->local_snr;
        unlock();

        // A round trip completed, which is the only real proof a profile works in
        // both directions.
        link_profile_traffic_ok();
        break;
    }

    if (!e->acked) {
        lock();
        s_timeout_count++;
        unlock();
    }
    sx126x_standby();
}

static void send_ping(void)
{
    lock();
    uint8_t len = s_payload_len;
    unlock();

    if (len < sizeof(link_hdr_t)) {
        len = sizeof(link_hdr_t);
    }

    uint8_t buf[LINK_MAX_PAYLOAD];
    memset(buf, 0xA5, sizeof(buf));  // filler out to the requested size

    sender_log_t e;
    exchange(PKT_PING, buf, len, &e);
    log_add(&e);

    if (e.acked) {
        ESP_LOGI(TAG, "#%lu %uB ACK rtt=%ums rx_rssi=%.0f rx_snr=%.1f",
                 (unsigned long)e.seq, e.len, e.rtt_ms, e.ack_rssi, e.ack_snr);
    } else {
        ESP_LOGW(TAG, "#%lu %uB no ACK", (unsigned long)e.seq, e.len);
    }
}

static void send_battery(void)
{
    lock();
    uint8_t len = s_payload_len;
    unlock();

    // Pad out to the configured size like a ping: a longer transmission keys the
    // PA for longer, which is exactly what the max-drain test wants.
    if (len < sizeof(link_batt_t)) {
        len = sizeof(link_batt_t);
    }

    uint8_t buf[LINK_MAX_PAYLOAD];
    memset(buf, 0xA5, sizeof(buf));

    link_batt_t *b = (link_batt_t *)buf;
    b->pin_mv   = (uint16_t)battery_read_pin_mv();
    b->uptime_s = battery_uptime_s();

    sender_log_t e;
    exchange(PKT_BATT, buf, len, &e);
    log_add(&e);

    ESP_LOGI(TAG, "#%lu batt pin=%umV uptime=%us %s",
             (unsigned long)e.seq, b->pin_mv, (unsigned)b->uptime_s,
             e.acked ? "ACK" : "no ACK");
}

// Negotiate a new profile with the receiver, then adopt it here too. The CFG frame
// goes out at the *current* profile so the receiver is guaranteed to understand it.
static void do_cfg_push(void)
{
    lock();
    const sx126x_cfg_t want = s_push_cfg;
    s_push_req = false;
    unlock();

    sx126x_cfg_t old;
    link_get_cfg(&old);

    lock();
    const uint32_t cadence_ms = s_interval_ms;
    unlock();

    // Only the sender knows the interval, so it sizes the provisional window and
    // ships it: both ends must time out together or they diverge. Capped to the
    // 16-bit wire field (~18 hours, far beyond any sane profile).
    uint32_t silence_s = link_profile_silence_s(&want, cadence_ms);
    if (silence_s > UINT16_MAX) {
        silence_s = UINT16_MAX;
    }

    uint8_t buf[sizeof(link_cfgpkt_t)];
    link_cfgpkt_t *p = (link_cfgpkt_t *)buf;
    link_cfg_to_wire(&want, &p->cfg);
    p->rollback_s = (uint16_t)silence_s;

    bool confirmed = false;
    for (int attempt = 1; attempt <= CFG_PUSH_RETRIES && !confirmed; attempt++) {
        char msg[64];
        snprintf(msg, sizeof(msg), "negotiating (attempt %d/%d)", attempt, CFG_PUSH_RETRIES);
        push_state(msg);

        link_put_hdr(buf, PKT_CFG, s_ctrl_seq++);
        if (sx126x_tx(buf, sizeof(buf), tx_timeout_ms(&old, sizeof(buf))) != ESP_OK) {
            continue;
        }

        const uint32_t to = ack_timeout_ms(&old);
        sx126x_rx_start(to);

        uint8_t rx[LINK_MAX_PAYLOAD];
        const int64_t deadline = esp_timer_get_time() + (int64_t)to * 1000;

        while (esp_timer_get_time() < deadline) {
            const int n = sx126x_rx_wait(rx, sizeof(rx), NULL, 100);
            if (n > 0 && link_check(rx, n, PKT_CFGACK, sizeof(link_hdr_t))) {
                confirmed = true;
                break;
            }
        }
        sx126x_standby();
    }

    // Apply here either way. "Push" means both ends end up on this profile, so
    // refusing to switch when the receiver stays quiet would defeat the point -
    // and an unanswered push does not mean the receiver ignored it, only that the
    // CFGACK did not make it back. In that case the receiver has already switched
    // and it is the sender staying put that breaks the link.
    //
    // Applying optimistically is safe because it is provisional: if the receiver
    // really did not hear it, no traffic passes and link_profile_tick() returns
    // both ends to `old` once the silence window expires.
    if (link_cfg_equal(&want, &old)) {
        push_state(confirmed ? "receiver confirmed; already in use here"
                             : "already in use here; receiver did not confirm");
        ESP_LOGI(TAG, "Config push: profile already active locally");
        return;
    }

    link_set_cfg(&want, false);
    link_profile_provisional(&old, silence_s);

    if (confirmed) {
        push_state("applied on both; provisional until traffic passes");
        ESP_LOGW(TAG, "Config push accepted; new profile is provisional");
    } else {
        push_state("applied here; receiver did not confirm");
        ESP_LOGW(TAG, "Config push unanswered; applied anyway, reverting in %us if silent",
                 (unsigned)silence_s);
    }
}

static void sender_task(void *arg)
{
    // Give the receiver a moment to reach its listening loop after a co-power-up.
    vTaskDelay(pdMS_TO_TICKS(1000));

    int64_t last_persist_us = esp_timer_get_time();

    for (;;) {
        link_profile_tick();

        // Apply any profile queued by an HTTP handler here, between exchanges,
        // where it cannot interrupt a transmission.
        if (link_apply_pending()) {
            continue;
        }

        if (s_push_req) {
            do_cfg_push();
            continue;
        }

        lock();
        const bool     run      = s_running;
        const uint32_t interval = s_interval_ms;
        unlock();

        if (!run) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        const int64_t cycle_start = esp_timer_get_time();
        const link_mode_t mode = link_get_mode();

        if (mode == LINK_MODE_BATTERY) {
            send_battery();
            // Checkpoint the discharge clock on the logging interval, not every
            // packet, so flash wear stays proportionate to the test length.
            if (esp_timer_get_time() - last_persist_us >
                (int64_t)CONFIG_LOG_INTERVAL_S * 1000000) {
                battery_uptime_persist();
                last_persist_us = esp_timer_get_time();
            }
        } else {
            send_ping();
        }

        // Pace from the start of the cycle. interval == 0 means back-to-back: the
        // next packet goes out as soon as the acknowledgement is in. The same path
        // covers an interval shorter than the exchange itself, which is easy to hit
        // at SF12.
        //
        // vTaskDelay(1) rather than no delay at all: the loop must yield or it
        // starves the idle task and trips the task watchdog. At 1000 Hz that is 1 ms,
        // against a minimum exchange of tens of milliseconds.
        const int64_t elapsed_ms = (esp_timer_get_time() - cycle_start) / 1000;
        const int32_t remaining  = (int32_t)interval - (int32_t)elapsed_ms;
        vTaskDelay(remaining > 0 ? pdMS_TO_TICKS(remaining) : 1);
    }
}

esp_err_t sender_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    load_settings();  // before the task starts, so the first packet uses them
    if (xTaskCreate(sender_task, "sender", 5120, NULL, 5, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void sender_get_status(sender_status_t *out)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    lock();
    out->running       = s_running;
    out->next_seq      = s_seq;
    out->tx_count      = s_tx_count;
    out->ack_count     = s_ack_count;
    out->timeout_count = s_timeout_count;
    out->interval_ms   = s_interval_ms;
    out->payload_len   = s_payload_len;
    out->last_rtt_ms   = s_last_rtt_ms;
    out->last_ack_rssi = s_last_ack_rssi;
    out->last_ack_snr  = s_last_ack_snr;
    out->last_local_rssi = s_last_local_rssi;
    out->last_local_snr  = s_last_local_snr;
    out->log_seq       = s_log_seq;
    const uint32_t interval = s_interval_ms;
    const uint8_t  len      = s_payload_len;
    unlock();

    out->airtime_ms     = sx126x_airtime_ms(&cfg, len);
    out->ack_airtime_ms = sx126x_airtime_ms(&cfg, sizeof(link_ack_t));

    // Both directions key a PA, so the honest figure counts the acknowledgement
    // too. The cycle can never be shorter than the exchange itself.
    const float cycle = (float)interval > (out->airtime_ms + out->ack_airtime_ms)
                            ? (float)interval
                            : (out->airtime_ms + out->ack_airtime_ms);
    out->duty_cycle = (cycle > 0.0f) ? (out->airtime_ms / cycle) : 0.0f;
}

int sender_copy_log(sender_log_t *out, int max, uint32_t since, uint32_t *next)
{
    lock();
    const uint32_t head = s_log_seq;
    // Entries older than the ring are gone; start from the oldest we still hold.
    uint32_t from = since;
    const uint32_t oldest = (head > SENDER_LOG_SIZE) ? head - SENDER_LOG_SIZE : 0;
    if (from < oldest) {
        from = oldest;
    }

    int n = 0;
    for (uint32_t i = from; i < head && n < max; i++, n++) {
        out[n] = s_log[i % SENDER_LOG_SIZE];
    }
    unlock();

    *next = head;
    return n;
}

void sender_set_running(bool run)
{
    lock();
    s_running = run;
    unlock();
    ESP_LOGW(TAG, "%s", run ? "Started" : "Stopped");
}

esp_err_t sender_set_interval(uint32_t ms)
{
    // 0 is legal and means "no gap": transmit again as soon as the acknowledgement
    // for the previous packet arrives. That is what sustains max drain, because it
    // keeps the LoRa PA keyed for most of the wall clock instead of duty-cycling it.
    if (ms > 600000) {
        return ESP_ERR_INVALID_ARG;
    }
    lock();
    s_interval_ms = ms;
    unlock();
    save_settings();
    return ESP_OK;
}

esp_err_t sender_set_payload_len(int len)
{
    // Takes an int so an out-of-range value from the query string is rejected
    // rather than silently wrapping into a uint8_t.
    if (len < (int)sizeof(link_batt_t) || len > LINK_MAX_PAYLOAD) {
        return ESP_ERR_INVALID_ARG;  // must still fit the largest packet body
    }
    lock();
    s_payload_len = (uint8_t)len;
    unlock();
    save_settings();
    return ESP_OK;
}

void sender_reset_counters(void)
{
    lock();
    s_tx_count = s_ack_count = s_timeout_count = 0;
    s_last_rtt_ms = 0;
    s_last_ack_rssi = s_last_ack_snr = 0.0f;
    s_last_local_rssi = s_last_local_snr = 0.0f;
    unlock();
    ESP_LOGW(TAG, "Counters reset");
}

esp_err_t sender_request_cfg_push(const sx126x_cfg_t *cfg)
{
    lock();
    if (s_push_req) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_push_cfg = *cfg;
    s_push_req = true;
    unlock();
    push_state("queued");
    return ESP_OK;
}

bool sender_cfg_push_busy(void)
{
    lock();
    const bool b = s_push_req;
    unlock();
    return b;
}

void sender_cfg_push_state(char *out, size_t n)
{
    lock();
    strncpy(out, s_push_state, n - 1);
    out[n - 1] = '\0';
    unlock();
}
