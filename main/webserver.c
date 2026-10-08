#include "webserver.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"
#include "link.h"
#include "lwip/sockets.h"
#include "net.h"
#include "sdkconfig.h"
#include "solar.h"

#if CONFIG_ROLE_SENDER
#include "battery.h"
#include "powerload.h"
#include "sender.h"
extern const uint8_t page_start[] asm("_binary_sender_html_start");
extern const uint8_t page_end[]   asm("_binary_sender_html_end");
#else
#include "receiver.h"
#include "storage.h"
extern const uint8_t page_start[] asm("_binary_receiver_html_start");
extern const uint8_t page_end[]   asm("_binary_receiver_html_end");
#endif

static const char *TAG = "web";

#ifndef CONFIG_LED_LOW_BATT_MV
#define CONFIG_LED_LOW_BATT_MV 0  // LED disabled in menuconfig
#endif

// Shared by both /status routes: whether the board has reset, why, and its heap, so
// a long run can tell a brownout or a leak from the dashboard alone.
static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:  return "power-on";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "panic";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:      return "watchdog";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_USB:      return "usb";
        case ESP_RST_EXT:      return "external";
        default:               return "other";
    }
}

static int health_json(char *buf, size_t n)
{
    return snprintf(buf, n,
        "\"boot_s\":%lld,\"reset_reason\":\"%s\",\"heap_free\":%lu,\"heap_min\":%lu,",
        (long long)(esp_timer_get_time() / 1000000), reset_reason_name(esp_reset_reason()),
        (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size());
}

#define QUERY_MAX 256

static void no_store(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *req, const char *buf, int len)
{
    httpd_resp_set_type(req, "application/json");
    no_store(req);
    return httpd_resp_send(req, buf, len);
}

// Read the whole query string once; "" when there is none.
static void get_query(httpd_req_t *req, char *out, size_t n)
{
    if (httpd_req_get_url_query_str(req, out, n) != ESP_OK) {
        out[0] = '\0';
    }
}

static bool q_str(const char *query, const char *key, char *out, size_t n)
{
    return httpd_query_key_value(query, key, out, n) == ESP_OK;
}

static bool q_long(const char *query, const char *key, long *out)
{
    char v[24];
    if (!q_str(query, key, v, sizeof(v))) {
        return false;
    }
    char *end = NULL;
    const long parsed = strtol(v, &end, 10);
    if (end == v) {
        return false;
    }
    *out = parsed;
    return true;
}

static bool q_is(const char *query, const char *key, const char *want)
{
    char v[16];
    return q_str(query, key, v, sizeof(v)) && strcmp(v, want) == 0;
}

// ------------------------------------------------------------ shared routes ----

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    no_store(req);  // always serve the latest page after a reflash
    return httpd_resp_send(req, (const char *)page_start, page_end - page_start);
}

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

// Start from the live profile and override whichever fields the query supplies.
// Returns false with *err set when a value is out of range.
static bool cfg_from_query(const char *query, sx126x_cfg_t *cfg, const char **err)
{
    link_get_cfg(cfg);
    long v;

    if (q_long(query, "sf", &v)) {
        if (v < 5 || v > 12) { *err = "sf must be 5..12"; return false; }
        cfg->sf = (uint8_t)v;
    }
    if (q_long(query, "bw", &v)) {
        bool found = false;
        for (int i = 0; i < LINK_BW_COUNT; i++) {
            if ((long)LINK_BW_HZ[i] == v) { cfg->bw = LINK_BW_CODE[i]; found = true; break; }
        }
        if (!found) { *err = "unsupported bandwidth"; return false; }
    }
    if (q_long(query, "cr", &v)) {
        if (v < 5 || v > 8) { *err = "cr must be 5..8"; return false; }
        cfg->cr = (uint8_t)v;
    }
    if (q_long(query, "pre", &v)) {
        if (v < 1 || v > 65535) { *err = "preamble must be 1..65535"; return false; }
        cfg->preamble = (uint16_t)v;
    }
    if (q_long(query, "pwr", &v)) {
        if (v < -9 || v > 22) { *err = "power must be -9..22 dBm"; return false; }
        cfg->tx_dbm = (int8_t)v;
    }
    if (q_long(query, "freq", &v)) {
        if (v < (long)LINK_FREQ_MIN_HZ || v > (long)LINK_FREQ_MAX_HZ) {
            *err = "frequency must be 150000000..960000000 Hz";
            return false;
        }
        cfg->freq_hz = (uint32_t)v;
    }
    if (q_long(query, "crc", &v)) {
        cfg->crc_on = (v != 0);
    }
    return true;
}

// GET /radio                 -> current profile
// GET /radio?sf=10&bw=125000 -> queue a local change, persisted once applied
//
// Deliberately local-only, and applied by the radio task rather than here. Changing
// the profile this way breaks the link until the other end is changed to match;
// /pushcfg is the coordinated version. The response reports the profile still in
// force, so the value only updates once the radio task has actually switched.
static esp_err_t radio_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));

    if (query[0] != '\0') {
        sx126x_cfg_t cfg;
        const char *err = NULL;
        if (!cfg_from_query(query, &cfg, &err)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
        }
        // Queued for the radio task, never applied here: sx126x_tx() drops the
        // driver lock while waiting for TxDone, so reconfiguring from this task
        // would abort whatever packet is in the air.
        if (link_request_apply(&cfg, true) != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "queue failed");
        }
#if CONFIG_ROLE_SENDER
        sender_wake();  // apply now, not after the current packet interval
#endif
    }

    sx126x_cfg_t cur;
    link_get_cfg(&cur);
#if CONFIG_ROLE_SENDER
    sender_status_t st;
    sender_get_status(&st);
    const uint8_t plen = st.payload_len;
#else
    const uint8_t plen = LINK_MAX_PAYLOAD;
#endif

    char buf[512];
    int n = snprintf(buf, sizeof(buf), "{");
    n += link_cfg_json(buf + n, sizeof(buf) - n, &cur, plen);
    n += snprintf(buf + n, sizeof(buf) - n, "}");
    return send_json(req, buf, n);
}

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));  // let the HTTP response flush first
    esp_restart();
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    if (!q_is(query, "confirm", "1")) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirm=1");
    }
    ESP_LOGW(TAG, "Reboot requested from the dashboard");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return send_json(req, "{\"rebooting\":true}", HTTPD_RESP_USE_STRLEN);
}

// GET /led            -> what the status LED is showing
// GET /led?level=24   -> status brightness 0..255, saved (0 = indicator off)
// GET /led?test=1     -> red, green, blue, white at full brightness, to check it
static esp_err_t led_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long v;
    if (q_long(query, "level", &v)) {
        const esp_err_t err = led_set_level((int)v);
        if (err == ESP_ERR_INVALID_ARG) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "level must be 0..255");
        }
        if (err != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "level applied but could not be saved");
        }
    }
    if (q_is(query, "test", "1")) {
        led_self_test();
    }
    if (q_is(query, "trace", "1")) {
        led_trace_t t[LED_TRACE_LEN];
        const int count = led_trace(t, LED_TRACE_LEN);
        static char tb[LED_TRACE_LEN * 56 + 64];
        int k = snprintf(tb, sizeof(tb), "{\"now_ms\":%lld,\"trace\":[",
                         (long long)(esp_timer_get_time() / 1000));
        // Sized for the longest pattern name; the guard only matters if one is added
        // that is longer still, when the newest entries are dropped, not overrun.
        for (int i = 0; i < count && k < (int)sizeof(tb) - 64; i++) {
            k += snprintf(tb + k, sizeof(tb) - k, "%s[%lld,\"#%06lx\",\"%s\"]", i ? "," : "",
                          (long long)t[i].at_ms, (unsigned long)t[i].rgb, t[i].pattern);
        }
        k += snprintf(tb + k, sizeof(tb) - k, "]}");
        return send_json(req, tb, k);
    }
    char buf[96];
    const int n = snprintf(buf, sizeof(buf), "{\"level\":%u,\"pattern\":\"%s\",\"rgb\":\"#%06lx\"}",
                           led_level(), led_pattern(), (unsigned long)led_rgb());
    return send_json(req, buf, n);
}

// Accumulates a chunked response into ~1 KB writes. Sending one chunk per log
// entry meant up to 128 tiny TCP writes per poll, which is what makes a congested
// link (a phone on a saturated SoftAP, or max-drain mode blasting broadcasts) time
// out mid-response with EAGAIN.
typedef struct {
    httpd_req_t *req;
    int          len;
    char         buf[1024];
} chunker_t;

static esp_err_t chunk_flush(chunker_t *c)
{
    if (c->len == 0) {
        return ESP_OK;
    }
    const esp_err_t err = httpd_resp_send_chunk(c->req, c->buf, c->len);
    c->len = 0;
    return err;
}

static esp_err_t chunk_printf(chunker_t *c, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(c->buf + c->len, sizeof(c->buf) - c->len, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return ESP_FAIL;
    }

    // Truncated: flush what already fit and format again into an empty buffer.
    if (n >= (int)(sizeof(c->buf) - c->len)) {
        const esp_err_t err = chunk_flush(c);
        if (err != ESP_OK) {
            return err;
        }
        va_start(ap, fmt);
        n = vsnprintf(c->buf, sizeof(c->buf), fmt, ap);
        va_end(ap);
        if (n < 0 || n >= (int)sizeof(c->buf)) {
            return ESP_FAIL;  // single entry larger than the buffer: not possible here
        }
    }
    c->len += n;
    return ESP_OK;
}

// ------------------------------------------------------------ sender routes ----
#if CONFIG_ROLE_SENDER

static esp_err_t status_handler(httpd_req_t *req)
{
    sender_status_t st;
    sender_get_status(&st);
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    const uint32_t up     = battery_uptime_s();
    const int      pin_mv = battery_read_pin_mv();

    char push[96], remote[80];
    sender_cfg_push_state(push, sizeof(push));
    sender_remote_command_state(remote, sizeof(remote));

    // Static, as on the receiver: the HTTP server runs one handler at a time, and
    // this is too big to share the handler's stack with the formatting.
    static char buf[2048];
    int n = snprintf(buf, sizeof(buf),
        "{\"role\":\"sender\",\"mode\":\"%s\",\"ip\":\"%s\",\"ssid\":\"%s\","
        "\"wifi\":%s,\"running\":%s,\"interval_ms\":%lu,"
        "\"tx\":%lu,\"ack\":%lu,\"timeout\":%lu,\"ack_rate\":%.1f,"
        "\"next_seq\":%lu,\"last_rtt_ms\":%lu,\"consec_miss\":%lu,"
        "\"ack_rssi\":%.1f,\"ack_snr\":%.1f,\"local_rssi\":%.1f,\"local_snr\":%.1f,"
        "\"ack_airtime_ms\":%.2f,\"duty_cycle\":%.4f,\"log_seq\":%lu,"
        "\"pin_mv\":%d,\"batt_mv\":%d,\"ratio\":%.3f,\"uptime_s\":%lu,"
        "\"load\":%s,\"load_arming_s\":%lu,\"led_low_mv\":%d,\"push_busy\":%s,\"push_state\":\"%s\","
        "\"remote_cmd\":\"%s\",\"charge\":\"%s\",\"peer_charge\":\"%s\","
        "\"apply_pending\":%s,\"rollback_pending\":%s,\"rollback_s\":%lu,"
        "\"led\":\"%s\",\"led_rgb\":\"#%06lx\",\"led_level\":%u,",
        link_mode_str(link_get_mode()), net_ip_str(), net_ssid(),
        net_wifi_is_on() ? "true" : "false",
        st.running ? "true" : "false", (unsigned long)st.interval_ms,
        (unsigned long)st.tx_count, (unsigned long)st.ack_count,
        (unsigned long)st.timeout_count,
        st.tx_count ? (100.0f * (float)st.ack_count / (float)st.tx_count) : 0.0f,
        (unsigned long)st.next_seq, (unsigned long)st.last_rtt_ms,
        (unsigned long)st.consec_miss,
        st.last_ack_rssi, st.last_ack_snr, st.last_local_rssi, st.last_local_snr,
        st.ack_airtime_ms, st.duty_cycle, (unsigned long)st.log_seq,
        pin_mv, battery_mv_from_pin(pin_mv), battery_ratio_x1000() / 1000.0, (unsigned long)up,
        powerload_is_on() ? "true" : "false",
        (unsigned long)powerload_arming_in_s(), CONFIG_LED_LOW_BATT_MV,
        sender_cfg_push_busy() ? "true" : "false", push, remote,
        charge_state_str(charge_state()), charge_state_str((charge_state_t)st.peer_charge),
        link_apply_queued() ? "true" : "false",
        link_profile_is_provisional() ? "true" : "false",
        (unsigned long)link_profile_revert_in_s(),
        led_pattern(), (unsigned long)led_rgb(), led_level());

    n += health_json(buf + n, sizeof(buf) - n);
    n += link_cfg_json(buf + n, sizeof(buf) - n, &cfg, st.payload_len);
    n += snprintf(buf + n, sizeof(buf) - n, "}");
    return send_json(req, buf, n);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long since = 0;
    q_long(query, "since", &since);

    // The HTTP server runs handlers on a single task, so one shared buffer is safe
    // and keeps 5 KB off the stack.
    static sender_log_t entries[SENDER_LOG_SIZE];
    uint32_t next = 0;
    const int count = sender_copy_log(entries, SENDER_LOG_SIZE, (uint32_t)since, &next);

    httpd_resp_set_type(req, "application/json");
    no_store(req);

    static chunker_t c;  // 1 KB; static for the same single-task reason as `entries`
    c.req = req;
    c.len = 0;

    esp_err_t err = chunk_printf(&c, "{\"next\":%lu,\"entries\":[", (unsigned long)next);
    for (int i = 0; i < count && err == ESP_OK; i++) {
        const sender_log_t *e = &entries[i];
        err = chunk_printf(&c,
            "%s{\"seq\":%lu,\"len\":%u,\"type\":%u,\"acked\":%s,\"rtt_ms\":%lu,"
            "\"airtime_ms\":%.2f,\"ack_rssi\":%.1f,\"ack_snr\":%.1f,"
            "\"local_rssi\":%.1f,\"local_snr\":%.1f,\"uptime_s\":%lu}",
            i ? "," : "", (unsigned long)e->seq, e->len, e->type,
            e->acked ? "true" : "false", (unsigned long)e->rtt_ms, e->airtime_ms,
            e->ack_rssi, e->ack_snr, e->local_rssi, e->local_snr,
            (unsigned long)e->at_uptime_s);
    }
    if (err == ESP_OK) {
        err = chunk_printf(&c, "]}");
    }
    if (err == ESP_OK) {
        err = chunk_flush(&c);
    }
    if (err != ESP_OK) {
        return ESP_FAIL;  // client went away mid-response; httpd closes the socket
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

// GET /control?run=1&interval_ms=5000&size=255
// Every value is checked before any is applied: a bad size used to be rejected
// only after the interval in the same request had already been saved.
static esp_err_t control_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long run = 0, interval = 0, size = 0;
    const bool has_run      = q_long(query, "run", &run);
    const bool has_interval = q_long(query, "interval_ms", &interval);
    const bool has_size     = q_long(query, "size", &size);

    if (has_interval && (interval < 0 || interval > 600000)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "interval must be 0..600000 ms (0 = back-to-back)");
    }
    if (has_size && (size < (long)sizeof(link_batt_t) || size > LINK_MAX_PAYLOAD)) {
        char msg[40];
        snprintf(msg, sizeof(msg), "size must be %u..%u bytes",
                 (unsigned)sizeof(link_batt_t), (unsigned)LINK_MAX_PAYLOAD);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
    }

    if (has_run) {
        sender_set_running(run != 0);
    }
    if (has_interval) {
        sender_set_interval((uint32_t)interval);
    }
    if (has_size) {
        sender_set_payload_len((int)size);
    }
    if (q_is(query, "reset", "1")) {
        sender_reset_counters();
    }

    sender_status_t st;
    sender_get_status(&st);
    char buf[128];
    const int n = snprintf(buf, sizeof(buf),
        "{\"running\":%s,\"interval_ms\":%lu,\"size\":%u}",
        st.running ? "true" : "false", (unsigned long)st.interval_ms, st.payload_len);
    return send_json(req, buf, n);
}

// GET /pushcfg?confirm=1&sf=10&bw=125000...
// Negotiates the profile with the receiver instead of applying it unilaterally.
static esp_err_t pushcfg_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    if (!q_is(query, "confirm", "1")) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirm=1");
    }

    sx126x_cfg_t cfg;
    const char *err = NULL;
    if (!cfg_from_query(query, &cfg, &err)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
    }
    if (sender_request_cfg_push(&cfg) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a push is already in flight");
    }
    return send_json(req, "{\"queued\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t mode_handler(httpd_req_t *req)
{
    char query[QUERY_MAX], v[16];
    get_query(req, query, sizeof(query));

    if (q_str(query, "m", v, sizeof(v))) {
        const link_mode_t m = (strcmp(v, "battery") == 0) ? LINK_MODE_BATTERY : LINK_MODE_RANGE;
        link_set_mode(m);
        // Deliberately does NOT touch the max-drain load. That is a persisted
        // setting the operator owns; having a mode switch silently flip it meant a
        // manual choice could be undone without asking.
    }

    char buf[48];
    const int n = snprintf(buf, sizeof(buf), "{\"mode\":\"%s\",\"load\":%s}",
                           link_mode_str(link_get_mode()),
                           powerload_is_on() ? "true" : "false");
    return send_json(req, buf, n);
}

static esp_err_t load_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long v;
    if (q_long(query, "on", &v)) {
        powerload_set(v != 0);
    }
    char buf[32];
    const int n = snprintf(buf, sizeof(buf), "{\"load\":%s}",
                           powerload_is_on() ? "true" : "false");
    return send_json(req, buf, n);
}

// GET /uptime           -> current discharge clock
// GET /uptime?reset=1   -> start a fresh discharge cycle at zero
static esp_err_t uptime_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));

    if (q_is(query, "reset", "1")) {
        sender_reset_discharge_clock();
    }

    char buf[48];
    const int n = snprintf(buf, sizeof(buf), "{\"uptime_s\":%lu}",
                           (unsigned long)battery_uptime_s());
    return send_json(req, buf, n);
}

// GET /battery                    -> sense pin, divider ratio, battery mV
// GET /battery?actual_mv=4160     -> calibrate the ratio from a multimeter reading
// GET /battery?ratio_x1000=4765   -> set the ratio directly
// GET /battery?reset=1            -> back to the build default
static esp_err_t battery_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long v;
    char msg[96];
    esp_err_t err = ESP_OK;

    if (q_long(query, "actual_mv", &v)) {
        if (v < 500 || v > 30000) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "actual_mv must be 500..30000");
        }
        int pin = 0;
        err = battery_calibrate((uint32_t)v, &pin);
        if (err == ESP_ERR_INVALID_STATE) {
            snprintf(msg, sizeof(msg), "the sense pin reads %d mV: no battery to calibrate against", pin);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        }
        if (err == ESP_ERR_INVALID_ARG) {
            snprintf(msg, sizeof(msg), "%ld mV for %d mV at the pin is a ratio outside 1..20: "
                     "check the reading", v, pin);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        }
    } else if (q_long(query, "ratio_x1000", &v)) {
        err = battery_set_ratio_x1000(v < 0 ? 0 : (uint32_t)v);
        if (err == ESP_ERR_INVALID_ARG) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ratio_x1000 must be 1000..20000");
        }
    } else if (q_is(query, "reset", "1")) {
        battery_ratio_reset();
    }
    if (err != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "ratio applied but could not be saved");
    }

    const int pin_mv = battery_read_pin_mv();
    char buf[112];
    const int n = snprintf(buf, sizeof(buf),
        "{\"pin_mv\":%d,\"batt_mv\":%d,\"ratio\":%.3f,\"default_ratio\":%.3f}",
        pin_mv, battery_mv_from_pin(pin_mv), battery_ratio_x1000() / 1000.0,
        battery_ratio_default_x1000() / 1000.0);
    return send_json(req, buf, n);
}

static esp_err_t wifi_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    if (!q_is(query, "on", "0")) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "only on=0 is supported");
    }
    // Reply before pulling the rug out from under the connection.
    send_json(req, "{\"wifi\":false}", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(300));
    powerload_set(false);
    net_wifi_off();
    return ESP_OK;
}

#else  // ---------------------------------------------------- receiver routes ----

static esp_err_t status_handler(httpd_req_t *req)
{
    receiver_status_t st;
    receiver_get_status(&st);
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    // Static for the same single-task reason as the log handlers: at ~1.6 KB worst
    // case it is too big for the handler's stack alongside the formatting.
    static char buf[2048];
    int n = snprintf(buf, sizeof(buf),
        "{\"role\":\"receiver\",\"mode\":\"%s\",\"ip\":\"%s\",\"ssid\":\"%s\","
        "\"rx\":%lu,\"lost\":%lu,\"dup\":%lu,\"out_of_order\":%lu,"
        "\"crc_err\":%u,\"hdr_err\":%u,\"pdr\":%.1f,"
        "\"have_packet\":%s,\"last_seq\":%lu,\"since_last_s\":%lu,"
        "\"rssi\":%.1f,\"snr\":%.1f,\"signal_rssi\":%.1f,\"noise_floor\":%.1f,"
        "\"snr_margin\":%.1f,\"link_budget\":%.1f,\"fade_margin\":%.1f,"
        "\"log_seq\":%lu,\"have_batt\":%s,\"pin_mv\":%u,\"batt_mv\":%lu,\"ratio\":%.3f,"
        "\"batt_uptime_s\":%lu,"
        "\"charge\":\"%s\",\"peer_charge\":\"%s\","
        "\"apply_pending\":%s,\"rollback_pending\":%s,\"rollback_s\":%lu,"
        "\"led\":\"%s\",\"led_rgb\":\"#%06lx\",\"led_level\":%u,",
        link_mode_str(link_get_mode()), net_ip_str(), net_ssid(),
        (unsigned long)st.rx_count, (unsigned long)st.lost, (unsigned long)st.dup,
        (unsigned long)st.out_of_order, st.crc_err, st.hdr_err, st.pdr,
        st.have_packet ? "true" : "false", (unsigned long)st.last_seq,
        (unsigned long)st.since_last_s,
        st.last_rssi, st.last_snr, st.last_signal_rssi, st.noise_floor,
        st.snr_margin, st.link_budget, st.fade_margin,
        (unsigned long)st.log_seq, st.have_batt ? "true" : "false",
        st.batt_pin_mv, (unsigned long)st.batt_mv, st.batt_ratio_x1000 / 1000.0,
        (unsigned long)st.batt_uptime_s,
        charge_state_str(charge_state()), charge_state_str((charge_state_t)st.peer_charge),
        link_apply_queued() ? "true" : "false",
        link_profile_is_provisional() ? "true" : "false",
        (unsigned long)link_profile_revert_in_s(),
        led_pattern(), (unsigned long)led_rgb(), led_level());

    n += snprintf(buf + n, sizeof(buf) - n,
        "\"cmd_queued\":%d,\"cmd_waiting\":\"%s\",\"cmd_state\":\"%s\","
        "\"contact_seen\":%s,\"contact_age_s\":%lu,\"sender_stopped\":%s,\"link_lost\":%s,"
        "\"sender_known\":%s,\"sender_age_s\":%lu,\"sender_running\":%s,"
        "\"sender_mode\":\"%s\",\"sender_interval_ms\":%lu,\"sender_payload\":%u,"
        "\"sender_load\":%s,\"sender_wifi\":%s,\"sender_ratio\":%.3f,",
        st.cmd_queued, st.cmd_waiting, st.cmd_state,
        st.contact_seen ? "true" : "false", (unsigned long)st.contact_age_s,
        st.sender_stopped ? "true" : "false", st.link_lost ? "true" : "false",
        st.sender_known ? "true" : "false", (unsigned long)st.sender_age_s,
        st.sender_running ? "true" : "false", link_mode_str((link_mode_t)st.sender_mode),
        (unsigned long)st.sender_interval_ms, st.sender_payload,
        st.sender_load ? "true" : "false", st.sender_wifi ? "true" : "false",
        st.sender_ratio_x1000 / 1000.0);

    n += health_json(buf + n, sizeof(buf) - n);
    n += link_cfg_json(buf + n, sizeof(buf) - n, &cfg, LINK_MAX_PAYLOAD);
    n += snprintf(buf + n, sizeof(buf) - n, "}");
    return send_json(req, buf, n);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long since = 0;
    q_long(query, "since", &since);

    static receiver_log_t entries[RECEIVER_LOG_SIZE];
    uint32_t next = 0;
    const int count = receiver_copy_log(entries, RECEIVER_LOG_SIZE, (uint32_t)since, &next);

    httpd_resp_set_type(req, "application/json");
    no_store(req);

    static chunker_t c;
    c.req = req;
    c.len = 0;

    esp_err_t err = chunk_printf(&c, "{\"next\":%lu,\"entries\":[", (unsigned long)next);
    for (int i = 0; i < count && err == ESP_OK; i++) {
        const receiver_log_t *e = &entries[i];
        err = chunk_printf(&c,
            "%s{\"seq\":%lu,\"len\":%u,\"type\":%u,\"rssi\":%.1f,\"signal_rssi\":%.1f,"
            "\"snr\":%.1f,\"snr_margin\":%.1f,\"fade_margin\":%.1f,\"path_loss\":%.1f,"
            "\"freq_err\":%.0f,\"noise_floor\":%.1f,\"gap\":%u,\"pdr\":%.1f,\"lost\":%lu,"
            "\"dup\":%lu,\"out_of_order\":%lu,\"crc_err\":%u,\"hdr_err\":%u,"
            "\"batt_mv\":%u,\"charge\":%u,\"uptime_s\":%lu}",
            i ? "," : "", (unsigned long)e->seq, e->len, e->type, e->rssi, e->signal_rssi,
            e->snr, e->snr_margin, e->fade_margin, e->path_loss, e->freq_err_hz,
            e->noise_floor, e->gap, e->pdr, (unsigned long)e->lost, (unsigned long)e->dup,
            (unsigned long)e->out_of_order, e->crc_err, e->hdr_err, e->batt_mv, e->charge,
            (unsigned long)e->at_uptime_s);
    }
    if (err == ESP_OK) {
        err = chunk_printf(&c, "]}");
    }
    if (err == ESP_OK) {
        err = chunk_flush(&c);
    }
    if (err != ESP_OK) {
        return ESP_FAIL;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t csv_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"battery_log.csv\"");
    no_store(req);  // a cached copy would be missing every sample since

    // The lock is taken per read, never across a send. The receiver task appends
    // under it from inside its radio loop, before acknowledging the packet, so
    // holding it for a whole transfer - seconds for a long log or a dozing phone,
    // and the dashboard refetches this file on every new sample - held the
    // acknowledgements past the sender's window.
    storage_lock();
    FILE *f = fopen(STORAGE_CSV_PATH, "r");
    storage_unlock();
    if (f == NULL) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no log file");
    }

    char chunk[512];
    for (;;) {
        storage_lock();
        const size_t r = fread(chunk, 1, sizeof(chunk), f);
        storage_unlock();
        if (r == 0) {
            break;
        }
        if (httpd_resp_send_chunk(req, chunk, r) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t clear_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    if (!q_is(query, "confirm", "1")) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirm=1");
    }
    if (storage_clear() != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "clear failed");
    }
    return send_json(req, "{\"cleared\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t reset_handler(httpd_req_t *req)
{
    receiver_reset_counters();
    return send_json(req, "{\"reset\":true}", HTTPD_RESP_USE_STRLEN);
}

// The sender's commands, by the name /sendercmd takes. A command with an argument
// reads it from `key`, checked against the same range the sender's own route uses,
// so a bad value is refused here rather than after a trip over the air.
typedef struct {
    const char *name;
    uint8_t     op;       // link_cmd_op_t
    const char *key;      // NULL: no argument
    long        min, max;
    bool        confirm;  // destructive, so confirm=1 as on the sender's own routes
} sender_cmd_t;

static const sender_cmd_t SENDER_CMDS[] = {
    { "status",      LINK_CMD_STATUS,         NULL,        0,   0,                   false },
    { "run",         LINK_CMD_RUN,            "on",        0,   1,                   false },
    { "interval",    LINK_CMD_INTERVAL,       "ms",        0,   600000,              false },
    { "payload",     LINK_CMD_PAYLOAD,        "size",      sizeof(link_batt_t), LINK_MAX_PAYLOAD, false },
    { "reset",       LINK_CMD_RESET_COUNTERS, NULL,        0,   0,                   false },
    { "load",        LINK_CMD_LOAD,           "on",        0,   1,                   false },
    { "clock_reset", LINK_CMD_RESET_CLOCK,    NULL,        0,   0,                   false },
    { "calibrate",   LINK_CMD_CALIBRATE,      "actual_mv", 500, 30000,               false },
    { "ratio_reset", LINK_CMD_RATIO_RESET,    NULL,        0,   0,                   false },
    { "wifi_off",    LINK_CMD_WIFI_OFF,       NULL,        0,   0,                   true },
    { "reboot",      LINK_CMD_REBOOT,         NULL,        0,   0,                   true },
};

// GET /sendercmd?op=interval&ms=2000         -> queue a command for the sender
// GET /sendercmd?op=mode&m=battery           -> battery or range test
// GET /sendercmd?op=push&confirm=1&sf=10...  -> a profile for both, as /radio takes it
// GET /sendercmd?op=cancel                   -> drop everything still waiting
//
// Queued, not sent: commands ride on this board's acknowledgements, so each goes out
// with the sender's next packet, or its next check-in while stopped.
static esp_err_t sendercmd_handler(httpd_req_t *req)
{
    char query[QUERY_MAX], name[16];
    get_query(req, query, sizeof(query));
    if (!q_str(query, "op", name, sizeof(name))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing op");
    }
    if (strcmp(name, "cancel") == 0) {
        receiver_cancel_commands();
        return send_json(req, "{\"cancelled\":true}", HTTPD_RESP_USE_STRLEN);
    }

    uint8_t      op  = 0;
    uint32_t     arg = 0;
    sx126x_cfg_t cfg;
    bool         has_cfg = false;
    char         msg[64];

    if (strcmp(name, "mode") == 0) {
        op = LINK_CMD_MODE;
        if (q_is(query, "m", "battery")) {
            arg = LINK_MODE_BATTERY;
        } else if (q_is(query, "m", "range")) {
            arg = LINK_MODE_RANGE;
        } else {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "m must be range or battery");
        }
    } else if (strcmp(name, "push") == 0) {
        if (!q_is(query, "confirm", "1")) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirm=1");
        }
        const char *err = NULL;
        if (!cfg_from_query(query, &cfg, &err)) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err);
        }
        op      = LINK_CMD_PUSH_CFG;
        has_cfg = true;
    } else {
        const sender_cmd_t *c = NULL;
        for (size_t i = 0; i < sizeof(SENDER_CMDS) / sizeof(SENDER_CMDS[0]); i++) {
            if (strcmp(name, SENDER_CMDS[i].name) == 0) {
                c = &SENDER_CMDS[i];
                break;
            }
        }
        if (c == NULL) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown op");
        }
        if (c->confirm && !q_is(query, "confirm", "1")) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing confirm=1");
        }
        if (c->key != NULL) {
            long v;
            if (!q_long(query, c->key, &v) || v < c->min || v > c->max) {
                snprintf(msg, sizeof(msg), "%s must be %ld..%ld", c->key, c->min, c->max);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
            }
            arg = (uint32_t)v;
        }
        op = c->op;
    }

    if (receiver_queue_command(op, arg, has_cfg ? &cfg : NULL) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "the queue is full: wait for the sender, or cancel");
    }
    return send_json(req, "{\"queued\":true}", HTTPD_RESP_USE_STRLEN);
}

#endif

// ------------------------------------------------------------------ start ----

// Every response goes out as several small writes - httpd sends the status line and
// each header field separately - and with Nagle on, the body then waits for the
// client to ACK them. lwIP delays that ACK by up to 250 ms, so the serial console's
// loopback requests (see console.c) took a quarter of a second each. Loopback only:
// a phone ACKs promptly, and over WiFi Nagle's coalescing is what keeps those
// header writes from going out as a dozen separate frames into a busy channel.
static bool peer_is_loopback(int sockfd)
{
    struct sockaddr_storage peer;
    socklen_t len = sizeof(peer);
    if (getpeername(sockfd, (struct sockaddr *)&peer, &len) != 0) {
        return false;
    }
    if (peer.ss_family == AF_INET) {
        return ((struct sockaddr_in *)&peer)->sin_addr.s_addr == htonl(INADDR_LOOPBACK);
    }
#if CONFIG_LWIP_IPV6
    // httpd listens on IPv6 when lwIP has it, so an IPv4 client shows up mapped:
    // ::ffff:127.0.0.1.
    if (peer.ss_family == AF_INET6) {
        const struct in6_addr *a = &((struct sockaddr_in6 *)&peer)->sin6_addr;
        static const uint8_t mapped[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 127, 0, 0, 1 };
        static const uint8_t v6[16]     = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
        return memcmp(a, mapped, 16) == 0 || memcmp(a, v6, 16) == 0;
    }
#endif
    return false;
}

static esp_err_t on_open(httpd_handle_t hd, int sockfd)
{
    if (peer_is_loopback(sockfd)) {
        const int one = 1;
        setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
    return ESP_OK;
}

esp_err_t webserver_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 16;  // the sender registers 14; one past the limit fails silently
    config.stack_size       = 6144;  // the log handlers format a lot of JSON
    // The default 5 s is tight when max-drain mode is saturating the WiFi TX path
    // or a phone client is dozing; both show up as EAGAIN on send.
    config.send_wait_timeout = 10;
    config.recv_wait_timeout = 10;
    config.open_fn           = on_open;

    httpd_handle_t server = NULL;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server (%s)", esp_err_to_name(ret));
        return ret;
    }

    const httpd_uri_t routes[] = {
        { .uri = "/",            .method = HTTP_GET, .handler = root_handler },
        { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler },
        { .uri = "/radio",       .method = HTTP_GET, .handler = radio_handler },
        { .uri = "/reboot",      .method = HTTP_GET, .handler = reboot_handler },
        { .uri = "/led",         .method = HTTP_GET, .handler = led_handler },
        { .uri = "/status",      .method = HTTP_GET, .handler = status_handler },
        { .uri = "/log",         .method = HTTP_GET, .handler = log_handler },
#if CONFIG_ROLE_SENDER
        { .uri = "/control",     .method = HTTP_GET, .handler = control_handler },
        { .uri = "/pushcfg",     .method = HTTP_GET, .handler = pushcfg_handler },
        { .uri = "/mode",        .method = HTTP_GET, .handler = mode_handler },
        { .uri = "/load",        .method = HTTP_GET, .handler = load_handler },
        { .uri = "/uptime",      .method = HTTP_GET, .handler = uptime_handler },
        { .uri = "/wifi",        .method = HTTP_GET, .handler = wifi_handler },
        { .uri = "/battery",     .method = HTTP_GET, .handler = battery_handler },
#else
        { .uri = "/data.csv",    .method = HTTP_GET, .handler = csv_handler },
        { .uri = "/clear",       .method = HTTP_GET, .handler = clear_handler },
        { .uri = "/reset",       .method = HTTP_GET, .handler = reset_handler },
        { .uri = "/sendercmd",   .method = HTTP_GET, .handler = sendercmd_handler },
#endif
    };
    for (int i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
