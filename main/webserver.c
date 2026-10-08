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
#include "link.h"
#include "net.h"
#include "sdkconfig.h"

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
        if (v < 6 || v > 65535) { *err = "preamble must be 6..65535"; return false; }
        cfg->preamble = (uint16_t)v;
    }
    if (q_long(query, "pwr", &v)) {
        if (v < -9 || v > 22) { *err = "power must be -9..22 dBm"; return false; }
        cfg->tx_dbm = (int8_t)v;
    }
    if (q_long(query, "freq", &v)) {
        if (v < (long)LINK_FREQ_MIN_HZ || v > (long)LINK_FREQ_MAX_HZ) {
            *err = "frequency outside the radio module's band";
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

    char push[96];
    sender_cfg_push_state(push, sizeof(push));

    char buf[1200];
    int n = snprintf(buf, sizeof(buf),
        "{\"role\":\"sender\",\"mode\":\"%s\",\"ip\":\"%s\",\"ssid\":\"%s\","
        "\"wifi\":%s,\"running\":%s,\"interval_ms\":%lu,"
        "\"tx\":%lu,\"ack\":%lu,\"timeout\":%lu,\"ack_rate\":%.1f,"
        "\"next_seq\":%lu,\"last_rtt_ms\":%lu,"
        "\"ack_rssi\":%.1f,\"ack_snr\":%.1f,\"local_rssi\":%.1f,\"local_snr\":%.1f,"
        "\"ack_airtime_ms\":%.2f,\"duty_cycle\":%.4f,\"log_seq\":%lu,"
        "\"pin_mv\":%d,\"uptime_s\":%lu,"
        "\"load\":%s,\"load_arming_s\":%lu,\"push_busy\":%s,\"push_state\":\"%s\","
        "\"rollback_pending\":%s,\"rollback_s\":%lu,",
        link_mode_str(link_get_mode()), net_ip_str(), net_ssid(),
        net_wifi_is_on() ? "true" : "false",
        st.running ? "true" : "false", (unsigned long)st.interval_ms,
        (unsigned long)st.tx_count, (unsigned long)st.ack_count,
        (unsigned long)st.timeout_count,
        st.tx_count ? (100.0f * (float)st.ack_count / (float)st.tx_count) : 0.0f,
        (unsigned long)st.next_seq, (unsigned long)st.last_rtt_ms,
        st.last_ack_rssi, st.last_ack_snr, st.last_local_rssi, st.last_local_snr,
        st.ack_airtime_ms, st.duty_cycle, (unsigned long)st.log_seq,
        pin_mv, (unsigned long)up,
        powerload_is_on() ? "true" : "false",
        (unsigned long)powerload_arming_in_s(),
        sender_cfg_push_busy() ? "true" : "false", push,
        link_profile_is_provisional() ? "true" : "false",
        (unsigned long)link_profile_revert_in_s());

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
            "%s{\"seq\":%lu,\"len\":%u,\"type\":%u,\"acked\":%s,\"rtt_ms\":%u,"
            "\"airtime_ms\":%.2f,\"ack_rssi\":%.1f,\"ack_snr\":%.1f,"
            "\"local_rssi\":%.1f,\"local_snr\":%.1f,\"uptime_s\":%lu}",
            i ? "," : "", (unsigned long)e->seq, e->len, e->type,
            e->acked ? "true" : "false", e->rtt_ms, e->airtime_ms,
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
static esp_err_t control_handler(httpd_req_t *req)
{
    char query[QUERY_MAX];
    get_query(req, query, sizeof(query));
    long v;

    if (q_long(query, "run", &v)) {
        sender_set_running(v != 0);
    }
    if (q_long(query, "interval_ms", &v)) {
        if (sender_set_interval((uint32_t)v) != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "interval must be 0..600000 ms (0 = back-to-back)");
        }
    }
    if (q_long(query, "size", &v)) {
        if (sender_set_payload_len((int)v) != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "size must be 16..255 bytes");
        }
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
        battery_uptime_reset();
    }

    char buf[48];
    const int n = snprintf(buf, sizeof(buf), "{\"uptime_s\":%lu}",
                           (unsigned long)battery_uptime_s());
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

    char buf[1200];
    int n = snprintf(buf, sizeof(buf),
        "{\"role\":\"receiver\",\"mode\":\"%s\",\"ip\":\"%s\",\"ssid\":\"%s\","
        "\"rx\":%lu,\"lost\":%lu,\"dup\":%lu,\"out_of_order\":%lu,"
        "\"crc_err\":%u,\"hdr_err\":%u,\"pdr\":%.1f,"
        "\"have_packet\":%s,\"last_seq\":%lu,\"since_last_s\":%lu,"
        "\"rssi\":%.1f,\"snr\":%.1f,\"signal_rssi\":%.1f,\"noise_floor\":%.1f,"
        "\"snr_margin\":%.1f,\"link_budget\":%.1f,\"fade_margin\":%.1f,"
        "\"log_seq\":%lu,\"have_batt\":%s,\"pin_mv\":%u,"
        "\"batt_uptime_s\":%lu,"
        "\"rollback_pending\":%s,\"rollback_s\":%lu,",
        link_mode_str(link_get_mode()), net_ip_str(), net_ssid(),
        (unsigned long)st.rx_count, (unsigned long)st.lost, (unsigned long)st.dup,
        (unsigned long)st.out_of_order, st.crc_err, st.hdr_err, st.pdr,
        st.have_packet ? "true" : "false", (unsigned long)st.last_seq,
        (unsigned long)st.since_last_s,
        st.last_rssi, st.last_snr, st.last_signal_rssi, st.noise_floor,
        st.snr_margin, st.link_budget, st.fade_margin,
        (unsigned long)st.log_seq, st.have_batt ? "true" : "false",
        st.batt_pin_mv, (unsigned long)st.batt_uptime_s,
        link_profile_is_provisional() ? "true" : "false",
        (unsigned long)link_profile_revert_in_s());

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
            "%s{\"seq\":%lu,\"len\":%u,\"type\":%u,\"rssi\":%.1f,\"snr\":%.1f,"
            "\"signal_rssi\":%.1f,\"gap\":%u,\"uptime_s\":%lu}",
            i ? "," : "", (unsigned long)e->seq, e->len, e->type,
            e->rssi, e->snr, e->signal_rssi, e->gap, (unsigned long)e->at_uptime_s);
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

    storage_lock();
    FILE *f = fopen(STORAGE_CSV_PATH, "r");
    if (f == NULL) {
        storage_unlock();
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no log file");
    }

    char chunk[512];
    size_t r;
    while ((r = fread(chunk, 1, sizeof(chunk), f)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, r) != ESP_OK) {
            fclose(f);
            storage_unlock();
            return ESP_FAIL;
        }
    }
    fclose(f);
    storage_unlock();
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

#endif

// ------------------------------------------------------------------ start ----

esp_err_t webserver_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 12;
    config.stack_size       = 6144;  // the log handlers format a lot of JSON
    // The default 5 s is tight when max-drain mode is saturating the WiFi TX path
    // or a phone client is dozing; both show up as EAGAIN on send.
    config.send_wait_timeout = 10;
    config.recv_wait_timeout = 10;

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
        { .uri = "/status",      .method = HTTP_GET, .handler = status_handler },
        { .uri = "/log",         .method = HTTP_GET, .handler = log_handler },
#if CONFIG_ROLE_SENDER
        { .uri = "/control",     .method = HTTP_GET, .handler = control_handler },
        { .uri = "/pushcfg",     .method = HTTP_GET, .handler = pushcfg_handler },
        { .uri = "/mode",        .method = HTTP_GET, .handler = mode_handler },
        { .uri = "/load",        .method = HTTP_GET, .handler = load_handler },
        { .uri = "/uptime",      .method = HTTP_GET, .handler = uptime_handler },
        { .uri = "/wifi",        .method = HTTP_GET, .handler = wifi_handler },
#else
        { .uri = "/data.csv",    .method = HTTP_GET, .handler = csv_handler },
        { .uri = "/clear",       .method = HTTP_GET, .handler = clear_handler },
        { .uri = "/reset",       .method = HTTP_GET, .handler = reset_handler },
#endif
    };
    for (int i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}
