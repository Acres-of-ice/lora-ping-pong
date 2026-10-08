#include "link.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "link";

#define NVS_NS       "radio"
#define NVS_KEY_CFG  "cfg"
#define NVS_KEY_MODE "mode"

const uint32_t LINK_BW_HZ[LINK_BW_COUNT] = {
    7810, 10420, 15630, 20830, 31250, 41670, 62500, 125000, 250000, 500000
};
const uint8_t LINK_BW_CODE[LINK_BW_COUNT] = {
    SX126X_BW_7810,  SX126X_BW_10420,  SX126X_BW_15630, SX126X_BW_20830,
    SX126X_BW_31250, SX126X_BW_41670,  SX126X_BW_62500, SX126X_BW_125000,
    SX126X_BW_250000, SX126X_BW_500000
};

uint8_t link_bw_from_hz(uint32_t hz)
{
    for (int i = 0; i < LINK_BW_COUNT; i++) {
        if (LINK_BW_HZ[i] == hz) {
            return LINK_BW_CODE[i];
        }
    }
    return SX126X_BW_125000;
}

static SemaphoreHandle_t s_mutex;
static sx126x_cfg_t      s_cfg;
static link_mode_t       s_mode;

// Provisional-profile state, guarded by the same mutex as the profile.
static bool         s_prov;
static sx126x_cfg_t s_prov_fallback;
static int64_t      s_prov_since_us;    // when the profile went live
static int64_t      s_prov_last_ok_us;  // last proof the link works under it
static uint32_t     s_prov_silence_s;   // derived; see link_profile_silence_s()

// Profile queued by an HTTP handler for the radio task to apply.
static bool         s_apply_req;
static sx126x_cfg_t s_apply_cfg;
static bool         s_apply_persist;

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

void link_cfg_to_wire(const sx126x_cfg_t *cfg, link_cfg_wire_t *out)
{
    out->freq_hz   = cfg->freq_hz;
    out->sf        = cfg->sf;
    out->bw        = cfg->bw;
    out->cr        = cfg->cr;
    out->preamble  = cfg->preamble;
    out->tx_dbm    = cfg->tx_dbm;
    out->sync_word = cfg->sync_word;
    out->crc_on    = cfg->crc_on ? 1 : 0;
}

void link_cfg_from_wire(const link_cfg_wire_t *w, sx126x_cfg_t *out)
{
    out->freq_hz   = w->freq_hz;
    out->sf        = w->sf;
    out->bw        = w->bw;
    out->cr        = w->cr;
    out->preamble  = w->preamble;
    out->tx_dbm    = w->tx_dbm;
    out->sync_word = w->sync_word;
    out->crc_on    = w->crc_on != 0;
}

bool link_cfg_equal(const sx126x_cfg_t *a, const sx126x_cfg_t *b)
{
    // Field by field rather than memcmp: sx126x_cfg_t has padding, and stale
    // padding bytes would make identical profiles compare unequal.
    return a->freq_hz   == b->freq_hz  && a->sf        == b->sf &&
           a->bw        == b->bw       && a->cr        == b->cr &&
           a->preamble  == b->preamble && a->tx_dbm    == b->tx_dbm &&
           a->sync_word == b->sync_word && a->crc_on   == b->crc_on;
}

bool link_cfg_valid(const sx126x_cfg_t *cfg)
{
    bool bw_ok = false;
    for (int i = 0; i < LINK_BW_COUNT; i++) {
        if (LINK_BW_CODE[i] == cfg->bw) {
            bw_ok = true;
            break;
        }
    }
    return bw_ok &&
           cfg->freq_hz >= LINK_FREQ_MIN_HZ && cfg->freq_hz <= LINK_FREQ_MAX_HZ &&
           cfg->sf >= 5 && cfg->sf <= 12 &&
           cfg->cr >= 5 && cfg->cr <= 8 &&
           cfg->preamble >= 6 &&
           cfg->tx_dbm >= -9 && cfg->tx_dbm <= 22;
}

int link_put_hdr(void *buf, link_pkt_type_t type, uint32_t seq)
{
    link_hdr_t *h = (link_hdr_t *)buf;
    h->magic = LINK_MAGIC;
    h->type  = (uint8_t)type;
    h->ver   = LINK_VERSION;
    h->seq   = seq;
    return (int)sizeof(link_hdr_t);
}

bool link_check(const void *buf, int len, link_pkt_type_t type, int need_len)
{
    if (len < (int)sizeof(link_hdr_t) || len < need_len) {
        return false;
    }
    const link_hdr_t *h = (const link_hdr_t *)buf;
    return h->magic == LINK_MAGIC && h->ver == LINK_VERSION && h->type == (uint8_t)type;
}

// ---------------------------------------------------------------- config ----

static void load_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;  // never saved yet; the Kconfig defaults stand
    }

    link_cfg_wire_t w;
    size_t sz = sizeof(w);
    if (nvs_get_blob(h, NVS_KEY_CFG, &w, &sz) == ESP_OK && sz == sizeof(w)) {
        link_cfg_from_wire(&w, &s_cfg);
        ESP_LOGI(TAG, "Loaded radio profile from NVS");
    }

    uint8_t m = 0;
    if (nvs_get_u8(h, NVS_KEY_MODE, &m) == ESP_OK) {
        s_mode = (m == LINK_MODE_BATTERY) ? LINK_MODE_BATTERY : LINK_MODE_RANGE;
    }
    nvs_close(h);
}

esp_err_t link_persist_cfg(void)
{
    lock();
    link_cfg_wire_t w;
    link_cfg_to_wire(&s_cfg, &w);
    const uint8_t m = (uint8_t)s_mode;
    unlock();

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY_CFG, &w, sizeof(w));
    if (err == ESP_OK) {
        err = nvs_set_u8(h, NVS_KEY_MODE, m);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t link_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_cfg = (sx126x_cfg_t){
        .freq_hz   = (uint32_t)CONFIG_LINK_FREQ_HZ,
        .sf        = CONFIG_LINK_DEFAULT_SF,
        .bw        = SX126X_BW_125000,
        .cr        = CONFIG_LINK_DEFAULT_CR,
        .preamble  = CONFIG_LINK_DEFAULT_PREAMBLE,
        .tx_dbm    = CONFIG_LINK_DEFAULT_TX_DBM,
        .sync_word = 0x12,  // private network
        .crc_on    = true,
    };
    s_mode = LINK_MODE_RANGE;

    load_from_nvs();
    return sx126x_apply(&s_cfg);
}

void link_get_cfg(sx126x_cfg_t *out)
{
    lock();
    *out = s_cfg;
    unlock();
}

esp_err_t link_set_cfg(const sx126x_cfg_t *cfg, bool persist)
{
    lock();
    s_cfg = *cfg;
    unlock();

    esp_err_t err = sx126x_apply(cfg);
    if (err != ESP_OK) {
        return err;
    }
    return persist ? link_persist_cfg() : ESP_OK;
}

link_mode_t link_get_mode(void)
{
    lock();
    link_mode_t m = s_mode;
    unlock();
    return m;
}

esp_err_t link_set_mode(link_mode_t mode)
{
    lock();
    s_mode = mode;
    unlock();
    ESP_LOGW(TAG, "Mode -> %s", link_mode_str(mode));
    return link_persist_cfg();
}

const char *link_mode_str(link_mode_t mode)
{
    return (mode == LINK_MODE_BATTERY) ? "battery" : "range";
}

// --------------------------------------------------------- apply queueing ----

esp_err_t link_request_apply(const sx126x_cfg_t *cfg, bool persist)
{
    lock();
    s_apply_cfg     = *cfg;
    s_apply_persist = persist;
    s_apply_req     = true;
    unlock();
    return ESP_OK;
}

bool link_apply_pending(void)
{
    lock();
    if (!s_apply_req) {
        unlock();
        return false;
    }
    const sx126x_cfg_t c = s_apply_cfg;
    const bool         p = s_apply_persist;
    s_apply_req = false;
    unlock();

    link_set_cfg(&c, p);
    return true;
}

// ---------------------------------------------------- provisional profiles ----

uint32_t link_profile_silence_s(const sx126x_cfg_t *cfg, uint32_t cadence_ms)
{
    // A full-size packet plus its acknowledgement is one exchange. Deliberately
    // uses LINK_MAX_PAYLOAD rather than the configured size: at SF12/BW7.81kHz that
    // is ~163 s, and underestimating here is what makes the timer fire mid-packet.
    const float exchange_ms = sx126x_airtime_ms(cfg, LINK_MAX_PAYLOAD) +
                              sx126x_airtime_ms(cfg, sizeof(link_ack_t));

    const uint32_t cycles_s =
        (uint32_t)(((exchange_ms + (float)cadence_ms) * LINK_PROV_SILENCE_CYCLES) / 1000.0f) + 1;

    return (cycles_s > LINK_PROV_SILENCE_MIN_S) ? cycles_s : LINK_PROV_SILENCE_MIN_S;
}

void link_profile_provisional(const sx126x_cfg_t *fallback, uint32_t silence_s)
{
    if (silence_s < LINK_PROV_SILENCE_MIN_S) {
        silence_s = LINK_PROV_SILENCE_MIN_S;
    }
    const int64_t now = esp_timer_get_time();
    lock();
    s_prov_fallback   = *fallback;
    s_prov            = true;
    s_prov_since_us   = now;
    s_prov_last_ok_us = now;   // silence is measured from the switch itself
    s_prov_silence_s  = silence_s;
    unlock();
    ESP_LOGW(TAG, "Profile is provisional: reverts after %us of silence, "
                  "commits after %us of traffic",
             (unsigned)silence_s, (unsigned)(silence_s * LINK_PROV_COMMIT_MULTIPLE));
}

void link_profile_traffic_ok(void)
{
    lock();
    if (s_prov) {
        s_prov_last_ok_us = esp_timer_get_time();
    }
    unlock();
}

bool link_profile_is_provisional(void)
{
    lock();
    const bool p = s_prov;
    unlock();
    return p;
}

uint32_t link_profile_revert_in_s(void)
{
    lock();
    const bool     p    = s_prov;
    const int64_t  last = s_prov_last_ok_us;
    const uint32_t win  = s_prov_silence_s;
    unlock();

    if (!p) {
        return 0;
    }
    const int64_t left = (last + (int64_t)win * 1000000) - esp_timer_get_time();
    return (left <= 0) ? 0 : (uint32_t)(left / 1000000) + 1;
}

void link_profile_tick(void)
{
    const int64_t now = esp_timer_get_time();

    lock();
    if (!s_prov) {
        unlock();
        return;
    }
    const uint32_t silence_s = s_prov_silence_s;
    const uint32_t commit_s  = silence_s * LINK_PROV_COMMIT_MULTIPLE;
    const bool silent = (now - s_prov_last_ok_us) > (int64_t)silence_s * 1000000;
    const bool proven = (now - s_prov_since_us)   > (int64_t)commit_s * 1000000;

    // Silence is checked first, so a profile that only works one way reverts
    // instead of committing on the strength of half a link.
    if (silent) {
        const sx126x_cfg_t revert = s_prov_fallback;
        s_prov = false;
        s_cfg  = revert;
        unlock();
        ESP_LOGE(TAG, "No traffic for %us under the new profile; reverting",
                 (unsigned)silence_s);
        sx126x_apply(&revert);
        link_persist_cfg();
        return;
    }
    if (proven) {
        s_prov = false;
        unlock();
        ESP_LOGI(TAG, "Profile carried traffic for %us; committing", (unsigned)commit_s);
        link_persist_cfg();
        return;
    }
    unlock();
}

// ------------------------------------------------------------------ json ----

int link_cfg_json(char *buf, size_t n, const sx126x_cfg_t *cfg, uint8_t payload_len)
{
    return snprintf(buf, n,
        "\"freq_hz\":%lu,\"sf\":%u,\"bw_hz\":%lu,\"cr\":%u,\"preamble\":%u,"
        "\"tx_dbm\":%d,\"crc\":%s,\"sync_word\":%u,\"payload_len\":%u,"
        "\"ldro\":%s,\"airtime_ms\":%.2f,\"bitrate_bps\":%.1f,\"sensitivity_dbm\":%.1f,"
        "\"snr_floor_db\":%.1f",
        (unsigned long)cfg->freq_hz, cfg->sf, (unsigned long)sx126x_bw_hz(cfg->bw),
        cfg->cr, cfg->preamble, cfg->tx_dbm, cfg->crc_on ? "true" : "false",
        cfg->sync_word, payload_len,
        sx126x_ldro_required(cfg->sf, cfg->bw) ? "true" : "false",
        sx126x_airtime_ms(cfg, payload_len),
        sx126x_bitrate_bps(cfg, payload_len),
        sx126x_sensitivity_dbm(cfg),
        sx126x_snr_floor_db(cfg->sf));
}
