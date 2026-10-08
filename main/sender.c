#include "sender.h"

#include <string.h>

#include "battery.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link.h"
#include "net.h"
#include "nvs.h"
#include "powerload.h"
#include "runstate.h"
#include "sdkconfig.h"
#include "solar.h"

static const char *TAG = "sender";

#define CFG_PUSH_RETRIES 3

// Pause after a transmit the radio refused outright. See exchange().
#define TX_FAIL_BACKOFF_MS 1000

// While stopped, how often to poll the receiver for commands (see poll_receiver()):
// every few seconds, but never more than 1/POLL_DUTY_DIVISOR of the time on air. At
// SF12 one poll is a second or more, and a stopped sender should be mostly quiet.
#define POLL_INTERVAL_MS  5000
#define POLL_DUTY_DIVISOR 10

// After acting on a command the sender polls at once for the next, but first leaves
// the receiver time to read the answer it just sent: in continuous receive a second
// frame landing before the first is read overwrites it in the radio's buffer.
#define POLL_AFTER_CMD_MS 200

#define NVS_NS           "sender"
#define NVS_KEY_INTERVAL "interval"
#define NVS_KEY_PAYLOAD  "payload"

static SemaphoreHandle_t s_mutex;
static TaskHandle_t      s_task;

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
static uint8_t  s_peer_charge = CHARGE_UNKNOWN;  // the receiver's charger, from its ACKs

static sender_log_t s_log[SENDER_LOG_SIZE];
static uint32_t     s_log_seq;    // total entries ever written
static uint32_t     s_log_floor;  // entries below this were dropped by a counter reset

// Config-push handshake, owned by the radio task but requested from HTTP. Busy
// covers both states: queued, and the negotiation itself, which takes seconds.
static bool         s_push_req;
static bool         s_push_active;
static sx126x_cfg_t s_push_cfg;
static char         s_push_state[96] = "idle";

// A command from the receiver's dashboard, lifted off an ACK by the exchange and
// acted on by the task loop between exchanges, like a queued profile.
#define RUNSTATE_KEY_CMD "lastcmd"
typedef struct {
    uint8_t id;      // saved: each id is acted on once, even across a reboot
    uint8_t result;  // so a repeat gets the same answer
} last_cmd_t;
static bool       s_rcmd_pending;
static link_cmd_t s_rcmd;
static last_cmd_t s_last_cmd;
static char       s_remote_state[80];  // the last one, for this board's dashboard

static void lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_mutex); }

static void push_state(const char *s)
{
    lock();
    strncpy(s_push_state, s, sizeof(s_push_state) - 1);
    s_push_state[sizeof(s_push_state) - 1] = '\0';
    unlock();
}

// The run so far, saved with every packet so a reset carries on from it. Restarting
// the sequence at 1 after a brownout laid the new packets over the old ones in every
// graph keyed by sequence, and a discharge clock saved only every few minutes
// stepped the battery curve backwards by up to that much.
#define RUNSTATE_KEY "sender"

typedef struct {
    uint32_t next_seq;
    uint32_t tx, ack, timeout;
    uint32_t clock_s;  // discharge clock
} run_state_t;

static void checkpoint(void)
{
    // Held across the write, so a reset from the dashboard cannot land between
    // the snapshot and the save and then be overwritten by the older numbers.
    lock();
    const run_state_t st = {
        .next_seq = s_seq,
        .tx       = s_tx_count,
        .ack      = s_ack_count,
        .timeout  = s_timeout_count,
        .clock_s  = battery_uptime_s(),
    };
    runstate_save(RUNSTATE_KEY, &st, sizeof(st));
    unlock();
}

static void restore_run_state(void)
{
    runstate_load(RUNSTATE_KEY_CMD, &s_last_cmd, sizeof(s_last_cmd));

    run_state_t st;
    if (!runstate_load(RUNSTATE_KEY, &st, sizeof(st)) || st.next_seq == 0) {
        return;  // nothing saved yet: start at sequence 1 with the clock at 0
    }
    s_seq           = st.next_seq;
    s_tx_count      = st.tx;
    s_ack_count     = st.ack;
    s_timeout_count = st.timeout;
    battery_uptime_set(st.clock_s);
    ESP_LOGW(TAG, "Resuming run: next packet #%lu, discharge clock %lus, %lu sent / %lu acked",
             (unsigned long)st.next_seq, (unsigned long)st.clock_s,
             (unsigned long)st.tx, (unsigned long)st.ack);
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

// Sized for the longest acknowledgement, one carrying a command, or the window
// could close on it mid-air at slow profiles.
static uint32_t ack_timeout_ms(const sx126x_cfg_t *cfg)
{
    return (uint32_t)(sx126x_airtime_ms(cfg, LINK_ACK_MAX_LEN) * 2.0f) + 500;
}

// An acknowledgement can carry a command from the receiver's dashboard after its
// usual fields. Lifted off here; the task loop acts on it between exchanges.
static void take_command(const uint8_t *rx, int n)
{
    if (n < (int)LINK_ACK_MAX_LEN) {
        return;
    }
    const link_cmd_t *c = (const link_cmd_t *)(rx + sizeof(link_ack_t));
    if (c->id == 0) {
        return;
    }
    lock();
    s_rcmd         = *c;
    s_rcmd_pending = true;
    unlock();
}

// Transmit one packet and wait for its acknowledgement. Fills `e`.
static void exchange(link_pkt_type_t type, uint8_t *buf, uint8_t len, sender_log_t *e)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    lock();
    const uint32_t seq = s_seq++;
    unlock();
    // Saved before transmitting, with the sequence already past this packet: a
    // reset mid-exchange resumes after it instead of sending its number twice.
    checkpoint();
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
        // A radio that is failing outright, or being recovered by the driver,
        // fails instantly; at interval 0 the loop would then retry - and log - at
        // the tick rate. The pause comes out of the interval, so it only slows
        // cadences shorter than itself.
        vTaskDelay(pdMS_TO_TICKS(TX_FAIL_BACKOFF_MS));
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
        e->rtt_ms     = (uint32_t)((esp_timer_get_time() - t0) / 1000);
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
        s_peer_charge     = a->charge;
        unlock();

        // A round trip completed, which is the only real proof a profile works in
        // both directions.
        link_profile_traffic_ok();
        take_command(rx, n);
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

    if (len < sizeof(link_ping_t)) {
        len = sizeof(link_ping_t);
    }

    uint8_t buf[LINK_MAX_PAYLOAD];
    memset(buf, 0xA5, sizeof(buf));  // filler out to the requested size
    ((link_ping_t *)buf)->charge = (uint8_t)charge_state();

    sender_log_t e;
    exchange(PKT_PING, buf, len, &e);
    log_add(&e);

    if (e.acked) {
        ESP_LOGI(TAG, "#%lu %uB ACK rtt=%lums rx_rssi=%.0f rx_snr=%.1f",
                 (unsigned long)e.seq, e.len, (unsigned long)e.rtt_ms, e.ack_rssi, e.ack_snr);
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
    b->pin_mv      = (uint16_t)battery_read_pin_mv();
    b->uptime_s    = battery_uptime_s();
    b->ratio_x1000 = battery_ratio_x1000();
    b->charge      = (uint8_t)charge_state();

    sender_log_t e;
    exchange(PKT_BATT, buf, len, &e);
    log_add(&e);

    ESP_LOGI(TAG, "#%lu batt %.3fV (pin %umV) uptime=%us %s",
             (unsigned long)e.seq, battery_mv_from_pin(b->pin_mv) / 1000.0, b->pin_mv,
             (unsigned)b->uptime_s, e.acked ? "ACK" : "no ACK");
}

// Negotiate a new profile with the receiver, then adopt it here too. The CFG frame
// goes out at the *current* profile so the receiver is guaranteed to understand it.
static void do_cfg_push(void)
{
    lock();
    const sx126x_cfg_t want = s_push_cfg;
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

// While stopped there is no traffic, so no ACK for the receiver to put a command
// in, and its "start" could never arrive. A short poll every few seconds keeps that
// door open: the receiver answers it like a ping but counts nothing.
static void poll_receiver(bool running)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    uint8_t buf[sizeof(link_poll_t)];
    link_poll_t *p = (link_poll_t *)buf;
    const uint32_t seq = s_ctrl_seq++;
    link_put_hdr(buf, PKT_POLL, seq);
    p->charge  = (uint8_t)charge_state();
    p->running = running;
    if (sx126x_tx(buf, sizeof(buf), tx_timeout_ms(&cfg, sizeof(buf))) != ESP_OK) {
        return;
    }

    const uint32_t to = ack_timeout_ms(&cfg);
    sx126x_rx_start(to);
    uint8_t rx[LINK_MAX_PAYLOAD];
    const int64_t deadline = esp_timer_get_time() + (int64_t)to * 1000;
    while (esp_timer_get_time() < deadline) {
        const int n = sx126x_rx_wait(rx, sizeof(rx), NULL, 100);
        if (n > 0 && link_check(rx, n, PKT_ACK, sizeof(link_ack_t)) &&
            ((const link_ack_t *)rx)->hdr.seq == seq) {
            lock();
            s_peer_charge = ((const link_ack_t *)rx)->charge;
            unlock();
            link_profile_traffic_ok();  // a round trip, as good as a ping's
            take_command(rx, n);
            break;
        }
    }
    sx126x_standby();
}

static uint32_t poll_period_ms(void)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);
    const uint32_t spaced =
        (uint32_t)sx126x_airtime_ms(&cfg, sizeof(link_poll_t)) * POLL_DUTY_DIVISOR;
    return spaced > POLL_INTERVAL_MS ? spaced : POLL_INTERVAL_MS;
}

// Answer a command with the settings as they now stand, so the receiver's
// dashboard can show them.
static void send_cmdack(uint8_t id, uint8_t result)
{
    sx126x_cfg_t cfg;
    link_get_cfg(&cfg);

    uint8_t buf[sizeof(link_cmdack_t)];
    link_cmdack_t *a = (link_cmdack_t *)buf;
    link_put_hdr(buf, PKT_CMDACK, s_ctrl_seq++);
    lock();
    a->running     = s_running;
    a->interval_ms = s_interval_ms;
    a->payload_len = s_payload_len;
    unlock();
    a->id          = id;
    a->result      = result;
    a->mode        = (uint8_t)link_get_mode();
    a->load        = powerload_is_on();
    a->wifi        = net_wifi_is_on();
    a->ratio_x1000 = battery_ratio_x1000();
    sx126x_tx(buf, sizeof(buf), tx_timeout_ms(&cfg, sizeof(buf)));
}

// Act on a command from the receiver, through the same calls this board's own
// dashboard makes, then answer it.
static void run_remote_command(const link_cmd_t *c)
{
    // Each id is acted on once. Seeing one again means our answer was lost and the
    // receiver repeated the command, so only the answer is repeated. The id is
    // saved before acting, so not even "reboot" can replay itself into a loop.
    if (c->id == s_last_cmd.id) {
        send_cmdack(c->id, s_last_cmd.result);
        return;
    }
    s_last_cmd = (last_cmd_t){ .id = c->id, .result = LINK_CMD_DONE };
    runstate_save(RUNSTATE_KEY_CMD, &s_last_cmd, sizeof(s_last_cmd));

    bool ok = true;
    switch (c->op) {
        case LINK_CMD_STATUS:
        case LINK_CMD_REBOOT:  // after the answer, below
            break;
        case LINK_CMD_RUN:
            sender_set_running(c->arg != 0);
            break;
        case LINK_CMD_MODE:
            link_set_mode(c->arg == LINK_MODE_BATTERY ? LINK_MODE_BATTERY : LINK_MODE_RANGE);
            break;
        case LINK_CMD_INTERVAL:
            ok = sender_set_interval(c->arg) == ESP_OK;
            break;
        case LINK_CMD_PAYLOAD:
            ok = c->arg <= LINK_MAX_PAYLOAD && sender_set_payload_len((int)c->arg) == ESP_OK;
            break;
        case LINK_CMD_RESET_COUNTERS:
            sender_reset_counters();
            break;
        case LINK_CMD_PUSH_CFG: {
            // Queued like the dashboard's push: the next pass negotiates it with the
            // receiver, provisionally, on the profile in use now.
            sx126x_cfg_t cfg;
            link_cfg_from_wire(&c->cfg, &cfg);
            ok = link_cfg_valid(&cfg) && sender_request_cfg_push(&cfg) == ESP_OK;
            break;
        }
        case LINK_CMD_LOAD:
            powerload_set(c->arg != 0);
            break;
        case LINK_CMD_RESET_CLOCK:
            sender_reset_discharge_clock();
            break;
        case LINK_CMD_CALIBRATE: {
            int pin = 0;
            ok = c->arg >= 500 && c->arg <= 30000 && battery_calibrate(c->arg, &pin) == ESP_OK;
            break;
        }
        case LINK_CMD_RATIO_RESET:
            battery_ratio_reset();
            break;
        case LINK_CMD_WIFI_OFF:
            powerload_set(false);
            net_wifi_off();
            break;
        default:
            ok = false;
            break;
    }
    if (!ok) {
        s_last_cmd.result = LINK_CMD_REFUSED;
        runstate_save(RUNSTATE_KEY_CMD, &s_last_cmd, sizeof(s_last_cmd));
    }

    char what[56];
    link_cmd_describe(c, what, sizeof(what));
    lock();
    snprintf(s_remote_state, sizeof(s_remote_state), "%s: %s", what, ok ? "done" : "refused");
    unlock();
    ESP_LOGW(TAG, "Command from the receiver: %s (%s)", what, ok ? "done" : "refused");

    send_cmdack(c->id, s_last_cmd.result);
    if (c->op == LINK_CMD_REBOOT) {
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
}

// Block for up to `ms`, returning early when sender_wake() is called.
static void wait_or_wake(uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) {
        ticks = 1;  // always block, so the idle task gets the CPU
    }
    ulTaskNotifyTake(pdTRUE, ticks);
}

static void sender_task(void *arg)
{
    // Give the receiver a moment to reach its listening loop after a co-power-up.
    vTaskDelay(pdMS_TO_TICKS(1000));

    int64_t cycle_start  = 0;
    bool    cycled       = false;  // no exchange yet, so the first is due at once
    int64_t next_poll_us = 0;
    bool    poll_soon    = false;  // just acted on a command: more may be waiting

    for (;;) {
        link_profile_tick();

        // Apply any profile queued by an HTTP handler here, between exchanges,
        // where it cannot interrupt a transmission.
        if (link_apply_pending()) {
            continue;
        }

        // Likewise a command the receiver's dashboard sent in an acknowledgement.
        lock();
        const bool       remote = s_rcmd_pending;
        const link_cmd_t rcmd   = s_rcmd;
        s_rcmd_pending = false;
        unlock();
        if (remote) {
            run_remote_command(&rcmd);
            poll_soon = true;
            continue;
        }

        lock();
        const bool push = s_push_req;
        if (push) {
            s_push_req    = false;
            s_push_active = true;
        }
        unlock();
        if (push) {
            do_cfg_push();
            lock();
            s_push_active = false;
            unlock();
            continue;
        }

        lock();
        const bool     run      = s_running;
        const uint32_t interval = s_interval_ms;
        unlock();

        // After a command, ask for the next at once rather than an interval from
        // now; any push it queued has gone first, above. While stopped, check in
        // every so often.
        const int64_t now = esp_timer_get_time();
        if (poll_soon || (!run && now >= next_poll_us)) {
            if (poll_soon) {
                vTaskDelay(pdMS_TO_TICKS(POLL_AFTER_CMD_MS));
            }
            poll_soon    = false;
            next_poll_us = now + (int64_t)poll_period_ms() * 1000;
            poll_receiver(run);
            continue;
        }
        if (!run) {
            wait_or_wake(100);
            continue;
        }

        // Pace from the start of the previous cycle, re-reading the interval each
        // pass so a new one counts from the cycle already under way. The wait ends
        // early when a dashboard control changes something (sender_wake()), so a
        // queued profile, a push or a stop takes effect at once instead of after an
        // interval that can be ten minutes long; any remaining wait then resumes.
        if (cycled) {
            const int64_t wait_us = cycle_start + (int64_t)interval * 1000 - esp_timer_get_time();
            if (wait_us > 0) {
                wait_or_wake((uint32_t)((wait_us + 999) / 1000));
                continue;
            }
        }
        cycled      = true;
        cycle_start = esp_timer_get_time();

        if (link_get_mode() == LINK_MODE_BATTERY) {
            send_battery();
        } else {
            send_ping();
        }

        // interval == 0 means back-to-back: the next packet goes out as soon as the
        // acknowledgement is in. The same path covers an interval shorter than the
        // exchange itself, which is easy to hit at SF12.
        //
        // vTaskDelay(1) rather than no delay at all: the loop must yield or it
        // starves the idle task and trips the task watchdog. At 1000 Hz that is 1 ms,
        // against a minimum exchange of tens of milliseconds.
        vTaskDelay(1);
    }
}

esp_err_t sender_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    load_settings();      // before the task starts, so the first packet uses them
    restore_run_state();  // likewise: the first packet continues the saved run
    if (xTaskCreate(sender_task, "sender", 5120, NULL, 5, &s_task) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void sender_wake(void)
{
    if (s_task != NULL) {
        xTaskNotifyGive(s_task);
    }
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
    out->peer_charge   = s_peer_charge;
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
    // Entries older than the ring are gone, and so are any from before the last
    // counter reset; start from the oldest we still hold.
    uint32_t from = since;
    uint32_t oldest = (head > SENDER_LOG_SIZE) ? head - SENDER_LOG_SIZE : 0;
    if (oldest < s_log_floor) {
        oldest = s_log_floor;
    }
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
    sender_wake();
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
    sender_wake();  // a shorter interval may already be due
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
    // Without this the dashboard's log, which restarts from cursor 0 after a
    // reset, refilled at once with every packet still in the ring.
    s_log_floor = s_log_seq;
    unlock();
    // Saved at once, or a reset followed by a reboot would bring the old counts
    // back. The sequence keeps counting: restarting it is what broke the graphs.
    checkpoint();
    ESP_LOGW(TAG, "Counters reset");
}

void sender_reset_discharge_clock(void)
{
    battery_uptime_set(0);
    checkpoint();
    ESP_LOGW(TAG, "Discharge cycle reset to 0");
}

esp_err_t sender_request_cfg_push(const sx126x_cfg_t *cfg)
{
    lock();
    if (s_push_req || s_push_active) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_push_cfg = *cfg;
    // Set before the flag, under the same lock, so the radio task's own progress
    // updates always land after it rather than being overwritten by it.
    strcpy(s_push_state, "queued");
    s_push_req = true;
    unlock();
    sender_wake();
    return ESP_OK;
}

bool sender_cfg_push_busy(void)
{
    lock();
    const bool b = s_push_req || s_push_active;
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

void sender_remote_command_state(char *out, size_t n)
{
    lock();
    strncpy(out, s_remote_state, n - 1);
    out[n - 1] = '\0';
    unlock();
}
