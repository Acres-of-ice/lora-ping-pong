#include "powerload.h"

#include <string.h>

#include "esp_log.h"
#include "esp_now.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "net.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "powerload";

#define MAX_TX_POWER 84  // 84 * 0.25 dBm = 21 dBm (maximum)
#define PAYLOAD_LEN  ESP_NOW_MAX_DATA_LEN

#define NVS_NS         "powerload"
#define NVS_KEY_ON     "on"
#define NVS_KEY_STREAK "bo_streak"

// Consecutive brownout resets before the load gives up and starts off. See
// powerload_init() - this is what stops a load too heavy for the supply from
// locking the board in an unreachable reset loop.
#define BROWNOUT_GIVE_UP 3

// Once the board has survived this long with the load on, the supply is evidently
// coping and the brownout streak is cleared.
#define STREAK_CLEAR_MS 60000

#if CONFIG_NET_MODE_STA
#define LOAD_IFIDX WIFI_IF_STA
#else
#define LOAD_IFIDX WIFI_IF_AP
#endif

static const uint8_t BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static volatile bool     s_on;      // operator intent, persisted
static volatile bool     s_armed;   // boot delay elapsed
static volatile bool     s_ready;
static SemaphoreHandle_t s_send_sem;  // given by the send callback to pace back-to-back TX
static uint8_t           s_payload[PAYLOAD_LEN];
static int64_t           s_arm_at_us;
static volatile int64_t  s_join_grace_until_us;  // 0 = no grace in effect

static uint8_t nvs_get_u8_or(const char *key, uint8_t fallback)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return fallback;
    }
    uint8_t v = fallback;
    if (nvs_get_u8(h, key, &v) != ESP_OK) {
        v = fallback;
    }
    nvs_close(h);
    return v;
}

static void nvs_put_u8(const char *key, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_u8(h, key, v) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

// Wake the task when a broadcast completes, so TX runs back-to-back at the
// radio's pace rather than in a CPU busy-wait. Runs in WiFi task context.
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    (void)status;
    xSemaphoreGive(s_send_sem);
}

static bool running(void)
{
    if (!s_on || !s_armed) {
        return false;
    }
    return esp_timer_get_time() >= s_join_grace_until_us;
}

void powerload_pause_for_join(void)
{
    const int64_t until = esp_timer_get_time() + (int64_t)CONFIG_POWERLOAD_JOIN_GRACE_MS * 1000;
    // A second client joining mid-grace should extend it, not shorten it.
    if (until > s_join_grace_until_us) {
        s_join_grace_until_us = until;
    }
    if (s_on && s_armed) {
        ESP_LOGI(TAG, "Station joining: pausing max-drain for %dms", CONFIG_POWERLOAD_JOIN_GRACE_MS);
    }
}

static void powerload_task(void *arg)
{
    // Hold off at boot so there is always a window in which the dashboard is
    // reachable. Without it, a load the supply cannot carry browns the board out
    // before app_main returns, and because the setting is persisted the board comes
    // back up and does it again - forever, with no way in to switch it off.
    vTaskDelay(pdMS_TO_TICKS(CONFIG_POWERLOAD_ARM_DELAY_S * 1000));
    s_armed = true;
    if (s_on) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        xSemaphoreGive(s_send_sem);
        ESP_LOGW(TAG, "MAX DRAIN armed: continuous TX @ 21 dBm, power-save off");
    }

    bool streak_cleared = false;
    const int64_t clear_at = esp_timer_get_time() + (int64_t)STREAK_CLEAR_MS * 1000;
    int64_t next_gap_us = esp_timer_get_time() +
                          (int64_t)CONFIG_POWERLOAD_GAP_PERIOD_MS * 1000;

    for (;;) {
        // Survived long enough under load that the supply is clearly coping.
        if (!streak_cleared && running() && esp_timer_get_time() > clear_at) {
            nvs_put_u8(NVS_KEY_STREAK, 0);
            streak_cleared = true;
            ESP_LOGI(TAG, "Stable under load; brownout streak cleared");
        }

        if (!running()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

#if CONFIG_POWERLOAD_GAP_MS > 0
        // Let the flood breathe. Saturating the TX path continuously starves the
        // WPA2 four-way handshake, so a client associates and is deauthed with
        // reason 15 a few seconds later - which locks you out of the dashboard that
        // would let you turn this off. A short periodic pause keeps the AP joinable
        // and TCP serviceable for a few mA.
        if (esp_timer_get_time() >= next_gap_us) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_POWERLOAD_GAP_MS));
            next_gap_us = esp_timer_get_time() +
                          (int64_t)CONFIG_POWERLOAD_GAP_PERIOD_MS * 1000;
        }
#endif

        if (esp_now_send(BROADCAST, s_payload, sizeof(s_payload)) != ESP_OK) {
            // TX queue full. This MUST block, not taskYIELD(): yielding re-enters
            // the scheduler, which picks the highest-priority ready task - this one -
            // so the idle task never runs and the task watchdog fires. At full rate
            // the queue is full most of the time, which turned that into an unbounded
            // busy-spin.
            //
            // Blocking costs no airtime either way: a full queue means the radio
            // already has frames to send. At 1000 Hz this is 1 ms.
            vTaskDelay(1);
            continue;
        }
        // Blocks until the send callback fires, so the next frame goes out the
        // instant the previous one is done - no fixed pacing anywhere. The timeout
        // only exists so a dropped completion callback cannot wedge the loop.
        xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(50));
    }
}

#if CONFIG_POWERLOAD_CPU_SPIN
// Keeps the core out of its idle state. FreeRTOS's idle task executes `waiti`,
// which halts the CPU until the next interrupt and is worth real milliamps - the
// opposite of what a worst-case draw test wants.
//
// Runs one priority above idle, so idle only gets the CPU during the deliberate
// yield below. That yield is mandatory, not politeness: the task watchdog is
// configured to watch the idle task (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0), so
// starving idle completely would trip it.
static void cpuload_task(void *arg)
{
    volatile uint32_t sink = 0;

    for (;;) {
        if (!running()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        const int64_t spin_until = esp_timer_get_time() + 20000;  // 20 ms
        while (esp_timer_get_time() < spin_until) {
            for (int i = 0; i < 1000; i++) {
                sink += (uint32_t)i;
            }
        }
        // 2 ticks, not 1: the idle task only gets the CPU when this task AND the
        // higher-priority powerload task are both blocked, so give that overlap a
        // wide enough window to actually happen. ~91% duty either way.
        vTaskDelay(2);
    }
}
#endif

esp_err_t powerload_init(void)
{
    if (!net_wifi_is_on()) {
        ESP_LOGW(TAG, "WiFi is off; max-drain unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    s_send_sem = xSemaphoreCreateBinary();
    if (s_send_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(s_payload, 0xA5, sizeof(s_payload));

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(MAX_TX_POWER));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, BROADCAST, sizeof(BROADCAST));
    peer.ifidx   = LOAD_IFIDX;
    peer.channel = 0;  // 0 = whatever channel the interface is already on
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // Decide the boot state: off on a fresh board, then whatever was last saved. The
    // saved state is what matters - a battery run resumes the same load after the
    // brownout it is expected to provoke near end of life, or the discharge curve
    // gets a kink that looks like the cell recovering.
    bool on = nvs_get_u8_or(NVS_KEY_ON, 0) != 0;

    // ...but a load the supply genuinely cannot carry browns out immediately and
    // repeatedly, and persisting it would make that unrecoverable. Track consecutive
    // brownouts and stand down after a few.
    uint8_t streak = nvs_get_u8_or(NVS_KEY_STREAK, 0);
    if (esp_reset_reason() == ESP_RST_BROWNOUT) {
        streak = (streak < 255) ? streak + 1 : 255;
        nvs_put_u8(NVS_KEY_STREAK, streak);
        ESP_LOGW(TAG, "Brownout reset #%u under load", streak);
        if (on && streak >= BROWNOUT_GIVE_UP) {
            on = false;
            nvs_put_u8(NVS_KEY_ON, 0);
            ESP_LOGE(TAG, "%u consecutive brownouts - standing the load down.", streak);
            ESP_LOGE(TAG, "The supply cannot carry it. Power from the battery or a");
            ESP_LOGE(TAG, "bench supply, or leave the CPU spin load off, then re-enable.");
        }
    } else if (streak != 0) {
        nvs_put_u8(NVS_KEY_STREAK, 0);
    }

    s_on    = on;
    s_armed = false;
    s_arm_at_us = esp_timer_get_time() + (int64_t)CONFIG_POWERLOAD_ARM_DELAY_S * 1000000;

    if (xTaskCreate(powerload_task, "powerload", 4096, NULL, 4, NULL) != pdPASS) {
        return ESP_FAIL;
    }
#if CONFIG_POWERLOAD_CPU_SPIN
    // Priority 1 = just above tskIDLE_PRIORITY, so it soaks up leftover CPU without
    // delaying the radio tasks or the HTTP server.
    if (xTaskCreate(cpuload_task, "cpuload", 2048, NULL, 1, NULL) != pdPASS) {
        return ESP_FAIL;
    }
#endif
    s_ready = true;

#if CONFIG_POWERLOAD_GAP_MS > 0
    ESP_LOGI(TAG, "Max-drain %s, arming in %ds, TX duty %d%% (%dms gap every %dms)",
             on ? "ON" : "OFF", CONFIG_POWERLOAD_ARM_DELAY_S,
             (100 * CONFIG_POWERLOAD_GAP_PERIOD_MS) /
                 (CONFIG_POWERLOAD_GAP_PERIOD_MS + CONFIG_POWERLOAD_GAP_MS),
             CONFIG_POWERLOAD_GAP_MS, CONFIG_POWERLOAD_GAP_PERIOD_MS);
#else
    ESP_LOGW(TAG, "Max-drain %s, arming in %ds, NO breathing gap - the AP will not"
                  " accept new clients while the load runs",
             on ? "ON" : "OFF", CONFIG_POWERLOAD_ARM_DELAY_S);
#endif
    return ESP_OK;
}

void powerload_set(bool on)
{
    if (!s_ready || on == s_on) {
        return;
    }
    s_on = on;
    if (on) {
        if (s_armed) {
            esp_wifi_set_ps(WIFI_PS_NONE);   // keep the radio fully powered
            xSemaphoreGive(s_send_sem);      // kick the first send
        }
        ESP_LOGW(TAG, "MAX DRAIN ON%s", s_armed ? "" : " (waiting for the boot delay)");
    } else {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // restore the low-power baseline
        ESP_LOGI(TAG, "MAX DRAIN OFF");
    }
    nvs_put_u8(NVS_KEY_ON, on ? 1 : 0);  // survives the resets this mode provokes
}

bool powerload_is_on(void)
{
    return s_on;
}

uint32_t powerload_arming_in_s(void)
{
    if (s_armed) {
        return 0;
    }
    const int64_t left = s_arm_at_us - esp_timer_get_time();
    return (left <= 0) ? 0 : (uint32_t)(left / 1000000) + 1;
}
