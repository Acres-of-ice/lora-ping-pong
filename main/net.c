#include "net.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#if CONFIG_ROLE_SENDER
#include "powerload.h"
#endif

static const char *TAG = "net";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRIES        10

static EventGroupHandle_t s_events;
static int  s_retries;
static bool s_on;
static char s_ip[16]  = "-";
static char s_ssid[32];

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " joined", MAC2STR(e->mac));
#if CONFIG_ROLE_SENDER
        // Give DHCP and a captive-portal check clear air; see powerload.h for why.
        powerload_pause_for_join();
#endif
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "client " MACSTR " left", MAC2STR(e->mac));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retries < MAX_RETRIES) {
            s_retries++;
            ESP_LOGW(TAG, "Disconnected, reconnecting (%d/%d)...", s_retries, MAX_RETRIES);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_events, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "=========================================");
        ESP_LOGI(TAG, " Dashboard: http://%s/", s_ip);
        ESP_LOGI(TAG, "=========================================");
        s_retries = 0;
        xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t net_start(void)
{
    s_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    snprintf(s_ssid, sizeof(s_ssid), "%s-%02X%02X",
             CONFIG_NET_AP_SSID_PREFIX, mac[4], mac[5]);

#if CONFIG_NET_MODE_STA
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        event_handler, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, CONFIG_NET_STA_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CONFIG_NET_STA_PASSWORD, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;  // accept open + WPA/WPA2

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_on = true;

    ESP_LOGI(TAG, "Joining '%s'...", CONFIG_NET_STA_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_events, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Failed to join '%s'", CONFIG_NET_STA_SSID);
        return ESP_FAIL;
    }
#else
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        event_handler, NULL, NULL));

    wifi_config_t wc = {
        .ap = {
            .channel        = CONFIG_NET_AP_CHANNEL,
            .max_connection = 4,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wc.ap.ssid, s_ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(s_ssid);
    strncpy((char *)wc.ap.password, CONFIG_NET_AP_PASSWORD, sizeof(wc.ap.password));
    if (strlen(CONFIG_NET_AP_PASSWORD) == 0) {
        wc.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_on = true;

    strncpy(s_ip, "192.168.4.1", sizeof(s_ip));
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, " AP '%s' pass '%s'", s_ssid, CONFIG_NET_AP_PASSWORD);
    ESP_LOGI(TAG, " Dashboard: http://%s/", s_ip);
    ESP_LOGI(TAG, "=========================================");
#endif

    // Power save costs latency on every dashboard poll for a saving that is noise
    // next to the LoRa PA. The battery test turns it off entirely anyway.
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    return ESP_OK;
}

void net_wifi_off(void)
{
    if (!s_on) {
        return;
    }
    ESP_LOGW(TAG, "WiFi off - dashboard unavailable until reset");
    esp_wifi_stop();
    s_on = false;
    strncpy(s_ip, "-", sizeof(s_ip));
}

bool net_wifi_is_on(void)     { return s_on; }
const char *net_ip_str(void)  { return s_ip; }
const char *net_ssid(void)    { return s_ssid; }
