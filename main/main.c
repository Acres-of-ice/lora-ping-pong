// LoRa range-test and battery-test rig.
//
// One firmware, two roles selected in menuconfig. The sender transmits numbered
// packets (or battery telemetry) and shows what came back; the receiver reports
// link quality and logs the battery run to CSV. Each has its own dashboard.

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "link.h"
#include "net.h"
#include "sx126x.h"
#include "webserver.h"

#if CONFIG_ROLE_SENDER
#include "battery.h"
#include "powerload.h"
#include "sender.h"
#else
#include "receiver.h"
#include "storage.h"
#endif

static const char *TAG = "main";

// Spelled out rather than printed as a bare number: under max drain the whole
// point is telling a brownout apart from a crash, and guessing at the enum
// ordering is how you misread that.
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "external pin";
        case ESP_RST_SW:        return "software restart";
        case ESP_RST_PANIC:     return "PANIC / exception";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        case ESP_RST_BROWNOUT:  return "BROWNOUT - supply sagged";
        case ESP_RST_SDIO:      return "SDIO";
        case ESP_RST_USB:       return "USB peripheral (flash/monitor)";
        case ESP_RST_JTAG:      return "JTAG";
        default:                return "unknown";
    }
}

void app_main(void)
{
#if CONFIG_ROLE_SENDER
    ESP_LOGI(TAG, "LoRa test rig starting - role: SENDER");
#else
    ESP_LOGI(TAG, "LoRa test rig starting - role: RECEIVER");
#endif
    const esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGW(TAG, "Reset reason: %s (%d)", reset_reason_str(reason), (int)reason);

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Radio first: a wiring fault should surface before WiFi fills the log.
    esp_err_t err = sx126x_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SX1262 init failed (%s).", esp_err_to_name(err));
        ESP_LOGE(TAG, "Check NSS/SCK/MOSI/MISO/RST/BUSY wiring and 3V3 to the module.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_ERROR_CHECK(link_init());

#if CONFIG_ROLE_SENDER
    ESP_ERROR_CHECK(battery_init());
#else
    ESP_ERROR_CHECK(storage_init());
#endif

    if (net_start() != ESP_OK) {
        ESP_LOGW(TAG, "No network; the radio still runs but the dashboard is unavailable");
    }

// #if CONFIG_ROLE_SENDER
//     // Not fatal: without WiFi there is no max-drain load, but the LoRa side works.
//     powerload_init();
// #endif

    if (net_wifi_is_on()) {
        webserver_start();
    }

#if CONFIG_ROLE_SENDER
    ESP_ERROR_CHECK(sender_start());
#else
    ESP_ERROR_CHECK(receiver_start());
#endif

    ESP_LOGI(TAG, "Ready - dashboard at http://%s/", net_ip_str());
}
