#include "runstate.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "runstate";

#define PARTITION "runstate"
#define NAMESPACE "run"

static nvs_handle_t s_nvs;
static bool         s_ready;

esp_err_t runstate_init(void)
{
    esp_err_t err = nvs_flash_init_partition(PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Never formatted (the partition was appended to the table after these
        // boards were first flashed), or from another NVS version: nothing in it
        // is worth keeping.
        err = nvs_flash_erase_partition(PARTITION);
        if (err == ESP_OK) {
            err = nvs_flash_init_partition(PARTITION);
        }
    }
    if (err == ESP_OK) {
        err = nvs_open_from_partition(PARTITION, NAMESPACE, NVS_READWRITE, &s_nvs);
    }
    s_ready = (err == ESP_OK);
    if (!s_ready) {
        ESP_LOGE(TAG, "Run state unavailable (%s): sequence and counters restart at every boot",
                 esp_err_to_name(err));
    }
    return err;
}

bool runstate_load(const char *key, void *out, size_t len)
{
    size_t stored = 0;
    if (!s_ready || nvs_get_blob(s_nvs, key, NULL, &stored) != ESP_OK || stored != len) {
        return false;
    }
    return nvs_get_blob(s_nvs, key, out, &stored) == ESP_OK;
}

void runstate_save(const char *key, const void *in, size_t len)
{
    if (s_ready && nvs_set_blob(s_nvs, key, in, len) == ESP_OK) {
        nvs_commit(s_nvs);
    }
}
