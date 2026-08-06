#include "storage.h"

#include <stdio.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "storage";
static SemaphoreHandle_t s_mutex;

esp_err_t storage_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = "storage",
        .max_files              = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return ret;
    }

    size_t total = 0, used = 0;
    if (esp_spiffs_info(conf.partition_label, &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    }

    // Create the file with a header the first time only.
    FILE *f = fopen(STORAGE_CSV_PATH, "r");
    if (f == NULL) {
        f = fopen(STORAGE_CSV_PATH, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Cannot create %s", STORAGE_CSV_PATH);
            return ESP_FAIL;
        }
        fputs(STORAGE_CSV_HEADER, f);
        fclose(f);
        ESP_LOGI(TAG, "Created new log file %s", STORAGE_CSV_PATH);
    } else {
        fclose(f);
        ESP_LOGI(TAG, "Existing log file found");
    }
    return ESP_OK;
}

esp_err_t storage_append(uint32_t uptime_s, uint16_t pin_mv, float rssi, float snr, uint32_t seq)
{
    storage_lock();
    FILE *f = fopen(STORAGE_CSV_PATH, "a");
    if (f == NULL) {
        storage_unlock();
        ESP_LOGE(TAG, "append: cannot open %s", STORAGE_CSV_PATH);
        return ESP_FAIL;
    }
    fprintf(f, "%u,%u,%.1f,%.2f,%u\n", (unsigned)uptime_s, pin_mv, rssi, snr, (unsigned)seq);
    // Flush every row: the sender is on a battery being deliberately run flat, so
    // a brownout mid-run should cost at most the in-flight sample.
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    storage_unlock();
    return ESP_OK;
}

uint32_t storage_last_uptime(void)
{
    storage_lock();
    FILE *f = fopen(STORAGE_CSV_PATH, "r");
    if (f == NULL) {
        storage_unlock();
        return 0;
    }
    char line[96];
    uint32_t last = 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] < '0' || line[0] > '9') {
            continue;  // header or blank line
        }
        unsigned up = 0;
        if (sscanf(line, "%u,", &up) == 1) {
            last = up;
        }
    }
    fclose(f);
    storage_unlock();
    return last;
}

esp_err_t storage_clear(void)
{
    storage_lock();
    FILE *f = fopen(STORAGE_CSV_PATH, "w");  // "w" truncates
    if (f == NULL) {
        storage_unlock();
        ESP_LOGE(TAG, "clear: cannot open %s", STORAGE_CSV_PATH);
        return ESP_FAIL;
    }
    fputs(STORAGE_CSV_HEADER, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    storage_unlock();
    ESP_LOGW(TAG, "Log cleared");
    return ESP_OK;
}

void storage_lock(void)   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
void storage_unlock(void) { xSemaphoreGive(s_mutex); }
