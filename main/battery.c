#include "battery.h"

#include <stdbool.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "battery";

#define BAT_ADC_UNIT     ADC_UNIT_1        // ADC2 is unusable while WiFi is active.
#define BAT_ADC_ATTEN    ADC_ATTEN_DB_12   // full range (~0..3.1V at the pin)
#define BAT_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#define BAT_SAMPLES      32                // averaged per reading to cut noise

#define NVS_NS         "calib"
#define NVS_KEY_UPTIME "uptime_s"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static bool                      s_calibrated;
static adc_channel_t             s_channel = CONFIG_BATTERY_ADC_CHANNEL;

// Discharge clock = value recovered at boot + seconds elapsed since boot.
static uint32_t s_base_uptime_s;
static int64_t  s_boot_us;

static void load_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;  // namespace doesn't exist yet -> keep the Kconfig default
    }

    uint32_t up = 0;
    if (nvs_get_u32(h, NVS_KEY_UPTIME, &up) == ESP_OK && up > 0) {
        s_base_uptime_s = up;
        ESP_LOGW(TAG, "Resuming discharge cycle at %us (survived a reset)", (unsigned)up);
    }
    nvs_close(h);
}

esp_err_t battery_init(void)
{
    s_boot_us = esp_timer_get_time();
    load_from_nvs();

    const adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = BAT_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc));

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg));

    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = BAT_ADC_UNIT,
        .chan     = s_channel,
        .atten    = BAT_ADC_ATTEN,
        .bitwidth = BAT_ADC_BITWIDTH,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        s_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
    } else {
        s_calibrated = false;
        ESP_LOGW(TAG, "ADC calibration unavailable; using nominal scaling");
    }

    ESP_LOGI(TAG, "Battery sense: ADC1 channel %d, reporting raw pin millivolts",
             (int)s_channel);
    return ESP_OK;
}

int battery_read_pin_mv(void)
{
    int acc = 0, got = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_channel, &raw) == ESP_OK) {
            acc += raw;
            got++;
        }
    }
    if (got == 0) {
        return 0;
    }
    const int raw_avg = acc / got;

    int mv;
    if (s_calibrated) {
        adc_cali_raw_to_voltage(s_cali, raw_avg, &mv);
    } else {
        mv = (raw_avg * 3100) / 4095;  // 12-bit full scale is ~3100 mV at 12 dB
    }
    return mv;
}

uint32_t battery_uptime_s(void)
{
    const int64_t elapsed = esp_timer_get_time() - s_boot_us;
    return s_base_uptime_s + (uint32_t)((elapsed < 0 ? 0 : elapsed) / 1000000);
}

void battery_uptime_persist(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_u32(h, NVS_KEY_UPTIME, battery_uptime_s()) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

void battery_uptime_reset(void)
{
    s_base_uptime_s = 0;
    s_boot_us = esp_timer_get_time();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u32(h, NVS_KEY_UPTIME, 0);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "Discharge cycle reset to 0");
}
