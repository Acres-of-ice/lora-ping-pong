#include "battery.h"

#include <stdbool.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "battery";

#define BAT_ADC_UNIT     ADC_UNIT_1        // ADC2 is unusable while WiFi is active.
#define BAT_ADC_ATTEN    ADC_ATTEN_DB_12   // full range (~0..3.1V at the pin)
#define BAT_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#define BAT_SAMPLES      32                // averaged per reading to cut noise

#define NVS_NS        "calib"
#define NVS_KEY_RATIO "ratio"

// Below this the pin is reading nothing worth dividing by: no battery, or a
// divider that is not connected.
#define PIN_MIN_MV 100

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static bool                      s_calibrated;
static adc_channel_t             s_channel = CONFIG_BATTERY_ADC_CHANNEL;

// Read by the radio and HTTP tasks, written from HTTP; one 16-bit store is atomic.
static volatile uint16_t s_ratio_x1000 = CONFIG_BATTERY_DIVIDER_X1000;

// Discharge clock = value restored at boot + seconds elapsed since boot. The pair
// is read by the radio task and reset from the HTTP task, so both sides go through
// the spinlock: a 64-bit store is two 32-bit writes here, and a read landing
// between them gave a sample off by over an hour, or a mix of old and new values.
static uint32_t     s_base_uptime_s;
static int64_t      s_boot_us;
static portMUX_TYPE s_clock_mux = portMUX_INITIALIZER_UNLOCKED;

static void load_ratio(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;  // never calibrated -> keep the Kconfig default
    }
    uint16_t r = 0;
    if (nvs_get_u16(h, NVS_KEY_RATIO, &r) == ESP_OK &&
        r >= BATTERY_RATIO_MIN_X1000 && r <= BATTERY_RATIO_MAX_X1000) {
        s_ratio_x1000 = r;
    }
    nvs_close(h);
}

esp_err_t battery_init(void)
{
    s_boot_us = esp_timer_get_time();
    load_ratio();

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

    ESP_LOGI(TAG, "Battery sense: ADC1 channel %d, divider ratio %u.%03u%s",
             (int)s_channel, s_ratio_x1000 / 1000, s_ratio_x1000 % 1000,
             s_ratio_x1000 == CONFIG_BATTERY_DIVIDER_X1000 ? " (build default)" : " (calibrated)");
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

int battery_mv_from_pin(int pin_mv)
{
    return (int)(((int64_t)pin_mv * s_ratio_x1000 + 500) / 1000);
}

uint16_t battery_ratio_x1000(void)         { return s_ratio_x1000; }
uint16_t battery_ratio_default_x1000(void) { return CONFIG_BATTERY_DIVIDER_X1000; }

esp_err_t battery_set_ratio_x1000(uint32_t ratio_x1000)
{
    if (ratio_x1000 < BATTERY_RATIO_MIN_X1000 || ratio_x1000 > BATTERY_RATIO_MAX_X1000) {
        return ESP_ERR_INVALID_ARG;
    }
    s_ratio_x1000 = (uint16_t)ratio_x1000;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_u16(h, NVS_KEY_RATIO, (uint16_t)ratio_x1000);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    ESP_LOGW(TAG, "Divider ratio set to %u.%03u", (unsigned)ratio_x1000 / 1000,
             (unsigned)ratio_x1000 % 1000);
    return err;
}

esp_err_t battery_calibrate(uint32_t actual_mv, int *pin_mv)
{
    const int pin = battery_read_pin_mv();
    *pin_mv = pin;
    if (pin < PIN_MIN_MV) {
        return ESP_ERR_INVALID_STATE;
    }
    // Rounded to the nearest thousandth. actual_mv is capped by the caller well
    // below the point where x1000 would overflow.
    const uint32_t ratio = (actual_mv * 1000u + (uint32_t)pin / 2) / (uint32_t)pin;
    return battery_set_ratio_x1000(ratio);
}

void battery_ratio_reset(void)
{
    s_ratio_x1000 = CONFIG_BATTERY_DIVIDER_X1000;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_RATIO);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "Divider ratio back to the build default");
}

uint32_t battery_uptime_s(void)
{
    taskENTER_CRITICAL(&s_clock_mux);
    const uint32_t base = s_base_uptime_s;
    const int64_t  boot = s_boot_us;
    taskEXIT_CRITICAL(&s_clock_mux);

    const int64_t elapsed = esp_timer_get_time() - boot;
    return base + (uint32_t)((elapsed < 0 ? 0 : elapsed) / 1000000);
}

void battery_uptime_set(uint32_t s)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_clock_mux);
    s_base_uptime_s = s;
    s_boot_us       = now;
    taskEXIT_CRITICAL(&s_clock_mux);
}
