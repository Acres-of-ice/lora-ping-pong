#include "solar.h"
#include <stdbool.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SOLAR_DONE GPIO_NUM_3
#define SOLAR_CHRG GPIO_NUM_5
#define SOLAR_CHRG_ACTIVE_LEVEL 0
#define SOLAR_DONE_ACTIVE_LEVEL 0

static const char *TAG = "SOLAR";
static bool s_charge_gpio_initialized = false;
static bool s_solar_chrg_ever_seen = false;

static bool solar_pin_active(gpio_num_t pin, int active_level)
{
    return gpio_get_level(pin) == active_level;
}

charge_state_t charge_state(void)
{
    if (charge_gpio_init_once() != ESP_OK)
    {
        return CHARGE_UNKNOWN;
    }

    const bool solar_raw = solar_pin_active(SOLAR_CHRG, SOLAR_CHRG_ACTIVE_LEVEL);
    const bool solar_done = solar_pin_active(SOLAR_DONE, SOLAR_DONE_ACTIVE_LEVEL);
    const bool solar_charging = solar_raw && !solar_done;

    if (solar_raw)
    {
        s_solar_chrg_ever_seen = true;
    }
    if (solar_done && s_solar_chrg_ever_seen)
    {
        return CHARGE_COMPLETE;
    }
    if (solar_charging)
    {
        return CHARGE_SOLAR;
    }
    return CHARGE_NONE;
}

const char *charge_state_str(charge_state_t s)
{
    switch (s)
    {
    case CHARGE_NONE:
        return "Not Charging";
    case CHARGE_SOLAR:
        return "Charging (Solar)";
    case CHARGE_COMPLETE:
        return "Solar Charging Complete";
    default:
        return "Unknown";
    }
}

const char *battery_charge_status(void)
{
    return charge_state_str(charge_state());
}

static void solar_task(void *arg)
{
    while (1)
    {
        ESP_LOGD(TAG, "Battery status: %s", battery_charge_status());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t charge_gpio_init_once(void)
{
    if (s_charge_gpio_initialized)
    {
        return ESP_OK;
    }

    // CHRG and DONE are open-drain: the CN3791 only ever pulls them low, so with no
    // panel, or between states, nothing drives them high. The internal pull-ups
    // make them read "off" then rather than floating, and are harmless alongside
    // external pull-ups to 3V3.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SOLAR_CHRG) | (1ULL << SOLAR_DONE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Solar charge GPIO init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_charge_gpio_initialized = true;
    xTaskCreate(solar_task, "solar_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}