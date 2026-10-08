#ifndef SOLAR_H
#define SOLAR_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// CN3791 solar charger state. The values travel over the air (each board reports
// its own to the other), so they are fixed.
typedef enum
{
    CHARGE_NONE     = 0,     // not charging
    CHARGE_SOLAR    = 1,     // charging from the panel
    CHARGE_COMPLETE = 2,     // charged
    CHARGE_UNKNOWN  = 0xFF,  // status pins unreadable, or no report received yet
} charge_state_t;

esp_err_t charge_gpio_init_once(void);
charge_state_t charge_state(void);
const char *charge_state_str(charge_state_t s);
const char *battery_charge_status(void);

#endif
