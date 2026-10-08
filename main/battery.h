#pragma once

#include <stdint.h>
#include "esp_err.h"

// Battery sensing for the sender. Ported from ESP32Webserver/main/battery.c. On the
// Repeater PCB the BAT+ divider feeds IO0 = ADC1 channel 0. Its ratio (battery mV
// per sense-pin mV) starts at CONFIG_BATTERY_DIVIDER_X1000 and is calibrated
// against a multimeter from the dashboard: resistor tolerance and ADC gain error
// together put the real figure well away from the nominal one.

#define BATTERY_RATIO_MIN_X1000 1000   // 1.000: no divider at all
#define BATTERY_RATIO_MAX_X1000 20000  // 20.000: far beyond any sensible divider

// Configure ADC1 + calibration, and load the saved divider ratio.
esp_err_t battery_init(void);

// Averaged, calibrated millivolts at the sense pin - the reading as taken, with no
// divider applied. This is what goes on the air and into the receiver's CSV, so a
// later recalibration still converts the whole log correctly.
int battery_read_pin_mv(void);

// Battery millivolts for a pin reading: pin_mv times the divider ratio.
int battery_mv_from_pin(int pin_mv);

// The divider ratio x1000, and the build default it falls back to.
uint16_t battery_ratio_x1000(void);
uint16_t battery_ratio_default_x1000(void);

// Set the ratio, saved to NVS. ESP_ERR_INVALID_ARG outside the limits above.
esp_err_t battery_set_ratio_x1000(uint32_t ratio_x1000);

// Calibrate from a multimeter reading of the battery: measure the pin now and set
// ratio = actual / pin. *pin_mv gets the reading used. ESP_ERR_INVALID_STATE when
// the pin reads too low to divide by (nothing connected); ESP_ERR_INVALID_ARG when
// the resulting ratio is outside the limits, which means a mistyped reading.
esp_err_t battery_calibrate(uint32_t actual_mv, int *pin_mv);

// Back to the build default, forgetting any calibration.
void battery_ratio_reset(void);

// Discharge-cycle clock, in seconds. Distinct from esp_timer uptime: a brownout
// under max drain is expected, so the sender saves the clock with every packet and
// restores it at boot rather than restarting at zero and forking the receiver's
// graph.
uint32_t battery_uptime_s(void);

// Set the clock: the saved value at boot, or 0 to start a fresh discharge cycle.
void battery_uptime_set(uint32_t s);
