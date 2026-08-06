#pragma once

#include <stdint.h>
#include "esp_err.h"

// Battery sensing for the sender. Ported from ESP32Webserver/main/battery.c; the
// ADC channel default is unchanged because A0/GPIO2 is the only ADC1 pin the
// SX1262 wiring leaves free.

// Configure ADC1 + calibration. Loads the discharge-cycle clock from NVS.
esp_err_t battery_init(void);

// Averaged, calibrated millivolts at the sense pin - the reading as taken, with no
// divider applied. Converting this to a battery voltage is a multiplication by the
// hardware divider ratio, done off-device against the logged CSV.
int battery_read_pin_mv(void);

// Discharge-cycle clock, in seconds. Distinct from esp_timer uptime: a brownout
// under max drain is expected, so this resumes from NVS instead of restarting at
// zero and forking the receiver's graph.
uint32_t battery_uptime_s(void);

// Checkpoint the clock to NVS. Called on the logging interval rather than every
// second, to keep flash wear proportionate to the test length.
void battery_uptime_persist(void);

// Start a fresh discharge cycle at zero.
void battery_uptime_reset(void);
