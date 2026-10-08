#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// State of the current run that must survive a reset: sequence numbers, counters,
// the discharge clock. It is rewritten with every packet, so it lives in its own
// NVS partition ("runstate" in partitions.csv) and wears that rather than the one
// holding settings and calibration.
//
// Sizing: about three NVS entries per save, in a 16-page partition, so each page
// is erased once per ~670 packets. At the default 5 s interval that is over ten
// years of flash endurance; sending back-to-back at SF7, about one year.

// Mount the partition, formatting it if it is new or unreadable. Call once at boot
// after nvs_flash_init(). On failure nothing persists, but nothing else breaks.
esp_err_t runstate_init(void);

// Copy a saved value into out. False, leaving out untouched, when nothing was
// saved under key or it has a different size (a build with another layout).
bool runstate_load(const char *key, void *out, size_t len);

void runstate_save(const char *key, const void *in, size_t len);
