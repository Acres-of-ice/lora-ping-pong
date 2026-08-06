#pragma once

#include <stdint.h>
#include "esp_err.h"

// SPIFFS-backed CSV log of the battery readings received over LoRa. Receiver only.
// Ported from ESP32Webserver/main/storage.c; the row now carries link quality
// alongside the reading so one file covers the whole run.
//
// pin_mv is the sender's sense-pin voltage as measured - the divider ratio is NOT
// applied. Multiply this column by the ratio to get battery volts.

#define STORAGE_CSV_PATH   "/spiffs/log.csv"
#define STORAGE_CSV_HEADER "uptime_s,pin_mv,rssi_dbm,snr_db,seq\n"

esp_err_t storage_init(void);

// Append one received reading. uptime_s is the *sender's* discharge clock, not
// the receiver's - that is the axis the discharge curve belongs on.
esp_err_t storage_append(uint32_t uptime_s, uint16_t pin_mv, float rssi, float snr, uint32_t seq);

// Uptime of the last data row, 0 if none. Lets the receiver tell whether an
// arriving sample continues the run or starts a new one.
uint32_t storage_last_uptime(void);

// Truncate back to the header row.
esp_err_t storage_clear(void);

// Serialise access so appends and the /data.csv stream never overlap.
void storage_lock(void);
void storage_unlock(void);
