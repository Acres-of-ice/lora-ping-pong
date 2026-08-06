#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Bring up WiFi. In AP mode (the default, and what you want in the field) this
// returns as soon as the AP is up; in STA mode it blocks until it has an IP or
// gives up retrying.
esp_err_t net_start(void);

// Shut the radio down for a genuinely WiFi-free run. The dashboard goes away
// until the next reset - there is deliberately no runtime way back, because the
// point is to remove WiFi from the power budget entirely.
void net_wifi_off(void);

bool net_wifi_is_on(void);

// "192.168.4.1" in AP mode, the DHCP address in STA mode, "-" once off.
const char *net_ip_str(void);

// AP SSID, so the dashboard can show which board it is talking to.
const char *net_ssid(void);
