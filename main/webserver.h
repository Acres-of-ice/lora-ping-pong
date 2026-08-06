#pragma once

#include "esp_err.h"

// Start the dashboard HTTP server. Routes depend on the configured role; see
// webserver.c. Call after net_start().
esp_err_t webserver_start(void);
