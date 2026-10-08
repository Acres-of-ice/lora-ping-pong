#pragma once

#include "esp_err.h"

// The dashboard's routes over the USB serial port, for bench work without joining
// the board's access point (and for a laptop whose only internet is its WiFi).
//
// Type a route on the serial console and the board fetches it from its own HTTP
// server over loopback, so every route behaves exactly as it does over WiFi:
//
//   /status
//   /radio?sf=9
//   @7 /sendercmd?op=run&on=0     <- "@id" tags the reply, for scripts
//
// The reply is framed so a script can pick it out of the log lines around it:
//
//   @@B <id> <path>
//   @@D <id> <up to 256 bytes of the raw HTTP response, as hex>
//   ...
//   @@E <id> <total bytes> +<ms taken> [error]
//
// tools/usb_bridge.py speaks this, and serves both dashboards on localhost over it.
esp_err_t console_start(void);
