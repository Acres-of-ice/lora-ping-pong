#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Maximum-drain mode for the battery test: continuous full-size WiFi broadcasts
// at 21 dBm with power save off. Ported from ESP32Webserver/main/powerload.c,
// with the peer moved to the AP interface and the auto-start removed - here it is
// armed by the battery-test mode switch, not at boot.
//
// Combined with the SX1262 transmitting at +22 dBm this is roughly the worst case
// the board can draw, which is the point: it is the load profile under test.

// Configure ESP-NOW + radio. Call after net_start() has succeeded.
//
// Starts in whatever state was last saved, defaulting to ON. That default is
// deliberate: this mode provokes brownout resets, and coming back up with the load
// off would bend the discharge curve in a way that looks like the battery rather
// than a missing load.
esp_err_t powerload_init(void);

// The load is held off for CONFIG_POWERLOAD_ARM_DELAY_S after boot so the dashboard
// is always reachable, and stands itself down after several consecutive brownout
// resets - a load the supply cannot carry would otherwise persist itself into an
// unrecoverable reset loop.

// Persisted to NVS, so the choice survives a reset.
void powerload_set(bool on);

// Suspend the flood for CONFIG_POWERLOAD_JOIN_GRACE_MS. Call when a station
// associates: the periodic breathing gap is free-running and not synchronised to
// the moment a fresh client actually needs quiet air, which is right after
// association while it negotiates DHCP. Starve that and the client's own
// connectivity checks give up fast - a join followed by a client-initiated leave
// (reason 3) within a couple hundred ms, rather than the ~4 s AP-side handshake
// timeout (reason 15) that the breathing gap already fixed.
void powerload_pause_for_join(void);

// Operator intent. May be true while the boot delay is still running.
bool powerload_is_on(void);

// Seconds until the load actually starts; 0 once armed.
uint32_t powerload_arming_in_s(void);
