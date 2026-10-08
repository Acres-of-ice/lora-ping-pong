# Repeater rig — hardware test report, 2026-10-08

Two Repeater PCBs side by side on the bench, both on USB power with a cell on BAT+
(4.13 V at the sender's divider). Driven entirely over USB with `tools/usb_bridge.py`;
every check is in `tools/rig_test.py` and can be re-run.

| Board | Role | USB serial | Port today |
|---|---|---|---|
| 44:BD:8D:23:3A:24 | sender | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:BD:8D:23:3A:24-if00` | ttyACM2 |
| 44:BD:8D:23:3A:B8 | receiver | `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:BD:8D:23:3A:B8-if00` | ttyACM1 |

`ttyACM0` on this laptop is its Fibocom L830 LTE modem, not a board — use the by-id paths.

## Result

**35/35 tests pass on the final firmware** (sender `fa35bc1d…`, receiver `a398738e…`), plus:

- a **30-minute soak** at interval 0 under 4,592 concurrent dashboard requests:
  3,949 sent / 3,949 acknowledged / 3,949 received, 0 lost, 0 CRC errors, 0 duplicates,
  heap flat on both boards (run on the build before the counter-reset fix);
- an **unplanned 1.5-hour run** at a 1 s interval on the final sender firmware:
  5,341 sent / 5,341 acknowledged, receiver PDR 100%.

## Bugs found and fixed

1. **A local `/radio` change during a provisional window was undone.** After a push, a
   `/radio` on either board left the provisional timer running: it could still revert the
   operator's explicit choice, and because `link_persist_cfg()` saves the *fallback* while
   provisional, NVS kept the old profile, so a reset brought it back. Confirmed on the
   hardware (still "reverting in 89 s" after `/radio` on both), fixed in `link_set_cfg()`
   — a persisted change now ends the window — and re-verified including across a reboot.
2. **"Reset counters" could leave more ACKs than packets sent** (an ACK rate over 100%).
   `/control?reset=1` runs on the HTTP task and can land between a packet's TxDone and its
   ACK; the packet's "sent" was zeroed, its ACK then counted. Measured on the old firmware:
   15% of resets at 255 B, 21% at 17 B. Fixed with a reset epoch in `sender.c` (an exchange
   that straddles a reset counts in no tally). The regression test fails on the old
   firmware ("ack 6 > sent 5") and passes on the new over 60 random resets.
3. **Every request over loopback took ~250 ms (seconds for long ones).** httpd writes each
   header field separately; on loopback they arrive as a burst that overflowed the client
   socket's 6-slot mailbox, and lwIP retries refused data only on its 250 ms timer.
   `CONFIG_LWIP_TCP_RECVMBOX_SIZE=32` (in `sdkconfig.defaults`) plus `TCP_NODELAY` for
   loopback peers only: **250 ms → 15 ms**. WiFi clients keep Nagle, so a busy channel
   still sees coalesced headers.

## The status LED (new)

WS2812 U8 on IO4, VDD on BAT+ (from the schematic). One task, strict priority:
radio fault (red 5 Hz) › **max drain: solid white at 255** › max drain arming (white 2 Hz)
› event flashes › background blinks. White is the maximum-power colour: all three dies at
full current. Full table in the README and in each dashboard's *Status LED* panel.

Verified from the firmware's own colour trace (`/led?trace=1`), timings to the millisecond:

| Check | Measured |
|---|---|
| Max drain | `#ffffff`, no flash ever breaks it, for the whole 90 s run; follows load on/off, WiFi off, brightness 0 |
| Max drain arming after reboot | "max drain arming" for 15 s, then solid white |
| Sender per packet | blue 20 ms at TX, green 60 ms on ACK (450 ms later = RTT) |
| Receiver per packet | green 60 ms; red 200 ms when packets were lost before it |
| Commit / revert | green-cyan / red-cyan, 150 ms each, on both boards |
| Remote command | violet double flash on the receiver when the sender confirms |
| Self-test | red, green, blue, white at 255, 500 ms apart |
| Link down | sender after 3 misses (5 s at 1 s interval), receiver after 21 s silence |
| Stopped / listening / command waiting / low battery / idle heartbeat | each shown in its state |
| Brightness 0 | dark, no flashes; max drain still full white; persists across reboot |

## Radio profiles (bench, −9 dBm)

Round-trip time against the airtime predicted by `sx126x_airtime_ms()` (packet + 14 B ACK):

| Profile | Payload | Predicted | Measured | SNR |
|---|---|---|---|---|
| SF5 / SF6 / SF7 | 255 B | 152 / 258 / 446 ms | 156 / 262 / 451 ms | 7.8 / 10.2 / 13.2 dB |
| SF8 / SF9 / SF10 | 255 B | 790 / 1415 / 2585 ms | 795 / 1424 / 2601 ms | 13.5 / 11.8 / 8.2 dB |
| SF11 / SF12 | 255 B | 5661 / 10174 ms | 5692 / 10238 ms | 8.0 / 5.0 dB |
| BW 500 / 250 kHz | 255 B | 111 / 223 ms | 114 / 226 ms | 13.5 / 13.0 dB |
| BW 62.5 / 41.67 / 31.25 kHz | 255 B | 892 / 1338 / 1784 ms | 898 / 1345 / 1793 ms | ~11 dB |
| BW 20.83 / 15.63 kHz | 128 / 64 B | 1570 / 1314 ms | 1581 / 1329 ms | 10.2 / 9.8 dB |
| CR 4/6, 4/7, 4/8 | 255 B | 527 / 608 / 689 ms | 531 / 612 / 693 ms | ~13 dB |
| Preamble 6 / 12 / 32 | 255 B | 442 / 454 / 495 ms | 447 / 459 / 499 ms | ~13 dB |
| 866.0625 / 866.9375 MHz | 255 B | 446 ms | 451 ms | 12 dB |
| SF12 CR4/8 | 64 B | 5521 ms | 5585 ms | 5.8 dB |
| SF7 | 17 / 64 / 128 B | 98 / 164 / 262 ms | 101 / 168 / 265 ms | ~12.5 dB |

All within 1.2% of prediction (+3…64 ms of processing) — the airtime maths and LDRO handling are right
across the whole range. CRC off and back on works.

**BW 10.42 and 7.81 kHz do not link** with these two modules: they sit −3.3 kHz apart
(the receiver reports it on every packet), beyond the ~BW/4 a LoRa demodulator tolerates.
That is crystal offset, not firmware. The safety net worked both times: both boards rolled
back to BW 125 on their own after 88 s and the link resumed.

**TX power** tracks dB for dB: −9 → +22 dBm gives −34 → −2 dBm at the receiver
(at +22 dBm side by side the front end is near saturation, as the README warns).

## Everything else

| Area | Result |
|---|---|
| Routes | 31 malformed requests → 400 and change nothing; unknown route → 404; a bad value in one field leaves the others unapplied; both pages served intact (30 / 43 KB) |
| Push that commits | provisional on both, committed after 181 s (window 180 s) |
| Push to a deaf receiver | applied unconfirmed, reverted after the 90 s window, link back |
| Local mismatch | link-down shown on both LEDs, recovers once fixed |
| Interval 0 | 2.20 exchanges/s (airtime allows 2.24), 44/44 delivered |
| 600 000 ms interval, payload 17 / 100 / 255 B, stop / start / polling | as specified |
| Remote commands | every op over LoRa: status, interval, payload, stop, start, both modes, reset, clock reset, calibrate, ratio reset, max drain on/off, push SF8 and back, WiFi off, reboot (not replayed; follow-up status shows WiFi back) |
| Command queue | 9th command refused, cancel empties it |
| Battery mode | receiver follows the mode; both show 4.13 V; CSV grows, clock reset restarts it, clear works |
| Calibration | multimeter reading, direct ratio, absurd reading refused, reset to default |
| Max drain, USB power, +22 dBm, interval 0, WiFi flood + CPU spin + white LED | 90 s, 199/199 acknowledged, no brownout, heap flat |
| Persistence | sender reboot: sequence, settings, LED level, clock kept; receiver reboot: statistics kept, gap counted as lost; USB hard reset of both: same |
| USB host not reading | each board's serial port closed for 45 s at interval 0 (board still enumerated): 0 packets lost — logging drops, never blocks |

Behaviour worth knowing (by design, now in the README): a reset drops the exchange in
flight from the sender's tallies — its "sent" and outcome together — and the discharge
clock loses at most one interval; neither steps back past a packet already sent.

## Not tested here

- **The LED's light itself.** Everything above is the firmware's trace of what it drove;
  nobody looked at the board. Press *Test colours* on either dashboard (or
  `tools/usb_bridge.py get sender "/led?test=1"`) and check red, green, blue, white.
- Current draw (no meter on the bench); whether white really out-draws other colours on
  this exact WS2812 is from the part's constant-current design, not a measurement.
- A battery discharge run, a brownout on battery, and the 3-brownout stand-down — the
  supply never sagged on USB, even under max drain.
- Running with no USB cable at all (true field condition): only "port closed, still
  enumerated" was tested. A sender unplugged and run from its cell under max drain would
  also cover battery-powered drain, the power-path switchover and hot-plug.
- The low-battery blink on real cells. It was triggered by faking the divider ratio; it is
  only as accurate as the calibration — with a ratio as far off as the README's 4.77
  example, a 3.75 V cell reads 3.4 V and blinks orange early. Calibrate first.
- The receiver's LED supply: only the sender's BAT+ was measured (4.13 V).
- Solar charging states: no panel; both boards read "Not Charging".
- The radio-fault LED pattern (needs a disconnected radio).
- Real range, and the dashboards over WiFi from a phone (the same HTTP server and routes
  were exercised over USB; the access points were up throughout).

## State left on the boards

Settings restored to what they were before testing and committed on both ends: SF7 /
BW125 / CR4/5 / preamble 8 / **+22 dBm** / 865.0625 MHz, interval 5 s, 255 B, range mode,
max drain off, LED level 24, divider ratio at the 4.333 default.

Changed by testing: sender sequence numbers advanced (~16 700), counters and discharge
clock were reset several times, and the receiver's CSV was cleared during the battery
test — its original two rows are saved in `test-results/receiver_csv_original.csv`.
