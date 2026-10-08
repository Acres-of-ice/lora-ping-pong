# LoRa range-test and battery-test rig

Two Repeater PCBs (ESP32-C3-MINI-1U + Ai-Thinker Ra-01SH SX1262, 865 MHz band).
One firmware, two roles, each serving its own dashboard over WiFi.

- **Range test** — the sender transmits numbered packets and counts
  acknowledgements; the receiver reports RSSI, SNR, delivery ratio and the rest of
  the link-quality picture.
- **Battery test** — the sender runs off a LiPo under deliberate maximum load,
  transmitting its voltage and uptime; the receiver logs them to CSV and graphs
  the discharge curve.

Radio settings (SF, bandwidth, coding rate, preamble, TX power, frequency, payload
size) change from the dashboard at runtime and persist to NVS. The sender can push
a profile to the receiver over the air, with both ends rolling back automatically
if the new profile turns out not to work.

## Wiring (Repeater PCB: ESP32-C3-MINI-1U ↔ Ra-01SH)

| Signal | Ra-01SH pin | ESP32-C3 GPIO | Note |
|---|---|---|---|
| MOSI | 14 | 7 | |
| MISO | 13 | 2 | strapping pin, R4 10k pull-up |
| SCK | 12 | 6 | |
| NSS | 15 | 10 | |
| BUSY | 10 | 1 | |
| DIO1 | 6 | 20 (U0RXD) | via solder jumper JP5 |
| RESET | 4 | 21 (U0TXD) | via solder jumper JP6 |
| DIO2, DIO3, TXEN, RXEN | 7, 8, 5, 11 | — | not connected |
| Battery divider (sender only) | — | 0 (ADC1_CH0) | R12 100k / R13 30k |

The Ra-01SH has a plain crystal (no TCXO on DIO3), runs from its LDO (no DC-DC
inductor), and steers its antenna switch internally from DIO2 — so
`CONFIG_SX126X_TCXO`, `CONFIG_SX126X_DCDC` and `CONFIG_SX126X_RFSW_ENABLE` are all
off. Defaults live in `components/sx126x/Kconfig.projbuild`.

**UART0 is the radio's.** GPIO20/21 are UART0, so the console runs over the C3's
native USB (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`, set in `sdkconfig.defaults`).
JP5 must bridge RXD0↔DIO1 and JP6 TXD0↔RESET, not the UART header.

**Battery voltage** is reported as measured at the pin. Multiply by
(100k + 30k) / 30k = **4.333** for the cell voltage — a full 4.2 V cell reads
~969 mV at the pin.

**Frequency.** The default carrier is 865.0625 MHz (IN865 channel 1), not
865.000 MHz: a carrier on the band edge puts half of a 125 kHz signal outside the
865–867 MHz band. Profiles outside the Ra-01SH's 803–930 MHz are refused.

## Build

Two builds from one tree, each with its own build directory and sdkconfig:

```sh
. $IDF_PATH/export.sh
idf.py set-target esp32c3          # once

# sender
idf.py -B build.sender   -D SDKCONFIG=sdkconfig.sender \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.sender.defaults" \
       build flash monitor

# receiver
idf.py -B build.receiver -D SDKCONFIG=sdkconfig.receiver \
       -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.receiver.defaults" \
       build flash monitor
```

Each board comes up as its own access point (`LORA-TEST-xxxx`, password
`loratest`); the dashboard is at <http://192.168.4.1/>. Switch to station mode in
`menuconfig` under *LoRa Test Rig → Network* if you would rather join a router.

> After editing any `Kconfig.projbuild`, run `idf.py -B build.<role> -D SDKCONFIG=sdkconfig.<role> reconfigure`.
> A plain `build` against an existing sdkconfig does not always pick up newly added
> symbols, and a missing `CONFIG_*` silently compiles out whatever `#if` guards it.
>
> After editing **`sdkconfig.defaults`**, delete `sdkconfig.sender` / `sdkconfig.receiver`
> and let the commands above regenerate them. That file is only consulted when an
> sdkconfig is first created — `reconfigure` will *not* retro-apply it, so changes
> appear to have worked while the old values are still compiled in.

## Layout

```
components/sx126x/     SX1262 driver: BUSY handshake, opcodes, PA config,
                       airtime and sensitivity maths
main/link.c            packet format, radio profile + NVS, rollback timer
main/sender.c          TX loop, acknowledgement handling, config push
main/receiver.c        RX loop, link statistics, battery logging
main/battery.c         ADC + divider + brownout-surviving discharge clock
main/storage.c         SPIFFS CSV log (receiver)
main/powerload.c       max-drain WiFi load (sender, battery test)
main/webserver.c       HTTP routes
main/www/              the two dashboards
```

## HTTP endpoints

Both: `/` `/status` `/log?since=N` `/radio[?sf=&bw=&cr=&pre=&pwr=&freq=]`
`/reboot?confirm=1`

Sender also: `/control?run=&interval_ms=&size=&reset=1` ·
`/pushcfg?confirm=1&sf=…` · `/mode?m=range|battery` · `/load?on=0|1` ·
`/uptime?reset=1` · `/wifi?on=0`

Receiver also: `/data.csv` · `/clear?confirm=1` · `/reset`

Everything is a plain `GET`, so it is all reachable with curl during bring-up:

```sh
curl http://192.168.4.1/status
curl "http://192.168.4.1/radio?sf=9&bw=125000"
curl -o battery.csv http://192.168.4.1/data.csv
```

## Notes worth knowing

**`/radio` is local-only; `/pushcfg` changes both ends.** `/radio` reconfigures
just the board you call it on, so the link stays down until the other end matches —
use it to recover an already-broken link. `/pushcfg` sends the profile to the
receiver and adopts it on the sender too.

A push applies on the sender whether or not the receiver confirms. That is
deliberate: an unanswered push usually means only the `CFGACK` was lost, so the
receiver has already switched and it is the sender staying behind that breaks the
link. Applying optimistically is safe because the switch is provisional either way.

**Provisional profiles.** A freshly applied profile reverts to the previous one
after a spell of silence, and only becomes permanent after carrying traffic for
twice that long. Silence-revert deliberately fires first, and that ordering is what
makes the two ends converge: if a profile happens to work in one direction only, the
receiver keeps hearing pings and would otherwise commit, while the sender — getting
no acks — reverts. With silence winning, the receiver follows the sender back instead
of stranding itself on a profile nobody is transmitting.

**Those windows are derived, not fixed** (`link_profile_silence_s()`), because one
exchange can legitimately take minutes at the slow end. Any constant short enough to
be useful at BW 125 kHz expires mid-packet down there and throws away a perfectly
good profile:

| Profile (CR4/5, pre 8, 5 s interval) | One exchange | Silence window |
|---|---|---|
| SF7, BW 125 kHz | 0.4 s | 90 s (floor) |
| SF12, BW 125 kHz | 10.2 s | 90 s (floor) |
| SF12, BW 31.25 kHz | 40.7 s | 2.3 min |
| SF12, BW 7.81 kHz | 162.8 s | 8.4 min |

The window is `max(90 s, 3 × (full-size exchange + interval))`. It is sized off a
255-byte payload whatever the configured size, so raising the payload later cannot
invalidate it. Only the sender knows the interval, so it computes the window and
ships it in the CFG frame's `rollback_s` — both ends must time out together or they
diverge, which is the one thing this mechanism exists to prevent.

**Never reconfigure the radio from an HTTP handler.** `sx126x_tx()` releases the
driver lock while waiting for TxDone, so an `sx126x_apply()` from the HTTP task
lands a `SetStandby` mid-transmission and silently kills the packet — it shows up as
`TX timeout (irq=0x0000)` with the chip back in `STDBY_RC` and no device errors set.
Handlers call `link_request_apply()`, and the radio task applies it between
exchanges via `link_apply_pending()`.

**Airtime is not free.** At SF12/BW125 a 255-byte packet is ~9.0 s in the air and
a full exchange ~10.2 s. The dashboards recompute airtime, duty cycle and
throughput as you type, before anything is committed. The same formula is
implemented twice on purpose — in C (`sx126x_airtime_ms`) and in the page's
JavaScript — so a disagreement between them is a real signal that one is wrong.

**Max drain is built to be the worst case, and it can exceed your supply.** Every
knob is turned up: 160 MHz CPU (the C3 ceiling), `-O2`, no power management or
tickless idle, a 1 kHz tick, continuous full-size WiFi broadcasts at 21 dBm with
power save off, an optional CPU spin task, and the SX1262 at +22 dBm. Set the packet
interval to **0** and LoRa transmits again the instant each acknowledgement lands,
so the PA stays keyed for most of the wall clock rather than duty-cycling.

Measured on the bench: that combination **browns out USB power** on a XIAO ESP32C3.
WiFi-only is fine; WiFi plus the LoRa PA keyed continuously for 14 s (SF12, CR4/8,
255 B) is not. For a sustained run, power the board from the battery under test — which
is the actual experiment — or a bench supply, not a USB port. If it still sags, turn
`CONFIG_POWERLOAD_CPU_SPIN` off first: it buys the least current for the most risk.

Two safeguards exist because "on by default and persisted" is otherwise a trap:

- **A boot delay** (`CONFIG_POWERLOAD_ARM_DELAY_S`, 15 s) before the load starts, so
  the dashboard is always reachable. Without it a load the supply cannot carry browns
  out before `app_main` returns, and the saved setting brings it straight back — a
  reset loop with no window to switch it off.
- **A brownout streak counter.** Three consecutive brownout resets and the load stands
  itself down and saves that, logging why. A single brownout near end of battery life
  still resumes the load, which is the behaviour the battery test wants.

The discharge clock is checkpointed to NVS, so a reset resumes the run instead of
forking the receiver's graph, and boot logs the reset reason by name (`BROWNOUT -
supply sagged`).

**Other SX1262 modules.** The driver still supports modules that need a host GPIO
for the antenna switch (the Seeed Wio-SX1262's RF_SW): enable
`CONFIG_SX126X_RFSW_ENABLE`, plus `CONFIG_SX126X_RFSW_TX_HIGH` for revisions that
use it as TX/RX select. That module also wants `CONFIG_SX126X_TCXO` and can use
`CONFIG_SX126X_DCDC`. Getting the switch wrong looks like transmit working at
arm's length and nowhere further.

**Turn the power down on the bench.** Side by side at +22 dBm the receiver front
end saturates: RSSI pins at roughly 0 dBm and SNR sits at the SX126x's ~13 dB
reporting ceiling, so neither number means anything. Drop TX power to −9 dBm for
bench work, or separate the boards, before reading anything into the signal tiles.

**What persists across a reset.** Everything you set from a dashboard is in NVS, so a
brownout mid-run resumes on the same settings instead of silently reverting to the
build defaults and bending the discharge curve for a non-battery reason:

| Setting | NVS namespace | Default |
|---|---|---|
| Radio profile (SF/BW/CR/preamble/power/frequency) | `radio` | Kconfig |
| Test mode (range / battery) | `radio` | range |
| Packet interval, payload size | `sender` | Kconfig |
| Max-drain load | `powerload` | **on** |
| Discharge clock | `calib` | 0 |

Max drain defaults to **on** and the mode switch deliberately leaves it alone — it is
an operator setting, not something a mode change should flip underneath you. A
provisionally-pushed radio profile is the one exception: it is not written until it
commits, so a reset during the provisional window returns to the previous profile.

## Troubleshooting

**`XOSC_START_ERR`, or transmit silently never completing.** The Ra-01SH uses a
plain crystal, so `CONFIG_SX126X_TCXO` is off; modules that clock from a TCXO
powered by DIO3 need it on. Getting it wrong is genuinely misleading — SPI register
reads and writes all succeed because `STDBY_RC` runs off the RC oscillator, every
configuration command is accepted, and then TX and RX simply never happen because
both need the XOSC. The symptom is a `TX timeout` with `irq=0x0000`: not even the
chip's own timeout flag set, because the chip never left standby.

`sx126x_init()` now forces `STDBY_XOSC` at boot and fails there with a named error
rather than letting this turn up later as an unexplained timeout, and it clears
device errors *before* checking them — the chip latches `XOSC_START_ERR` during its
own power-up, before DIO3 is configured, and the flag is sticky until cleared.
Every TX timeout also logs the chip mode and error flags.

**`httpd_sock_err: error in send : 11` / `error in recv : 104`.** Not the radio — no
HTTP handler ever touches the SX1262, they read mutex-guarded snapshots. `11` is
`EAGAIN` (the TCP send buffer stayed full past `send_wait_timeout`) and `104` is
`ECONNRESET` (the client hung up — page reload, phone WiFi power-save, walking out of
AP range). Both are logged and recovered; the next poll reconnects.

Expect them while max drain is running: it saturates the WiFi TX path by design.
Throttling it to quieten the log would corrupt the measurement, so the log noise is
the right trade. The `/log` responses are batched into 1 KB writes rather than one
per entry, which keeps this to a minimum.

**Verified working:** SF7/BW125/CR4-5, 255-byte payload, both boards on the bench —
measured RTT 465 ms against 451 ms of predicted airtime, no sequence gaps.
