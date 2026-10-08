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
if the new profile turns out not to work. The receiver's dashboard can also drive
the sender over LoRa, so the whole rig runs from one phone on `lora-recv`.

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
| Solar charger CHRG (CN3791) | — | 5 | open-drain, low while charging |
| Solar charger DONE (CN3791) | — | 3 | open-drain, low once charged |
| Battery divider (sender only) | — | 0 (ADC1_CH0) | R12 100k / R13 30k |
| Status LED (WS2812, U8) DIN | — | 4 | VDD is BAT+, not 3V3 — see *Status LED* |

The Ra-01SH has a plain crystal (no TCXO), runs the SX1262 from its internal LDO
(no DC-DC inductor), and keeps its antenna switch inside the module, driven by the
chip's DIO2. The driver is built for exactly that, so none of it is configurable;
only the pin numbers are, in `components/sx126x/Kconfig.projbuild`.

**UART0 is the radio's.** GPIO20/21 are UART0, so the console runs over the C3's
native USB (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`, set in `sdkconfig.defaults`).
JP5 must bridge RXD0↔DIO1 and JP6 TXD0↔RESET, not the UART header.

**Battery voltage.** Both dashboards show the sense-pin reading and the battery
voltage, which is that times the divider ratio. The ratio starts at the nominal
(100k + 30k) / 30k = 4.333, but resistor tolerance and ADC gain error move it a long
way: one board read 873 mV at the pin with 4.16 V on the cell, a ratio of 4.77.
Calibrate it: measure the battery with a multimeter, enter the reading on the sender
dashboard (or under *Sender control* on the receiver's) and press **Calibrate**. The ratio is saved on the sender and sent with every
battery sample, so the receiver shows the same voltage. The CSV keeps the raw pin
reading, so a later recalibration still converts the whole run.

**Solar charging.** Each board reads its own CN3791's CHRG and DONE pins and reports
the result to the other: the sender in every ping and battery packet, the receiver in
every acknowledgement. Both dashboards show both boards, and the receiver's packet log
records the sender's state per packet, so a battery run shows exactly when the cell was
being topped up.

**Frequency.** The default carrier is 865.0625 MHz (IN865 channel 1), not
865.000 MHz: a carrier on the band edge puts half of a 125 kHz signal outside the
865–867 MHz band. Anything the SX1262 tunes (150–960 MHz) is accepted, but the
Ra-01SH's antenna matching is built for 803–930 MHz, so expect far less range
outside it.

## Status LED

The WS2812 (U8) shows what the board is doing without a phone. Status runs dim — a
tenth of full by default — and is lit for tens of milliseconds at a time, so the
indicator costs well under a milliamp on average and does not bend a battery run.
One task owns the LED and shows the most important thing going on:

| Priority | Pattern | Meaning |
|---|---|---|
| 1 | Red, 5 blinks a second | The radio never came up (wiring, or 3V3 at the module) |
| 2 | **Solid white, full brightness** | Max drain is running (sender) |
| 3 | White, 2 blinks a second | Max drain is on and starts when the boot delay ends |
| 4 | Short flashes, one per event | See below |
| 5 | A blink every few seconds | The background state, see below |

**Max drain is solid white at 255 on purpose.** A WS2812 sinks a constant current per
die, so all three dies fully on is the most it can draw (roughly 35–60 mA, by part);
no other colour comes close. It ignores the brightness setting, no packet flash
interrupts it, and it follows the load exactly: off during the boot delay, off when
the brownout streak stands the load down, off with WiFi.

Flashes, at the status brightness:

| Flash | Sender | Receiver |
|---|---|---|
| Blue, 20 ms | a packet went out | — |
| Green / yellow-green / amber / orange, 60 ms | acknowledged; colour is the weaker direction's SNR margin, ≥10 / 5–10 / 0–5 / <0 dB | a packet arrived; colour is its SNR margin |
| Red, 200 ms | no acknowledgement | packets went missing before this one |
| Red ×2 | the radio refused to transmit | the radio refused to send the ACK |
| Pink, 30 ms | — | a frame failed CRC / header check |
| Violet ×2 | carried out a command from the receiver | the sender confirmed a command |
| Cyan ×3 | a new profile went live (provisional) | accepted a pushed profile |
| Red then cyan | the provisional profile rolled back | same |
| Green then cyan | the provisional profile was saved | same |

The margin colours make a walk-test readable at a glance: green is comfortable,
orange is a link about to drop.

Background, shown only after a full period with nothing else lighting the LED — so a
link passing packets shows its packets, not a heartbeat on top:

| Background | Sender | Receiver |
|---|---|---|
| Red, every 2 s | link down: 3 exchanges in a row unanswered | link down: silent for 3× its observed cadence (at least 20 s) |
| Violet, every 1 s | — | commands waiting to go to the sender |
| Cyan ×2, every 2 s | profile still provisional | same |
| Orange ×3, every 4 s | battery below `CONFIG_LED_LOW_BATT_MV` (3.4 V) — calibrate the divider first, or it fires early | — |
| Amber, every 3 s | stopped (still checking in) | the sender is stopped |
| Blue, every 3 s | — | listening; nothing heard since boot |
| Green blip, every 5 s | idle and healthy | idle and healthy |

Both dashboards have a *Status LED* panel: the pattern showing now, the brightness
(0–255, saved; 0 turns status off but never hides the fault or max-drain patterns),
a colour test, and this legend. `/led?test=1` runs red, green, blue and white at full
brightness for 400 ms each — the quickest check that all three dies work.

**The LED runs off BAT+.** With no cell fitted it stays dark however correct the
firmware is, and it keeps its last colour through an ESP reset (including max drain's
white — init writes black first thing). Below about 3.5 V on BAT+ the green and blue
dies fade first. `/led?trace=1` returns the last 48 colour changes with timestamps,
which is how the patterns and timings were verified without anyone watching.

## Bench work over USB

Every dashboard route also works over the USB serial port: type `/status` (or any
route) on the console and the board answers it from its own web server over loopback,
so it behaves exactly as it does over WiFi. That matters on a laptop whose only
internet is its WiFi — joining `lora-send`/`lora-recv` would cut it off.

`tools/usb_bridge.py` holds both ports open and serves each real dashboard on
localhost, over USB:

```sh
tools/usb_bridge.py serve --sender   /dev/serial/by-id/usb-Espressif_…_3A:24-if00 \
                          --receiver /dev/serial/by-id/usb-Espressif_…_3A:B8-if00
# sender   -> http://127.0.0.1:8081/
# receiver -> http://127.0.0.1:8082/
tools/usb_bridge.py get receiver /status         # one-off, from another shell
```

Use the `/dev/serial/by-id/` paths: `ttyACM` numbers shuffle with plug order, and on
a laptop with a WWAN modem one of them is the modem. Keep the bridge running rather
than reopening ports — opening an ESP32-C3's USB serial port with the wrong DTR/RTS
sequence resets the board. `release`/`attach` free a port for flashing meanwhile.

`tools/rig_test.py` is the hardware test campaign built on the bridge (`--list` for
the tests, `soak --soak-min 30` for a long run). It writes `test-results/<time>/`.

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

Each board comes up as its own open access point, no password: `lora-send` on the
sender, `lora-recv` on the receiver. The dashboard is at <http://192.168.4.1/>.
Anyone in range can join and use it, so set `CONFIG_NET_AP_PASSWORD` (8–63
characters, WPA2) under *LoRa Test Rig → Network* if that matters. Station mode is
there too if you would rather join a router.

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
main/sender.c          TX loop, acknowledgement handling, config push,
                       commands from the receiver
main/receiver.c        RX loop, link statistics, battery logging, command
                       queue for the sender
main/battery.c         ADC + divider + brownout-surviving discharge clock
main/storage.c         SPIFFS CSV log (receiver)
main/powerload.c       max-drain WiFi load (sender, battery test)
main/webserver.c       HTTP routes
main/led.c             WS2812 status LED: RMT driver, patterns, priorities
main/console.c         dashboard routes over the USB serial console
main/www/              the two dashboards
tools/usb_bridge.py    both dashboards on localhost over USB
tools/rig_test.py      hardware test campaign
```

## HTTP endpoints

Both: `/` `/status` `/log?since=N` `/radio[?sf=&bw=&cr=&pre=&pwr=&freq=]`
`/reboot?confirm=1` · `/led[?level=0..255|test=1|trace=1]`

`/status` also reports `boot_s`, `reset_reason` and free / minimum heap, so a long
run shows a brownout or a leak without a serial cable.

Sender also: `/control?run=&interval_ms=&size=&reset=1` ·
`/pushcfg?confirm=1&sf=…` · `/mode?m=range|battery` · `/load?on=0|1` ·
`/uptime?reset=1` · `/battery[?actual_mv=|ratio_x1000=|reset=1]` · `/wifi?on=0`

Receiver also: `/data.csv` · `/clear?confirm=1` · `/reset` ·
`/sendercmd?op=…` (see below)

Everything is a plain `GET`, so it is all reachable with curl during bring-up:

```sh
curl http://192.168.4.1/status
curl "http://192.168.4.1/radio?sf=9&bw=125000"
curl -o battery.csv http://192.168.4.1/data.csv
```

## Driving the sender from the receiver

The receiver's **Sender control** panel does everything the sender's own dashboard
does: start/stop, range or battery test, interval, payload size, reset counters, max
drain, reset discharge clock, calibrate / reset the divider ratio, WiFi off and
reboot. **Push to both** in its radio-profile panel changes the profile on both
boards. Flash **both** boards with this firmware: it is link protocol v6, and the
two ends ignore each other's frames until their versions match.

The link is half-duplex and the sender leads: it only listens right after its own
transmissions. So a command is queued on the receiver (up to 8) and rides on the
acknowledgement for the sender's next packet. It arrives within one packet interval
while the sender runs; a stopped sender checks in every 5 s for exactly this reason
(less often at slow profiles: it never spends more than a tenth of the time on air
doing it). After acting on a command the sender asks for the next one at once, so a
queue of them drains in seconds, not one per interval.

The sender acts on each command once, and the ids are saved on both boards, so
neither a lost answer nor a reboot repeats one. Its answer carries its settings
back: the panel shows them, the receiver follows its test mode, and a calibration
shows on the receiver's battery card straight away. **Cancel waiting** drops what has
not gone out yet; a command already delivered may have been carried out.

**Push to both** goes to the sender as a command, and the sender then pushes the
profile back exactly as its own **Push to receiver** does: provisionally, with both
ends reverting if it carries nothing.

The same thing over HTTP, one command per request:

```
/sendercmd?op=status | run&on=0|1 | mode&m=range|battery | interval&ms=0..600000
          | payload&size=17..255 | reset | load&on=0|1 | clock_reset
          | calibrate&actual_mv=500..30000 | ratio_reset | wifi_off&confirm=1
          | reboot&confirm=1 | push&confirm=1&sf=&bw=&cr=&pre=&pwr=&freq= | cancel
```

Nothing authenticates a command, on the air or over the open access point: anyone
on `lora-recv` can stop or reboot the sender.

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

The load is off on a freshly flashed board and starts from the dashboard's
**Start max drain** button. Its on/off state is persisted, so once on it survives
resets, and two safeguards exist because a persisted load is otherwise a trap:

- **A boot delay** (`CONFIG_POWERLOAD_ARM_DELAY_S`, 15 s) before the load starts, so
  the dashboard is always reachable. Without it a load the supply cannot carry browns
  out before `app_main` returns, and the saved setting brings it straight back — a
  reset loop with no window to switch it off.
- **A brownout streak counter.** Three consecutive brownout resets and the load stands
  itself down and saves that, logging why. A single brownout near end of battery life
  still resumes the load, which is the behaviour the battery test wants.

The run itself survives a reset, and boot logs the reset reason by name (`BROWNOUT -
supply sagged`). The sender saves its sequence number, counters and discharge clock
before every transmit, and the receiver saves its delivery statistics with every
packet. After a reset the sender carries on from the next sequence number rather than
from 1, and the clock never steps back past a logged sample. Either would otherwise
lay the new packets over the old ones in every graph. These per-packet saves go to their
own `runstate` partition, so their flash wear stays away from the settings in `nvs`.

**Other SX1262 modules.** The driver is fixed to the Ra-01SH's hardware: crystal,
LDO, antenna switch on DIO2. A module with a TCXO powered from DIO3, a DC-DC
inductor, or an antenna switch the host must drive (the Seeed Wio-SX1262 has all
three) needs that support back; the driver at commit `b395066` has options for all
of it. Getting the switch wrong looks like transmit working at arm's length and
nowhere further.

**Turn the power down on the bench.** Side by side at +22 dBm the receiver front
end saturates: RSSI pins at roughly 0 dBm and SNR sits at the SX126x's ~13 dB
reporting ceiling, so neither number means anything. Drop TX power to −9 dBm for
bench work, or separate the boards, before reading anything into the signal tiles.

**What persists across a reset.** Everything you set from a dashboard is in NVS, so a
brownout mid-run resumes on the same settings instead of silently reverting to the
build defaults and bending the discharge curve for a non-battery reason:

| Setting | Where | Default |
|---|---|---|
| Radio profile (SF/BW/CR/preamble/power/frequency) | `nvs`: `radio` | Kconfig |
| Test mode (range / battery) | `nvs`: `radio` | range |
| Packet interval, payload size | `nvs`: `sender` | Kconfig |
| Max-drain load | `nvs`: `powerload` | off |
| Battery divider ratio (sender; the receiver keeps the last one it was sent) | `nvs`: `calib` / `receiver` | 4.333 |
| Sequence number, counters, discharge clock (sender) | `runstate` partition | 1 / 0 / 0 |
| Delivery statistics, CRC and header error counts (receiver) | `runstate` partition | 0 |
| Remote-command ids (receiver's next, sender's last) | `runstate` partition | random / none |
| Status LED brightness | `nvs`: `led` | `CONFIG_LED_LEVEL` (24) |

Because the save happens before each transmit, a reset drops the exchange in flight
from the sender's tallies — its "sent" and its outcome together, so sent = acked +
missed still holds — and the discharge clock resumes from the last packet, losing at
most one interval of on-time. Neither ever steps back past a packet already sent.

**Reset counters** zeroes the counts, and the saved copies with them, but the sequence
number keeps counting so graphs keyed by it keep moving forward. **Reset discharge
clock** starts a new run: the receiver's graph plots from there on, while its CSV
keeps the earlier runs.

The mode switch deliberately leaves max drain alone — it is an operator setting, not
something a mode change should flip underneath you. A
provisionally-pushed radio profile is the one exception: it is not written until it
commits, so a reset during the provisional window returns to the previous profile.
A local `/radio` change ends a provisional window on that board: it is an explicit
choice, so it is saved at once and nothing reverts it.

## Troubleshooting

**`XOSC_START_ERR`, or transmit silently never completing.** The radio's 32 MHz
crystal did not start. The Ra-01SH's crystal needs no setup from the host, so there
is no setting to get wrong: suspect the 3V3 supply at the module, then the module.
The failure is genuinely misleading — SPI register reads and writes all succeed
because `STDBY_RC` runs off the RC oscillator, every configuration command is
accepted, and then TX and RX simply never happen because both need the crystal.
The symptom is a `TX timeout` with `irq=0x0000`: not even the chip's own timeout
flag set, because the chip never left standby.

`sx126x_init()` forces `STDBY_XOSC` at boot and fails there with a named error
rather than letting this turn up later as an unexplained timeout. It clears device
errors *before* checking them, because the flags are sticky and one latched during
the chip's own power-up would otherwise read as a fresh failure. Every TX timeout
also logs the chip mode and error flags.

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
