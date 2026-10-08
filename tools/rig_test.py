#!/usr/bin/env python3
"""Hardware test campaign for the two-board rig.

Drives both boards through tools/usb_bridge.py (start that first) and checks what
they actually do: routes and their validation, the radio-profile matrix with RTT
against predicted airtime, pushes that commit and pushes that roll back, every
remote command, battery mode and the CSV, max drain and its LED, persistence
across resets, WiFi off, the status LED's patterns, and a soak.

    tools/rig_test.py                    # everything except the soak
    tools/rig_test.py routes led         # just these groups
    tools/rig_test.py soak --soak-min 30
    tools/rig_test.py --list

Boards side by side: the bench profile runs at -9 dBm so the receiver is not
saturated. Each group starts from that baseline and the run ends by restoring the
settings found at the start (profile, interval, payload, mode, LED level, load).

Writes test-results/<time>/report.md and results.json.
"""

import argparse
import json
import math
import os
import sys
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from usb_bridge import client  # noqa: E402

S, R = "sender", "receiver"
BENCH = dict(sf=7, bw=125000, cr=5, pre=8, pwr=-9, freq=865062500)
BENCH_INTERVAL = 1000
LINK_ACK_LEN = 14             # sizeof(link_ack_t)
LINK_ACK_MAX_LEN = 14 + 21    # an ACK carrying a command
SILENCE_MIN_S = 90


# ------------------------------------------------------------------ helpers ----

def airtime_ms(sf, bw_hz, cr, pre, crc, n):
    """Mirror of sx126x_airtime_ms()."""
    tsym = (1 << sf) * 1000.0 / bw_hz
    de = 1 if tsym > 16 else 0
    n_pre = pre + (6.25 if sf < 7 else 4.25)
    den = 4 * sf if sf < 7 else 4 * (sf - 2 * de)
    num = 8 * n + 16 * crc - 4 * sf + (0 if sf < 7 else 8) + 20
    blocks = max(0, int(math.ceil(num / den)))
    return (n_pre + 8 + blocks * cr) * tsym


def silence_window_s(cfg, interval_ms):
    """Mirror of link_profile_silence_s()."""
    ex = airtime_ms(cfg["sf"], cfg["bw"], cfg["cr"], cfg["pre"], 1, 255) + \
        airtime_ms(cfg["sf"], cfg["bw"], cfg["cr"], cfg["pre"], 1, LINK_ACK_LEN)
    return max(SILENCE_MIN_S, int((ex + interval_ms) * 3 / 1000) + 1)


def scaled(rgb, level=24):
    """A full-scale colour as the LED shows it at a status level (led.c scale())."""
    v = int(rgb[1:], 16)
    c = [(v >> 16) & 255, (v >> 8) & 255, v & 255]
    return "#" + "".join(f"{(x * level + 127) // 255:02x}" for x in c)


RED, GREEN, BLUE, CYAN = scaled("#ff0000"), scaled("#00ff00"), scaled("#0000ff"), scaled("#00ffff")
VIOLET = scaled("#a000ff")


def flashes(trace):
    """(colour, ms lit, pattern) for every lit span in an LED trace."""
    out = []
    for a, b in zip(trace, trace[1:]):
        if a[1] != "#000000":
            out.append((a[1], b[0] - a[0], a[2]))
    return out


def has_sequence(fl, steps, tol=25):
    """True when consecutive flashes match [(colour, ms), ...] within tol ms."""
    for i in range(len(fl) - len(steps) + 1):
        if all(fl[i + k][0] == c and abs(fl[i + k][1] - ms) <= tol for k, (c, ms) in enumerate(steps)):
            return True
    return False


class Fail(Exception):
    pass


class Rig:
    def __init__(self, outdir):
        self.outdir = outdir
        self.notes = []

    # --- raw access
    def req(self, board, path, timeout=20):
        r = client({"op": "get", "board": board, "path": path, "timeout": timeout})
        if "error" in r:
            raise Fail(f"{board} {path}: {r['error']}")
        return r["status"], r["body"]

    def get(self, board, path, expect=200, timeout=20):
        st, body = self.req(board, path, timeout)
        if st != expect:
            raise Fail(f"{board} {path}: HTTP {st}, wanted {expect}: {body[:160]}")
        if st == 200 and body[:1] in "{[":
            return json.loads(body)
        return body

    def status(self, board):
        return self.get(board, "/status")

    def tail(self, board, since):
        return client({"op": "tail", "board": board, "since": since})

    def log_has(self, board, since, text):
        return any(text in l for _, l in self.tail(board, since)["lines"])

    # --- waiting
    def wait(self, fn, timeout, what, poll=1.0):
        end = time.time() + timeout
        last = None
        while time.time() < end:
            try:
                v = fn()
            except Fail as e:
                last = e
                v = None
            if v:
                return v
            time.sleep(poll)
        raise Fail(f"timed out after {timeout:.0f}s waiting for {what}" +
                   (f" (last error: {last})" if last else ""))

    def note(self, msg):
        self.notes.append(msg)
        print(f"      · {msg}", flush=True)

    def expect(self, cond, msg):
        if not cond:
            raise Fail(msg)

    # --- the rig
    def cfg_now(self, board):
        r = self.get(board, "/radio")
        return dict(sf=r["sf"], bw=r["bw_hz"], cr=r["cr"], pre=r["preamble"],
                    pwr=r["tx_dbm"], freq=r["freq_hz"])

    def acks(self):
        return self.status(S)["ack"]

    def wait_acks(self, n, timeout, start=None):
        """Wait for n more acknowledged exchanges. Returns the new ack count."""
        base = self.acks() if start is None else start
        return self.wait(lambda: (a := self.acks()) >= base + n and a, timeout,
                         f"{n} acknowledged packets")

    def control(self, **kw):
        q = "&".join(f"{k}={v}" for k, v in kw.items())
        return self.get(S, f"/control?{q}")

    def push(self, cfg, timeout=90):
        """Push a profile from the sender and wait for the handshake to finish."""
        q = "&".join(f"{k}={v}" for k, v in cfg.items())
        self.get(S, f"/pushcfg?confirm=1&{q}")
        time.sleep(0.3)
        st = self.wait(lambda: (s := self.status(S)) and not s["push_busy"] and s,
                       timeout, "the push handshake")
        return st["push_state"]

    def local_radio(self, board, cfg):
        q = "&".join(f"{k}={v}" for k, v in cfg.items())
        self.get(board, f"/radio?{q}")
        self.wait(lambda: not self.status(board)["apply_pending"], 60, "local apply")

    def same(self, a, b):
        return all(a[k] == b[k] for k in ("sf", "bw", "cr", "pre", "pwr", "freq"))

    def link_ok(self, timeout=30):
        try:
            self.wait_acks(2, timeout)
            return True
        except Fail:
            return False

    def baseline(self, interval=BENCH_INTERVAL, payload=255):
        """Bench profile on both ends, committed, link carrying traffic."""
        s_cfg, r_cfg = self.cfg_now(S), self.cfg_now(R)
        st = self.status(S)
        for b in (S, R):
            if self.status(b)["led_level"] != 24:
                self.get(b, "/led?level=24")
        if st["load"]:
            self.get(S, "/load?on=0")
        if st["mode"] != "range":
            self.get(S, "/mode?m=range")
        self.control(run=1, interval_ms=interval, size=payload)
        if not (self.same(s_cfg, BENCH) and self.same(r_cfg, BENCH)):
            if self.same(s_cfg, r_cfg) and self.link_ok(30):
                self.push(BENCH)
            else:
                # Ends disagree or the link is dead: set both locally, which is
                # what /radio is for.
                self.local_radio(S, BENCH)
                self.local_radio(R, BENCH)
        # A provisional profile must not expire mid-test.
        for b in (S, R):
            if self.status(b)["rollback_pending"]:
                self.local_radio(b, BENCH)  # /radio persists, so it is committed
        self.expect(self.link_ok(40), "baseline: no traffic on the bench profile")


# ------------------------------------------------------------------- tests ----

TESTS = []


def test(group, slow=False):
    def deco(fn):
        TESTS.append((group, fn.__name__, fn, slow))
        return fn
    return deco


# --- routes ----------------------------------------------------------------

@test("routes")
def both_boards_answer(rig):
    s, r = rig.status(S), rig.status(R)
    rig.expect(s["role"] == "sender" and r["role"] == "receiver", "roles wrong")
    rig.note(f"sender ssid {s['ssid']}, receiver ssid {r['ssid']}, "
             f"heap {s['heap_free']}/{r['heap_free']} B")
    for b in (S, R):
        for path in ("/radio", "/log?since=0", "/led"):
            rig.get(b, path)


@test("routes")
def dashboard_pages_intact(rig):
    for b, title in ((S, "LoRa Sender"), (R, "LoRa Receiver")):
        page = rig.get(b, "/")
        rig.expect(f"<title>{title}</title>" in page and page.rstrip().endswith("</html>"),
                   f"{b} page truncated or wrong ({len(page)} B)")
        rig.expect("Status LED" in page, f"{b} page lacks the LED panel")
        rig.note(f"{b} page {len(page)} B intact")


@test("routes")
def bad_input_rejected(rig):
    cases = [
        (S, "/radio?sf=4"), (S, "/radio?sf=13"), (S, "/radio?bw=12345"),
        (S, "/radio?cr=9"), (S, "/radio?pwr=23"), (S, "/radio?pwr=-10"),
        (S, "/radio?freq=100"), (S, "/radio?pre=0"), (R, "/radio?sf=13"),
        (S, "/control?interval_ms=-1"), (S, "/control?interval_ms=600001"),
        (S, "/control?size=16"), (S, "/control?size=256"),
        (S, "/pushcfg?sf=9"), (S, "/reboot"), (R, "/reboot"),
        (S, "/battery?actual_mv=100"), (S, "/battery?ratio_x1000=999"),
        (S, "/wifi?on=1"), (S, "/led?level=256"), (R, "/led?level=-1"),
        (R, "/sendercmd"), (R, "/sendercmd?op=bogus"),
        (R, "/sendercmd?op=interval&ms=700000"), (R, "/sendercmd?op=payload&size=10"),
        (R, "/sendercmd?op=wifi_off"), (R, "/sendercmd?op=reboot"),
        (R, "/sendercmd?op=mode&m=x"), (R, "/sendercmd?op=calibrate&actual_mv=1"),
        (R, "/sendercmd?op=push&sf=9"), (R, "/clear"),
    ]
    before = rig.cfg_now(S), rig.status(S)
    for b, path in cases:
        st, body = rig.req(b, path)
        rig.expect(st == 400, f"{b} {path}: HTTP {st}, wanted 400 ({body[:80]})")
    st, _ = rig.req(S, "/nope")
    rig.expect(st == 404, f"/nope gave {st}")
    after = rig.cfg_now(S), rig.status(S)
    rig.expect(rig.same(before[0], after[0]) and before[1]["interval_ms"] == after[1]["interval_ms"]
               and before[1]["payload_len"] == after[1]["payload_len"],
               "a rejected request changed something")
    rig.note(f"{len(cases)} bad requests -> 400, unknown route -> 404, nothing changed")


@test("routes")
def control_validates_before_applying(rig):
    st = rig.status(S)
    code, _ = rig.req(S, "/control?interval_ms=2000&size=999")
    rig.expect(code == 400, "bad size accepted")
    rig.expect(rig.status(S)["interval_ms"] == st["interval_ms"],
               "interval applied although the same request had a bad size")


@test("routes")
def console_and_bridge_speed(rig):
    t = []
    for _ in range(10):
        t0 = time.time()
        rig.get(S, "/led")
        t.append((time.time() - t0) * 1000)
    t0 = time.time()
    rig.get(R, "/")
    page = (time.time() - t0) * 1000
    rig.note(f"/led median {sorted(t)[5]:.0f} ms; receiver page {page:.0f} ms")
    rig.expect(sorted(t)[5] < 100, "routes over USB are slow again")


# --- link -------------------------------------------------------------------

@test("link")
def bench_baseline_and_rtt(rig):
    rig.baseline()
    rig.get(S, "/control?reset=1")
    rig.get(R, "/reset")
    time.sleep(1)
    rig.wait_acks(10, 60)
    log = rig.get(S, "/log?since=0")["entries"]
    acked = [e for e in log if e["acked"]]
    rig.expect(len(acked) >= 10, "too few acknowledged")
    pred = airtime_ms(7, 125000, 5, 8, 1, 255) + airtime_ms(7, 125000, 5, 8, 1, LINK_ACK_LEN)
    rtt = sorted(e["rtt_ms"] for e in acked)[len(acked) // 2]
    rig.note(f"SF7/BW125 255 B: median RTT {rtt} ms, predicted airtime {pred:.0f} ms "
             f"(+{rtt - pred:.0f} ms processing)")
    rig.expect(pred <= rtt <= pred * 1.10 + 20, "RTT out of line with predicted airtime")
    r = rig.status(R)
    rig.note(f"receiver: rssi {r['rssi']} dBm, snr {r['snr']} dB, pdr {r['pdr']}%, "
             f"noise floor {r['noise_floor']} dBm")
    rig.expect(r["pdr"] == 100.0 and r["lost"] == 0, f"losses at the bench: {r['lost']}")


@test("link", slow=True)
def push_commits_after_window(rig):
    rig.baseline()
    cfg = dict(BENCH, cr=6)
    state = rig.push(cfg)
    rig.expect("both" in state, f"push not confirmed: {state}")
    win = silence_window_s(cfg, BENCH_INTERVAL)
    for b in (S, R):
        st = rig.status(b)
        rig.expect(st["rollback_pending"] and st["cr"] == 6, f"{b} not provisional on CR4/6")
        rig.expect(st["led"] in ("provisional profile", "event"), f"{b} LED {st['led']}")
    rig.note(f"both provisional; silence window {win}s, commit after {2 * win}s")
    t0 = time.time()
    rig.wait(lambda: not rig.status(S)["rollback_pending"] and not rig.status(R)["rollback_pending"],
             2 * win + 30, "commit", poll=5)
    rig.note(f"committed on both after {time.time() - t0:.0f}s")
    for b in (S, R):
        rig.expect(rig.cfg_now(b)["cr"] == 6, f"{b} reverted instead of committing")
        fl = flashes(rig.get(b, "/led?trace=1")["trace"])
        rig.expect(has_sequence(fl, [(GREEN, 150), (CYAN, 150)]),
                   f"{b}: no green-cyan commit flash in the LED trace")
    rig.push(BENCH)


@test("link")
def profile_matrix(rig):
    rig.baseline()
    rows = []
    # (name, cfg overrides, payload); slow ones get a shorter payload.
    matrix = [
        ("SF5", dict(sf=5), 255), ("SF6", dict(sf=6), 255), ("SF8", dict(sf=8), 255),
        ("SF9", dict(sf=9), 255), ("SF10", dict(sf=10), 255), ("SF11", dict(sf=11), 255),
        ("SF12", dict(sf=12), 255),
        ("BW250", dict(bw=250000, freq=865125000), 255),
        ("BW500", dict(bw=500000, freq=865250000), 255),
        ("BW62.5", dict(bw=62500), 255), ("BW41.67", dict(bw=41670), 255),
        ("BW31.25", dict(bw=31250), 255), ("BW20.83", dict(bw=20830), 128),
        ("BW15.63", dict(bw=15630), 64),
        ("CR4/6", dict(cr=6), 255), ("CR4/7", dict(cr=7), 255), ("CR4/8", dict(cr=8), 255),
        ("pre6", dict(pre=6), 255), ("pre12", dict(pre=12), 255), ("pre32", dict(pre=32), 255),
        ("866.0625", dict(freq=866062500), 255), ("866.9375", dict(freq=866937500), 255),
        ("SF12 CR4/8 64B", dict(sf=12, cr=8), 64),
        ("SF7 17B", dict(), 17), ("SF7 64B", dict(), 64), ("SF7 128B", dict(), 128),
    ]
    for name, over, payload in matrix:
        cfg = dict(BENCH, **over)
        rig.control(size=payload)
        state = rig.push(cfg, timeout=180)
        pred = airtime_ms(cfg["sf"], cfg["bw"], cfg["cr"], cfg["pre"], 1, payload) + \
            airtime_ms(cfg["sf"], cfg["bw"], cfg["cr"], cfg["pre"], 1, LINK_ACK_LEN)
        cursor = rig.get(S, "/log?since=999999999")["next"]
        n = 3
        try:
            rig.wait_acks(n, n * (pred / 1000 + BENCH_INTERVAL / 1000) * 2 + 20)
            ents = [e for e in rig.get(S, f"/log?since={cursor}")["entries"] if e["acked"]]
            rtt = sorted(e["rtt_ms"] for e in ents)[len(ents) // 2] if ents else -1
            r = rig.status(R)
            ok = ents and pred <= rtt <= pred * 1.10 + 30
            rows.append((name, payload, round(pred), rtt, r["rssi"], r["snr"], "ok" if ok else "RTT?"))
        except Fail:
            rows.append((name, payload, round(pred), "-", "-", "-", "NO LINK"))
        rig.note(f"{name:>14} {payload:3d} B  pred {pred:7.0f} ms  " +
                 f"rtt {rows[-1][3]}  rssi {rows[-1][4]}  snr {rows[-1][5]}  {rows[-1][6]}")
        if rows[-1][6] == "NO LINK":
            # Expected below ~BW/4 of carrier offset. Let the rollback bring both back.
            win = silence_window_s(cfg, BENCH_INTERVAL)
            rig.wait(lambda: not rig.status(S)["rollback_pending"]
                     and not rig.status(R)["rollback_pending"], win + 60,
                     "rollback after a dead profile", poll=5)
            rig.expect(rig.link_ok(60), f"{name}: link did not come back after rollback")
            rig.note(f"{name}: rolled back on both, link restored")
    rig.control(size=255)
    rig.push(BENCH)
    rig.matrix = rows
    bad = [r for r in rows if r[6] == "RTT?"]
    rig.expect(not bad, f"RTT out of line for {[r[0] for r in bad]}")
    dead = [r[0] for r in rows if r[6] == "NO LINK"]
    if dead:
        rig.note(f"no link (expected for narrow bandwidths at this crystal offset): {dead}")
    rig.expect(all(r[6] == "ok" for r in rows if not r[0].startswith("BW1")),
               "a profile other than the narrowest bandwidths failed")


@test("link", slow=True)
def narrowest_bandwidths(rig):
    """BW 10.42 and 7.81 kHz: tolerate only about BW/4 of carrier offset, and these
    two radios sit about 3.4 kHz apart. Either they work, or both ends must roll
    back on their own and the link come back - nothing may be left stranded."""
    rig.baseline()
    rig.control(size=64)
    for bw in (10420, 7810):
        cfg = dict(BENCH, bw=bw)
        state = rig.push(cfg, timeout=120)
        ferr = rig.get(R, "/log?since=0")["entries"][-1:] or [{}]
        pred = airtime_ms(7, bw, 5, 8, 1, 64) + airtime_ms(7, bw, 5, 8, 1, LINK_ACK_LEN)
        try:
            rig.wait_acks(2, 2 * (pred / 1000 + 1) + 20)
            rig.note(f"BW {bw / 1000:.2f} kHz works ({state})")
        except Fail:
            win = silence_window_s(cfg, BENCH_INTERVAL)
            t0 = time.time()
            rig.wait(lambda: not rig.status(S)["rollback_pending"]
                     and not rig.status(R)["rollback_pending"], win + 60,
                     f"rollback from BW {bw}", poll=5)
            rig.expect(rig.cfg_now(S)["bw"] == 125000 and rig.cfg_now(R)["bw"] == 125000,
                       "did not roll back to BW 125 kHz on both")
            rig.expect(rig.link_ok(60), "link not back after the rollback")
            rig.note(f"BW {bw / 1000:.2f} kHz: no link (offset {ferr[0].get('freq_err', '?')} Hz "
                     f"at BW 125); both rolled back to BW 125 after {time.time() - t0 + 20:.0f}s "
                     "and the link resumed")
            fl = flashes(rig.get(R, "/led?trace=1")["trace"])
            rig.expect(has_sequence(fl, [(RED, 150), (CYAN, 150)]), "receiver: no revert flash")
    rig.control(size=255)


@test("link")
def power_sweep(rig):
    rig.baseline()
    out = []
    for p in (-9, -3, 0, 5, 10, 14, 17, 20, 22):
        rig.push(dict(BENCH, pwr=p))
        rig.wait_acks(3, 30)
        time.sleep(0.5)
        r = rig.status(R)
        out.append((p, r["rssi"], r["signal_rssi"], r["snr"]))
    rig.push(BENCH)
    rig.note("TX dBm -> receiver RSSI / signal RSSI / SNR: " +
             ", ".join(f"{p:+d}->{a:.0f}/{b:.0f}/{c:.1f}" for p, a, b, c in out))
    rig.expect(out[-1][1] > out[0][1] + 5, "RSSI did not rise with TX power")


@test("link")
def crc_off_and_on(rig):
    rig.baseline()
    rig.push(dict(BENCH, crc=0))
    rig.expect(rig.status(S)["crc"] is False and rig.status(R)["crc"] is False, "CRC still on")
    rig.wait_acks(3, 30)
    rig.push(dict(BENCH, crc=1))
    rig.expect(rig.status(S)["crc"] is True and rig.status(R)["crc"] is True, "CRC still off")
    rig.wait_acks(3, 30)


@test("link")
def local_mismatch_then_recover(rig):
    rig.baseline()
    rig.local_radio(R, dict(BENCH, sf=8))  # receiver deaf to the sender
    t0 = time.time()
    rig.wait(lambda: rig.status(S)["consec_miss"] >= 3, 30, "sender to see 3 misses")
    rig.wait(lambda: rig.status(S)["led"] == "link down", 15, "sender LED: link down")
    rig.note(f"sender LED 'link down' after {time.time() - t0:.0f}s")
    rig.wait(lambda: rig.status(R)["link_lost"] and rig.status(R)["led"] == "link down", 40,
             "receiver LED: link down")
    rig.note(f"receiver LED 'link down' after {time.time() - t0:.0f}s")
    rig.local_radio(R, BENCH)
    rig.wait_acks(2, 20)
    rig.wait(lambda: rig.status(S)["led"] == "ok" or rig.status(S)["led"] == "event", 10,
             "sender LED back to normal")
    rig.wait(lambda: not rig.status(R)["link_lost"], 10, "receiver link back")


@test("link")
def radio_during_provisional_window(rig):
    """An explicit local /radio is the operator's decision: it must end a pending
    provisional window and be what is saved, not the profile the window would
    have fallen back to."""
    rig.baseline()
    rig.push(dict(BENCH, cr=6))
    rig.expect(rig.status(S)["rollback_pending"], "push did not go provisional")
    for b in (S, R):
        rig.local_radio(b, dict(BENCH, cr=7))
    for b in (S, R):
        st = rig.status(b)
        rig.expect(st["cr"] == 7, f"{b} not on CR4/7")
        rig.expect(not st["rollback_pending"], f"{b}: /radio left the provisional window running "
                   f"(would revert in {st['rollback_s']}s)")
    boot = rig.status(S)["boot_s"]
    rig.get(S, "/reboot?confirm=1")
    rig.wait(lambda: rig.status(S)["boot_s"] < boot, 30, "reboot")
    rig.expect(rig.cfg_now(S)["cr"] == 7, f"after reboot the sender is on CR4/{rig.cfg_now(S)['cr']}, "
               "not the CR4/7 set with /radio")
    for b in (S, R):
        rig.local_radio(b, BENCH)


@test("link", slow=True)
def push_to_deaf_receiver_rolls_back(rig):
    rig.baseline()
    rig.local_radio(R, dict(BENCH, sf=8))   # the push cannot be heard
    cfg = dict(BENCH, sf=9)
    state = rig.push(cfg)
    rig.expect("did not confirm" in state, f"push state: {state}")
    rig.expect(rig.status(S)["rollback_pending"], "sender not provisional")
    win = silence_window_s(cfg, BENCH_INTERVAL)
    rig.note(f"applied unconfirmed; expecting revert to SF7 within {win}s")
    rig.wait(lambda: not rig.status(S)["rollback_pending"], win + 20, "sender revert", poll=5)
    rig.expect(rig.cfg_now(S)["sf"] == 7, "sender did not revert to SF7")
    fl = flashes(rig.get(S, "/led?trace=1")["trace"])
    rig.expect(has_sequence(fl, [(RED, 150), (CYAN, 150)]), "no red-cyan revert flash")
    rig.local_radio(R, BENCH)
    rig.expect(rig.link_ok(30), "link not back after revert")


# --- control -----------------------------------------------------------------

@test("control")
def stop_start_and_polling(rig):
    rig.baseline()
    rig.control(run=0)
    s0 = rig.status(S)["tx"]
    rig.wait(lambda: rig.status(R)["sender_stopped"], 15, "receiver to see the sender stopped")
    rig.wait(lambda: rig.status(S)["led"] == "stopped", 10, "sender LED stopped")
    rig.wait(lambda: rig.status(R)["led"] == "stopped", 10, "receiver LED stopped")
    time.sleep(11)
    s1 = rig.status(S)
    rig.expect(s1["tx"] == s0, "a stopped sender sent test packets")
    rig.expect(s1["consec_miss"] == 0, "polls going unanswered while stopped")
    rig.control(run=1)
    rig.wait_acks(2, 15)
    rig.wait(lambda: not rig.status(R)["sender_stopped"], 10, "receiver to see it running")


@test("control")
def interval_zero_back_to_back(rig):
    rig.baseline(interval=0)
    rig.get(S, "/control?reset=1")
    rig.get(R, "/reset")
    a0, t0 = rig.acks(), time.time()
    time.sleep(20)
    a1, t1 = rig.acks(), time.time()
    rate = (a1 - a0) / (t1 - t0)
    ex = (airtime_ms(7, 125000, 5, 8, 1, 255) + airtime_ms(7, 125000, 5, 8, 1, LINK_ACK_LEN)) / 1000
    rig.note(f"interval 0: {rate:.2f} exchanges/s; airtime alone allows {1 / ex:.2f}/s")
    rig.expect(rate > 0.85 / ex, "back-to-back is not back-to-back")
    r, st = rig.status(R), rig.status(S)
    rig.note(f"sender {st['tx']} sent / {st['ack']} acked; receiver {r['rx']} received, "
             f"lost {r['lost']}, pdr {r['pdr']}%")
    rig.expect(r["lost"] == 0 and st["timeout"] == 0, f"losses at interval 0: pdr {r['pdr']}")
    rig.control(interval_ms=600000)
    rig.expect(rig.status(S)["interval_ms"] == 600000, "600000 ms not accepted")
    rig.control(interval_ms=BENCH_INTERVAL)


@test("control")
def payload_sizes(rig):
    rig.baseline()
    for size in (17, 100, 255):
        rig.control(size=size)
        cursor = rig.get(R, "/log?since=999999999")["next"]
        rig.wait_acks(2, 20)
        ents = rig.get(R, f"/log?since={cursor}")["entries"]
        rig.expect(ents and all(e["len"] == size for e in ents[-1:]),
                   f"receiver saw {[e['len'] for e in ents]} for size {size}")
    rig.control(size=255)


@test("control")
def reset_mid_exchange_keeps_counts(rig):
    """/control?reset=1 runs on the HTTP task, so it can land between a packet's
    TxDone and its ACK. Sent must stay acked + missed (plus at most one in flight)."""
    import random
    # 17 B back-to-back: the wait for the ACK is a large share of every exchange.
    rig.baseline(interval=0, payload=17)
    worst = []
    # Measured on the unfixed firmware: about one reset in five left ack > sent.
    for _ in range(60):
        time.sleep(random.uniform(0.0, 0.5))
        rig.get(S, "/control?reset=1")
        time.sleep(0.6)  # past the exchange that was in flight
        st = rig.status(S)
        d = st["tx"] - st["ack"] - st["timeout"]
        worst.append(d)
        rig.expect(st["ack"] <= st["tx"], f"ack {st['ack']} > sent {st['tx']} after a reset")
        rig.expect(d in (0, 1), f"sent - acked - missed = {d}")
    rig.note(f"60 resets at random points: sent - acked - missed stayed in {sorted(set(worst))}")
    rig.control(interval_ms=BENCH_INTERVAL, size=255)


@test("control")
def counters_reset(rig):
    rig.baseline()
    seq = rig.status(S)["next_seq"]
    rig.get(S, "/control?reset=1")
    rig.get(R, "/reset")
    s, r = rig.status(S), rig.status(R)
    rig.expect(s["tx"] <= 1 and s["ack"] <= 1 and r["rx"] <= 1, "counters not zeroed")
    rig.expect(s["next_seq"] >= seq, "sequence went backwards on reset")
    rig.wait_acks(2, 20)
    r = rig.status(R)
    rig.expect(r["pdr"] == 100.0, f"pdr after reset {r['pdr']}")


# --- remote commands -----------------------------------------------------------

def sendercmd(rig, q, done_text=None, timeout=40):
    before = rig.status(R)["cmd_state"]
    rig.get(R, f"/sendercmd?{q}")
    def done():
        st = rig.status(R)
        return st["cmd_queued"] == 0 and st["cmd_state"] != before and st
    st = rig.wait(done, timeout, f"sender to confirm {q}")
    if done_text:
        rig.expect(done_text in st["cmd_state"], f"{q}: {st['cmd_state']}")
    return st


@test("remote")
def every_remote_command(rig):
    rig.baseline()
    st = sendercmd(rig, "op=status", "status: done")
    rig.expect(st["sender_known"], "sender not known after status")
    sendercmd(rig, "op=interval&ms=1500", "interval 1500 ms: done")
    rig.expect(rig.status(S)["interval_ms"] == 1500 and rig.status(R)["sender_interval_ms"] == 1500,
               "interval not applied / mirrored")
    sendercmd(rig, "op=payload&size=64", "payload 64 B: done")
    rig.expect(rig.status(S)["payload_len"] == 64, "payload not applied")
    sendercmd(rig, "op=run&on=0", "stop: done")
    rig.expect(not rig.status(S)["running"], "sender still running")
    sendercmd(rig, "op=run&on=1", "start: done")
    rig.expect(rig.status(S)["running"], "sender not running")
    sendercmd(rig, "op=mode&m=battery", "battery test: done")
    rig.wait(lambda: rig.status(R)["mode"] == "battery" and rig.status(R)["have_batt"], 20,
             "receiver following battery mode")
    sendercmd(rig, "op=mode&m=range", "range test: done")
    rig.wait(lambda: rig.status(R)["mode"] == "range", 20, "receiver following range mode")
    sendercmd(rig, "op=reset", "reset counters: done")
    sendercmd(rig, "op=clock_reset", "reset discharge clock: done")
    rig.expect(rig.status(S)["uptime_s"] < 30, "discharge clock not reset")
    sendercmd(rig, "op=calibrate&actual_mv=4200", "done")
    ratio = rig.status(S)["ratio"]
    rig.expect(abs(rig.status(R)["sender_ratio"] - ratio) < 0.002, "ratio not mirrored")
    rig.note(f"remote calibrate -> ratio {ratio:.3f}")
    sendercmd(rig, "op=ratio_reset", "reset ratio: done")
    rig.expect(abs(rig.status(S)["ratio"] - 4.333) < 0.001, "ratio not back to default")
    sendercmd(rig, "op=load&on=1", "max drain on: done")
    rig.wait(lambda: rig.status(S)["led"] == "max drain", 5, "sender LED max drain")
    sendercmd(rig, "op=load&on=0", "max drain off: done")
    rig.wait(lambda: rig.status(S)["led"] != "max drain", 5, "sender LED back")
    sendercmd(rig, "op=push&confirm=1&sf=8", "push SF8")
    rig.wait(lambda: rig.cfg_now(S)["sf"] == 8 and rig.cfg_now(R)["sf"] == 8, 30, "SF8 on both")
    rig.wait_acks(2, 30)
    sendercmd(rig, "op=push&confirm=1&sf=7", "push SF7")
    rig.wait(lambda: rig.cfg_now(S)["sf"] == 7 and rig.cfg_now(R)["sf"] == 7, 30, "SF7 on both")
    fl = flashes(rig.get(R, "/led?trace=1")["trace"])
    rig.expect(has_sequence(fl, [(VIOLET, 60), (VIOLET, 60)]), "no violet double flash on the receiver")
    rig.control(interval_ms=BENCH_INTERVAL, size=255)


@test("remote")
def queue_full_and_cancel(rig):
    rig.baseline()
    # Deaf to the sender, so nothing drains while the queue is being filled.
    rig.local_radio(R, dict(BENCH, sf=8))
    codes = [rig.req(R, "/sendercmd?op=status")[0] for _ in range(9)]
    st = rig.status(R)
    rig.note(f"9 queued -> {codes}; queued {st['cmd_queued']}")
    rig.expect(400 in codes and st["cmd_queued"] >= 7, "queue never reported full")
    rig.expect(st["led"] == "command waiting", f"receiver LED {st['led']}")
    rig.get(R, "/sendercmd?op=cancel")
    rig.expect(rig.status(R)["cmd_queued"] == 0, "cancel left commands")
    rig.local_radio(R, BENCH)
    rig.expect(rig.link_ok(30), "link not back")


@test("remote", slow=True)
def remote_wifi_off_and_reboot(rig):
    rig.baseline()
    sendercmd(rig, "op=wifi_off&confirm=1", "WiFi off: done")
    s = rig.status(S)  # the console still answers: loopback needs no WiFi
    rig.expect(s["wifi"] is False and s["load"] is False, "WiFi still on")
    rig.expect(rig.status(R)["sender_wifi"] is False, "receiver not told")
    rig.wait_acks(2, 20)
    boot = s["boot_s"]
    since = time.time()
    # The receiver follows a confirmed reboot with a status request of its own, so
    # by the time the queue is empty the last outcome may already be that one.
    sendercmd(rig, "op=reboot&confirm=1", timeout=60)
    rig.wait(lambda: rig.status(S)["boot_s"] < boot, 30, "sender reboot")
    rig.wait(lambda: rig.log_has(R, since, "Sender: reboot: done"), 10, "reboot confirmed")
    rig.wait(lambda: rig.status(R)["sender_wifi"] and rig.status(R)["cmd_queued"] == 0, 60,
             "the follow-up status showing WiFi back")
    time.sleep(15)
    rig.expect(rig.status(S)["boot_s"] >= 15, "sender rebooted again: reboot command replayed")


# --- battery mode -------------------------------------------------------------

@test("battery")
def battery_mode_and_csv(rig):
    rig.baseline()
    csv0 = rig.get(R, "/data.csv")
    with open(os.path.join(rig.outdir, "receiver_csv_before.csv"), "w") as f:
        f.write(csv0)
    rig.note(f"backed up the receiver's CSV ({len(csv0.splitlines()) - 1} rows) before testing")
    rig.get(S, "/mode?m=battery")
    rig.wait(lambda: rig.status(R)["mode"] == "battery", 15, "receiver to follow")
    rig.wait_acks(3, 20)
    r, s = rig.status(R), rig.status(S)
    rig.expect(r["have_batt"] and r["pin_mv"] > 0, "no battery sample at the receiver")
    rig.note(f"sender pin {s['pin_mv']} mV -> {s['batt_mv']} mV; receiver shows "
             f"{r['batt_mv']} mV (ratio {r['ratio']})")
    rig.expect(abs(r["batt_mv"] - s["batt_mv"]) < 60, "receiver battery differs from sender")
    csv1 = rig.get(R, "/data.csv")
    rig.expect(len(csv1.splitlines()) >= len(csv0.splitlines()) + 3, "CSV did not grow")
    rig.expect(csv1.splitlines()[0] == "uptime_s,pin_mv,rssi_dbm,snr_db,seq", "CSV header")
    last = csv1.splitlines()[-1].split(",")
    rig.expect(len(last) == 5 and int(last[1]) > 0, f"bad CSV row {last}")
    rig.get(S, "/uptime?reset=1")
    rig.wait_acks(2, 20)
    rig.expect(int(rig.get(R, "/data.csv").splitlines()[-1].split(",")[0]) < 30,
               "CSV uptime not restarted after clock reset")
    rig.get(R, "/clear?confirm=1")
    rig.expect(len(rig.get(R, "/data.csv").splitlines()) <= 2, "CSV not cleared")
    rig.wait_acks(2, 20)
    rig.expect(len(rig.get(R, "/data.csv").splitlines()) >= 2, "CSV not appending after clear")
    rig.get(S, "/mode?m=range")
    rig.wait(lambda: rig.status(R)["mode"] == "range", 15, "receiver back to range")


@test("battery")
def calibration(rig):
    b0 = rig.get(S, "/battery")
    rig.note(f"pin {b0['pin_mv']} mV, ratio {b0['ratio']}, battery {b0['batt_mv']} mV")
    rig.restore_ratio = True
    b1 = rig.get(S, "/battery?actual_mv=4100")
    rig.expect(abs(b1["batt_mv"] - 4100) < 40, f"calibrated reading {b1['batt_mv']}")
    rig.expect(rig.req(S, "/battery?actual_mv=60000")[0] == 400, "absurd reading accepted")
    b2 = rig.get(S, "/battery?ratio_x1000=4765")
    rig.expect(abs(b2["ratio"] - 4.765) < 0.001, "ratio not set directly")
    b3 = rig.get(S, "/battery?reset=1")
    rig.expect(abs(b3["ratio"] - b3["default_ratio"]) < 0.001, "reset did not restore default")


# --- max drain ------------------------------------------------------------------

@test("drain", slow=True)
def max_drain_white_and_survives(rig):
    rig.baseline(interval=0)
    rig.push(dict(BENCH, pwr=22))       # the worst case is the point
    s = rig.status(S)
    rig.get(S, "/load?on=1")
    st = rig.wait(lambda: (x := rig.status(S))["led"] == "max drain" and x, 5, "LED max drain")
    rig.expect(st["led_rgb"] == "#ffffff", f"LED colour {st['led_rgb']}, wanted full white")
    led = rig.get(S, "/led")
    rig.note(f"max drain: LED '{led['pattern']}' {led['rgb']}")
    # Flashes must not break the white: every trace entry while on is white.
    a0, t0 = rig.acks(), time.time()
    boot0 = s["boot_s"]
    time.sleep(90)
    s = rig.status(S)
    rig.expect(s["boot_s"] > boot0, f"sender reset under max drain ({s['reset_reason']})")
    tr = [t for t in rig.get(S, "/led?trace=1")["trace"] if t[0] / 1000 > s["boot_s"] - 85]
    rig.expect(all(t[1] == "#ffffff" for t in tr), f"LED left white during max drain: {tr[-5:]}")
    rate = (s["ack"] - a0) / (time.time() - t0)
    r = rig.status(R)
    rig.note(f"90 s at max drain, +22 dBm, interval 0: {rate:.2f} exchanges/s, "
             f"pdr {r['pdr']}%, sender heap min {s['heap_min']} B, no reset")
    rig.expect(s["led"] == "max drain" and s["led_rgb"] == "#ffffff", "LED not white at the end")
    # Arming: reboot with the load still on.
    boot = rig.status(S)["boot_s"]
    rig.get(S, "/reboot?confirm=1")
    rig.wait(lambda: rig.status(S)["boot_s"] < boot, 30, "reboot")
    st = rig.status(S)
    rig.expect(st["load"] and st["load_arming_s"] > 0, "load not resumed / not arming")
    rig.expect(st["led"] == "max drain arming", f"LED during arming: {st['led']}")
    rig.note(f"after reboot: LED '{st['led']}', arming in {st['load_arming_s']}s")
    rig.wait(lambda: rig.status(S)["led"] == "max drain", 25, "white once armed")
    rig.get(S, "/load?on=0")
    rig.wait(lambda: rig.status(S)["led"] not in ("max drain", "max drain arming"), 5, "LED off white")
    rig.push(BENCH)
    rig.control(interval_ms=BENCH_INTERVAL)


@test("drain")
def wifi_off_stops_drain(rig):
    rig.baseline()
    rig.get(S, "/load?on=1")
    rig.wait(lambda: rig.status(S)["led"] == "max drain", 5, "LED max drain")
    rig.get(S, "/wifi?on=0")
    time.sleep(1)
    s = rig.status(S)
    rig.expect(not s["wifi"] and not s["load"], "load or WiFi still on")
    rig.expect(s["led"] != "max drain", "LED still white with WiFi off")
    rig.wait_acks(2, 20)
    boot = s["boot_s"]
    rig.get(S, "/reboot?confirm=1")
    rig.wait(lambda: rig.status(S)["boot_s"] < boot, 30, "reboot")
    rig.expect(rig.status(S)["wifi"], "WiFi not back after reboot")


# --- persistence ----------------------------------------------------------------

@test("persist")
def sender_reboot_resumes(rig):
    rig.baseline()
    try:
        rig.control(size=200, interval_ms=1200)
        rig.get(S, "/led?level=40")
        s0 = rig.status(S)
        last = rig.get(S, "/log?since=0")["entries"][-1]
        rig.get(S, "/reboot?confirm=1")
        rig.wait(lambda: rig.status(S)["boot_s"] < s0["boot_s"], 30, "reboot")
        s1 = rig.status(S)
        rig.expect(s1["next_seq"] >= s0["next_seq"], "sequence went backwards")
        rig.expect(s1["interval_ms"] == 1200 and s1["payload_len"] == 200, "settings lost")
        rig.expect(s1["led_level"] == 40, "LED level lost")
        # Saved before each transmit, so a reset drops at most the last exchange from
        # the tallies - its "sent" and its outcome together.
        rig.expect(s1["tx"] >= s0["tx"] - 1 and s1["ack"] >= s0["ack"] - 1, "counters lost")
        rig.expect(s1["tx"] == s1["ack"] + s1["timeout"], "sent != acked + missed after the reset")
        # The clock is saved with each packet, so a reset loses the time since the
        # last one; what it must never do is step back past a sample already sent.
        rig.expect(s1["uptime_s"] >= last["uptime_s"],
                   f"discharge clock {s1['uptime_s']}s is behind the last sample's {last['uptime_s']}s")
        rig.wait_acks(2, 20)
        r = rig.status(R)
        rig.note(f"next_seq {s0['next_seq']} -> {s1['next_seq']}, sent {s0['tx']} -> {s1['tx']}; "
                 f"receiver pdr {r['pdr']}%, gap counted as lost: {r['lost']}")
    finally:
        rig.get(S, "/led?level=24")
        rig.control(size=255, interval_ms=BENCH_INTERVAL)


@test("persist")
def receiver_reboot_resumes(rig):
    rig.baseline()
    r0 = rig.status(R)
    rig.get(R, "/reboot?confirm=1")
    t0 = time.time()
    st = rig.wait(lambda: (x := rig.status(R))["boot_s"] < r0["boot_s"] and x, 30, "reboot")
    rig.note(f"receiver LED right after boot: '{st['led']}' (contact_seen {st['contact_seen']})")
    rig.wait_acks(2, 20)
    r1 = rig.status(R)
    rig.expect(r1["rx"] >= r0["rx"], "receiver statistics lost")
    rig.expect(r1["lost"] >= r0["lost"], "losses forgotten")
    rig.note(f"rx {r0['rx']} -> {r1['rx']}, lost {r0['lost']} -> {r1['lost']} "
             f"(packets sent while it rebooted count as lost)")


@test("persist")
def hardware_reset_both(rig):
    """A reset through the USB serial chip, the closest to a power cycle here."""
    import subprocess
    rig.baseline()
    s0, r0 = rig.status(S), rig.status(R)
    py = sys.executable
    ports = {S: "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:BD:8D:23:3A:24-if00",
             R: "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_44:BD:8D:23:3A:B8-if00"}
    for b in (S, R):
        client({"op": "release", "board": b})
        subprocess.run([py, "-m", "esptool", "--chip", "esp32c3", "-p", ports[b],
                        "--after", "hard_reset", "chip_id"], capture_output=True, timeout=60)
        time.sleep(1)
        client({"op": "attach", "board": b})
    time.sleep(3)
    s1, r1 = rig.status(S), rig.status(R)
    rig.expect(s1["boot_s"] < 30 and r1["boot_s"] < 30, "boards did not reset")
    rig.note(f"reset reasons: sender {s1['reset_reason']}, receiver {r1['reset_reason']}")
    rig.expect(s1["next_seq"] >= s0["next_seq"] and r1["rx"] >= r0["rx"], "run state lost")
    rig.expect(rig.same(rig.cfg_now(S), BENCH) and rig.same(rig.cfg_now(R), BENCH), "profile lost")
    rig.wait_acks(3, 30)


@test("persist", slow=True)
def usb_host_not_reading(rig):
    """In the field nothing reads the USB port. Logging must then drop, not block:
    a stalled log line on the radio task would delay ACKs and lose packets."""
    rig.baseline(interval=0)
    for b in (S, R):
        other = R if b == S else S
        rig.get(S, "/control?reset=1")
        rig.get(R, "/reset")
        boot = rig.status(b)["boot_s"]
        client({"op": "release", "board": b})
        t0 = time.time()
        time.sleep(45)
        client({"op": "attach", "board": b})
        time.sleep(1)
        st, o = rig.status(b), rig.status(other)
        s_st = st if b == S else o
        r_st = st if b == R else o
        rig.expect(st["boot_s"] >= boot + 40, f"{b} reset while its port was closed")
        rig.note(f"{b} port closed 45 s at interval 0: sender {s_st['tx']} sent / "
                 f"{s_st['ack']} acked, receiver lost {r_st['lost']}")
        rig.expect(r_st["lost"] == 0 and s_st["timeout"] == 0,
                   f"packets lost while {b}'s USB was not being read")
    rig.control(interval_ms=BENCH_INTERVAL)


# --- LED --------------------------------------------------------------------------

@test("led")
def self_test_sequence(rig):
    for b in (S, R):
        rig.get(b, "/led?test=1")
        time.sleep(2.5)
        tr = rig.get(b, "/led?trace=1")["trace"]
        cols = [t[1] for t in tr]
        seq = ["#ff0000", "#00ff00", "#0000ff", "#ffffff"]
        found = [c for c in cols if c in seq]
        rig.expect(found[-4:] == seq, f"{b} self-test colours {found[-6:]}")
        times = [t[0] for t in tr if t[1] in seq][-4:]
        gaps = [b2 - a for a, b2 in zip(times, times[1:])]
        rig.expect(all(480 <= g <= 540 for g in gaps), f"{b} self-test timing {gaps}")
    rig.note("R, G, B, W at 255, 500 ms apart, on both")


@test("led")
def packet_flash_timing(rig):
    rig.baseline(interval=2000)
    time.sleep(7)
    for b in (S, R):
        lv = rig.status(b)["led_level"]
        BLUE, GREEN = scaled("#0000ff", lv), scaled("#00ff00", lv)
        tr = rig.get(b, "/led?trace=1")["trace"]
        lit = [(t[0], t[1]) for t in tr if t[1] != "#000000"]
        dur = []
        for i, t in enumerate(tr[:-1]):
            if t[1] != "#000000":
                dur.append((t[1], tr[i + 1][0] - t[0]))
        rig.note(f"{b} flashes (colour, ms lit): {dur[-6:]}")
        if b == S:
            rig.expect(any(c == BLUE and 15 <= d <= 35 for c, d in dur), "no 20 ms blue TX tick")
        rig.expect(any(c == GREEN and 50 <= d <= 75 for c, d in dur), f"{b}: no 60 ms green flash")
        rig.expect(not any(t[2] == "ok" and t[1] != "#000000" for t in tr[-8:]),
                   f"{b}: idle heartbeat showing while packets flow")


@test("led")
def level_zero_and_restore(rig):
    rig.baseline()
    for b in (S, R):
        rig.get(b, "/led?level=0")
    time.sleep(4)
    for b in (S, R):
        st = rig.status(b)
        rig.expect(st["led"] == "off" and st["led_rgb"] == "#000000", f"{b} LED {st['led']}")
        tr = [t for t in rig.get(b, "/led?trace=1")["trace"] if t[0] / 1000 > st["boot_s"] - 3]
        rig.expect(all(t[1] == "#000000" for t in tr), f"{b} lit at level 0: {tr}")
    rig.get(S, "/load?on=1")
    rig.wait(lambda: rig.status(S)["led_rgb"] == "#ffffff", 5, "white at level 0 under max drain")
    rig.get(S, "/load?on=0")
    for b in (S, R):
        rig.get(b, "/led?level=24")
    rig.note("level 0: dark, no flashes; max drain still full white")


@test("led")
def low_battery_pattern(rig):
    """Fake a flat cell with the divider ratio rather than drain one: the LED reads the
    battery through the same calibrated conversion the dashboard shows."""
    rig.baseline(interval=12000)  # a quiet link, so the background shows
    b0 = rig.get(S, "/battery")
    try:
        b = rig.get(S, "/battery?ratio_x1000=3000")
        low = rig.status(S)["led_low_mv"]
        rig.note(f"ratio 3.000: battery reads {b['batt_mv']} mV against the {low} mV threshold")
        rig.expect(b["batt_mv"] < low, "fake reading not below the threshold")
        rig.wait(lambda: rig.status(S)["led"] == "low battery", 25, "LED low battery", poll=0.5)
        time.sleep(5)
        fl = flashes(rig.get(S, "/led?trace=1")["trace"])
        orange = scaled("#ff3200")
        rig.expect(has_sequence(fl, [(orange, 60)] * 3), f"no orange triple blink: {fl[-6:]}")
    finally:
        # Back to exactly what it was: the default stays "uncalibrated" in NVS.
        if abs(b0["ratio"] - b0["default_ratio"]) < 0.0005:
            rig.get(S, "/battery?reset=1")
        else:
            rig.get(S, f"/battery?ratio_x1000={round(b0['ratio'] * 1000)}")
    rig.wait(lambda: rig.status(S)["led"] != "low battery", 25, "LED back from low battery", poll=0.5)
    rig.control(interval_ms=BENCH_INTERVAL)


@test("led")
def idle_heartbeat(rig):
    rig.baseline(interval=12000)
    time.sleep(14)
    tr = rig.get(S, "/led?trace=1")["trace"]
    rig.expect(any(t[1] == "#001800" and t[2] == "ok" for t in tr), "no heartbeat during a long interval")
    rig.control(interval_ms=BENCH_INTERVAL)


# --- soak ---------------------------------------------------------------------------

@test("soak", slow=True)
def soak(rig):
    minutes = rig.soak_min
    rig.baseline(interval=0)
    rig.get(S, "/control?reset=1")
    rig.get(R, "/reset")
    s0, r0 = rig.status(S), rig.status(R)
    t0 = time.time()
    polls, heap_s, heap_r = 0, [], []
    while time.time() - t0 < minutes * 60:
        # Hammer both boards the way an open dashboard does, and harder.
        for b in (S, R):
            st = rig.status(b)
            rig.get(b, "/log?since=0")
            polls += 2
            (heap_s if b == S else heap_r).append(st["heap_free"])
            rig.expect(st["boot_s"] >= (time.time() - t0) - 5,
                       f"{b} reset during the soak ({st['reset_reason']})")
        time.sleep(0.5)
    s1, r1 = rig.status(S), rig.status(R)
    rig.note(f"{minutes} min at interval 0 with {polls} dashboard requests: sender {s1['tx']} sent, "
             f"{s1['ack']} acked ({s1['ack_rate']}%); receiver {r1['rx']} received, "
             f"pdr {r1['pdr']}%, lost {r1['lost']}, crc {r1['crc_err']}, dup {r1['dup']}")
    rig.note(f"heap free sender {min(heap_s)}..{max(heap_s)} (min ever {s1['heap_min']}), "
             f"receiver {min(heap_r)}..{max(heap_r)} (min ever {r1['heap_min']})")
    rig.expect(r1["pdr"] >= 99.0, f"pdr {r1['pdr']}")
    rig.expect(heap_s[-1] > heap_s[0] - 2048 and heap_r[-1] > heap_r[0] - 2048, "heap shrinking")
    rig.control(interval_ms=BENCH_INTERVAL)


# --------------------------------------------------------------------- runner ----

def snapshot(rig):
    s = rig.status(S)
    return dict(cfg=rig.cfg_now(S), interval=s["interval_ms"], payload=s["payload_len"],
                mode=s["mode"], load=s["load"], running=s["running"],
                led_s=s["led_level"], led_r=rig.status(R)["led_level"])


def restore(rig, snap):
    rig.baseline()
    rig.push(snap["cfg"])
    rig.wait_acks(2, 60)
    rig.control(interval_ms=snap["interval"], size=snap["payload"], run=int(snap["running"]))
    rig.get(S, f"/mode?m={snap['mode']}")
    rig.get(S, f"/load?on={int(snap['load'])}")
    rig.get(S, f"/led?level={snap['led_s']}")
    rig.get(R, f"/led?level={snap['led_r']}")
    # Commit the restored profile so a reset cannot take it back to the bench one.
    for b in (S, R):
        if rig.status(b)["rollback_pending"]:
            rig.local_radio(b, snap["cfg"])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("groups", nargs="*")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("-k", dest="names", action="append", default=[],
                    help="only tests whose name contains this (repeatable)")
    ap.add_argument("--fast", action="store_true", help="skip the slow tests")
    ap.add_argument("--soak-min", type=float, default=20)
    ap.add_argument("--no-restore", action="store_true")
    args = ap.parse_args()

    if args.list:
        for g, n, _, slow in TESTS:
            print(f"{g:9} {n}{'  (slow)' if slow else ''}")
        return

    groups = args.groups or sorted({g for g, *_ in TESTS} - {"soak"})
    if args.names and not args.groups:
        groups = sorted({g for g, *_ in TESTS})
    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join("test-results", stamp)
    os.makedirs(outdir, exist_ok=True)
    rig = Rig(outdir)
    rig.soak_min = args.soak_min
    snap = snapshot(rig)
    print(f"start state: {snap}")

    results = []
    for g, name, fn, slow in TESTS:
        if args.names:
            if not any(k in name for k in args.names):
                continue
        elif g not in groups or (slow and args.fast):
            continue
        print(f"[{g}] {name}", flush=True)
        rig.notes = []
        t0 = time.time()
        try:
            fn(rig)
            ok, err = True, ""
        except Fail as e:
            ok, err = False, str(e)
        except Exception as e:
            ok, err = False, f"{type(e).__name__}: {e}\n{traceback.format_exc()}"
        dt = time.time() - t0
        print(f"    {'PASS' if ok else 'FAIL'} ({dt:.0f}s){'  ' + err if err else ''}", flush=True)
        results.append(dict(group=g, name=name, ok=ok, error=err, notes=rig.notes,
                            seconds=round(dt)))

    if not args.no_restore:
        try:
            restore(rig, snap)
            print("restored the starting settings")
        except Exception as e:
            print(f"RESTORE FAILED: {e}")

    with open(os.path.join(outdir, "results.json"), "w") as f:
        json.dump(dict(start=snap, results=results, matrix=getattr(rig, "matrix", None)), f, indent=1)
    passed = sum(r["ok"] for r in results)
    with open(os.path.join(outdir, "report.md"), "w") as f:
        f.write(f"# Rig test {stamp}\n\n{passed}/{len(results)} passed\n\n")
        for r in results:
            f.write(f"- **{'PASS' if r['ok'] else 'FAIL'}** `{r['group']}/{r['name']}` ({r['seconds']} s)\n")
            for n in r["notes"]:
                f.write(f"    - {n}\n")
            if r["error"]:
                f.write(f"    - error: {r['error'].splitlines()[0]}\n")
    print(f"\n{passed}/{len(results)} passed -> {outdir}/report.md")
    sys.exit(0 if passed == len(results) else 1)


if __name__ == "__main__":
    main()
