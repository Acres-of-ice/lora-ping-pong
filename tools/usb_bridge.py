#!/usr/bin/env python3
"""Both dashboards over USB, with no WiFi needed.

The firmware answers any dashboard route typed on its USB serial console (see
main/console.h). This holds the boards' serial ports open, logs everything they
print, and serves each board's real dashboard on localhost:

    tools/usb_bridge.py serve --sender /dev/serial/by-id/...3A:24-if00 \\
                              --receiver /dev/serial/by-id/...3A:B8-if00

    http://127.0.0.1:8081/   sender dashboard
    http://127.0.0.1:8082/   receiver dashboard

and answers one-off requests from another shell while it runs:

    tools/usb_bridge.py get receiver /status
    tools/usb_bridge.py get sender "/control?run=0"

Keeping the ports open matters: opening an ESP32-C3's USB serial port resets the
board, so a tool that reopened it per request would reset the run each time.
release/attach close and reopen a port so it can be flashed meanwhile.

Needs pyserial (it is in the ESP-IDF Python environment).
"""

import argparse
import json
import os
import re
import socket
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CONTROL_SOCK = os.environ.get("USB_BRIDGE_SOCK", "/tmp/lora-usb-bridge.sock")
HTTP_PORTS = {"sender": 8081, "receiver": 8082}
ANSI = re.compile(rb"\x1b\[[0-9;]*m")


def parse_http(raw):
    """(status, headers, body) from a raw HTTP/1.1 response, de-chunked."""
    head, _, body = raw.partition(b"\r\n\r\n")
    lines = head.decode("latin-1").split("\r\n")
    status = int(lines[0].split()[1]) if lines and len(lines[0].split()) > 1 else 0
    headers = {}
    for ln in lines[1:]:
        k, _, v = ln.partition(":")
        headers[k.strip().lower()] = v.strip()
    if headers.get("transfer-encoding", "").lower() == "chunked":
        out, rest = b"", body
        while rest:
            size_line, _, rest = rest.partition(b"\r\n")
            try:
                n = int(size_line.split(b";")[0], 16)
            except ValueError:
                break
            if n == 0:
                break
            out += rest[:n]
            rest = rest[n + 2:]
        body = out
    return status, headers, body


class Board:
    """One serial port: a reader thread, a log file, and framed request/response."""

    def __init__(self, name, port, logdir):
        self.name, self.port = name, port
        self.ser = None
        self.lock = threading.Lock()          # one request in flight per board
        self.cond = threading.Condition()
        self.frames = {}                      # id -> {"data": bytearray, "end": str|None}
        self.next_id = 1000
        self.log = open(os.path.join(logdir, f"{name}.log"), "a", buffering=1)
        self.lines = []                       # recent log lines, for "tail"
        self.reader = None
        self.stop = threading.Event()
        self.last_took_ms = None

    def attach(self):
        import serial
        if self.ser is not None:
            return
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, 115200, 0.1
        # Leave DTR/RTS as the kernel sets them on open. Toggling them, as pyserial
        # does when told dtr=False/rts=False, is exactly the sequence that resets
        # an ESP32-C3 through its USB serial port.
        s.open()
        self.ser = s
        self.stop.clear()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()
        self._log_line(f"### bridge attached {self.port}")

    def release(self):
        if self.ser is None:
            return
        self.stop.set()
        self.reader.join(2)
        try:
            self.ser.close()
        finally:
            self.ser = None
        self._log_line("### bridge released port")

    def _log_line(self, text):
        stamp = time.strftime("%H:%M:%S") + f".{int(time.time() * 1000) % 1000:03d}"
        self.log.write(f"{stamp} {text}\n")
        self.lines.append((time.time(), text))
        del self.lines[:-4000]

    def _read(self):
        buf = b""
        while not self.stop.is_set():
            try:
                # Whatever has arrived, or wait for the first byte: a fixed-size
                # read sat out its whole timeout on every short reply.
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:  # unplugged, or reset mid-read
                self._log_line(f"### read error: {e}")
                time.sleep(1)
                try:
                    self.ser.close()
                    self.ser.open()
                except Exception:
                    pass
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle(ANSI.sub(b"", line.rstrip(b"\r")))

    def _handle(self, line):
        # A frame marker can follow a log fragment on the same line if the two
        # interleaved mid-line, so look for it anywhere.
        i = line.find(b"@@")
        if i >= 0 and line[i + 2:i + 3] in (b"B", b"D", b"E") and line[i + 3:i + 4] == b" ":
            if i > 0:
                self._log_line(line[:i].decode(errors="replace"))
            kind = line[i + 2:i + 3]
            parts = line[i + 4:].split(b" ", 2)
            try:
                fid = int(parts[0])
            except (ValueError, IndexError):
                return
            with self.cond:
                f = self.frames.setdefault(fid, {"data": bytearray(), "end": None})
                if kind == b"D" and len(parts) > 1:
                    try:
                        f["data"] += bytes.fromhex(parts[1].decode())
                    except ValueError:
                        f["corrupt"] = True
                elif kind == b"E":
                    f["end"] = b" ".join(parts[1:]).decode(errors="replace")
                    self.cond.notify_all()
            return
        self._log_line(line.decode(errors="replace"))

    def request(self, path, timeout=15.0):
        """GET path on the board. Returns (status, headers, body) or raises."""
        if self.ser is None:
            raise RuntimeError(f"{self.name}: port released")
        with self.lock:
            fid = self.next_id
            self.next_id += 1
            with self.cond:
                self.frames[fid] = {"data": bytearray(), "end": None}
            self.ser.write(f"@{fid} {path}\n".encode())
            deadline = time.time() + timeout
            with self.cond:
                while self.frames[fid]["end"] is None:
                    left = deadline - time.time()
                    if left <= 0:
                        self.frames.pop(fid, None)
                        raise TimeoutError(f"{self.name}: no answer to {path} in {timeout}s")
                    self.cond.wait(left)
                f = self.frames.pop(fid)
        total, _, rest = f["end"].partition(" ")
        took, err = None, rest
        if rest.startswith("+"):
            took, _, err = rest[1:].partition(" ")
        self.last_took_ms = int(took) if took and took.isdigit() else None
        if err:
            raise RuntimeError(f"{self.name}: {path}: {err}")
        if f.get("corrupt") or int(total) != len(f["data"]):
            raise RuntimeError(f"{self.name}: {path}: framing lost "
                               f"({len(f['data'])} of {total} bytes)")
        return parse_http(bytes(f["data"]))


class Bridge:
    def __init__(self, ports, logdir):
        os.makedirs(logdir, exist_ok=True)
        self.boards = {n: Board(n, p, logdir) for n, p in ports.items() if p}

    def handle(self, msg):
        op, name = msg.get("op", "get"), msg.get("board")
        b = self.boards.get(name)
        if b is None:
            return {"error": f"no board {name!r}"}
        try:
            if op == "get":
                t0 = time.time()
                st, hd, body = b.request(msg["path"], msg.get("timeout", 15.0))
                return {"status": st, "type": hd.get("content-type", ""),
                        "body": body.decode("utf-8", errors="replace"),
                        "board_ms": b.last_took_ms,
                        "total_ms": round((time.time() - t0) * 1000)}
            if op == "tail":
                since = msg.get("since", 0)
                return {"lines": [[t, l] for t, l in list(b.lines) if t > since],
                        "now": time.time()}
            if op == "write":  # raw text to the console, e.g. a blank line
                b.ser.write(msg["text"].encode())
                return {"ok": True}
            if op == "release":
                b.release()
                return {"ok": True}
            if op == "attach":
                b.attach()
                return {"ok": True}
        except Exception as e:
            return {"error": str(e)}
        return {"error": f"unknown op {op!r}"}


def serve(args):
    bridge = Bridge({"sender": args.sender, "receiver": args.receiver}, args.logdir)
    for b in bridge.boards.values():
        b.attach()

    if os.path.exists(CONTROL_SOCK):
        os.unlink(CONTROL_SOCK)

    class Control(socketserver.StreamRequestHandler):
        def handle(self):
            for line in self.rfile:
                reply = bridge.handle(json.loads(line))
                self.wfile.write((json.dumps(reply) + "\n").encode())

    class UnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
        daemon_threads = True

    ctl = UnixServer(CONTROL_SOCK, Control)
    threading.Thread(target=ctl.serve_forever, daemon=True).start()

    def proxy_for(board):
        class Proxy(BaseHTTPRequestHandler):
            def do_GET(self):
                try:
                    st, hd, body = board.request(self.path, 30.0)
                except Exception as e:
                    self.send_error(502, str(e))
                    return
                self.send_response(st)
                for k in ("content-type", "content-disposition", "cache-control"):
                    if k in hd:
                        self.send_header(k, hd[k])
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, fmt, *a):
                pass
        return Proxy

    for name, b in bridge.boards.items():
        srv = ThreadingHTTPServer(("127.0.0.1", HTTP_PORTS[name]), proxy_for(b))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"{name}: {b.port} -> http://127.0.0.1:{HTTP_PORTS[name]}/", flush=True)
    print(f"control socket {CONTROL_SOCK}; logs in {args.logdir}", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass


def client(msg):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(CONTROL_SOCK)
    s.sendall((json.dumps(msg) + "\n").encode())
    data = b""
    while not data.endswith(b"\n"):
        chunk = s.recv(65536)
        if not chunk:
            break
        data += chunk
    s.close()
    return json.loads(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("serve")
    sp.add_argument("--sender")
    sp.add_argument("--receiver")
    sp.add_argument("--logdir", default="usb_bridge_logs")
    g = sub.add_parser("get")
    g.add_argument("board")
    g.add_argument("path")
    g.add_argument("--timeout", type=float, default=15.0)
    for op in ("release", "attach"):
        sub.add_parser(op).add_argument("board")
    args = ap.parse_args()

    if args.cmd == "serve":
        serve(args)
    elif args.cmd == "get":
        r = client({"op": "get", "board": args.board, "path": args.path,
                    "timeout": args.timeout})
        if "error" in r:
            sys.exit(f"error: {r['error']}")
        print(r["body"])
        if r["status"] != 200:
            sys.exit(f"HTTP {r['status']}")
    else:
        r = client({"op": args.cmd, "board": args.board})
        if "error" in r:
            sys.exit(f"error: {r['error']}")


if __name__ == "__main__":
    main()
