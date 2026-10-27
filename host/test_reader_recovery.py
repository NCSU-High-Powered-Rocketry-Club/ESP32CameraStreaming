#!/usr/bin/env python3
"""Recovery harness for host/c3_usb_mjpeg.py.

Drives the REAL SerialReader thread against a fake C3 on a pty and checks the
paths that used to kill the reader thread (i.e. that used to require restarting
the script):

  [1] read error (yanked cable / C3 reboot) -> close fd, reconnect, frames again
  [2] port keeps failing to open (re-enumerating) -> keep retrying, never die
      (old code: os.read(None) raised TypeError out of run() -> thread dead)
  [3] open but silent port -> fd cycle (the only thing that clears a wedged
      host-side CDC endpoint)
  [4] a deliberate PAUSE ('x') survives the 'g' keep-alive
  [5] port renamed (ttyACM0 -> ttyACM1) -> adopt the port that answers the probe
  [6] the C3's live link line ("[rx] link rssi=… ch=… sta=…") reaches the reader
      and /status (dashboard SIGNAL card)
  [7] HTTP: /status JSON + strict /ctl validation (incl. the F0/F1
      fps-priority mode + the 1..60 fps cap and AUTO: accepted, relayed to the
      C3, range-validated, and tracked in /status)
  [8] a port held by another program -> "port is locked", not "Bad file descriptor"

Notes on the fake: the device lives behind a symlink (like
/dev/serial/by-id/*), so "the port" can disappear and come back as a different
device under the same name. A Linux USB CDC port returns EIO on read when the
cable is pulled; a pty cannot reproduce that (its reads just return b''), so
that single fault is injected through an os shim - everything else is real.

Usage: python3 c3_recovery_test.py <path-to-c3_usb_mjpeg.py>
"""
import importlib.util
import json
import os
import pty
import struct
import sys
import threading
import time
import urllib.request
import zlib
from http.server import ThreadingHTTPServer

MODULE_PATH = sys.argv[1] if len(sys.argv) > 1 else \
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "c3_usb_mjpeg.py")
spec = importlib.util.spec_from_file_location("c3mod", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

FAILS = []
LINK = "/tmp/c3fake-tty"
LINK2 = "/tmp/c3fake-tty2"


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name +
          ("  [" + detail + "]" if detail else ""))
    if not cond:
        FAILS.append(name)


def frame(fid, payload):
    return (bytes([mod.MAGIC_HI, mod.MAGIC_LO]) +
            struct.pack(">I", len(payload)) + struct.pack(">H", fid) +
            struct.pack(">I", zlib.crc32(payload)) + payload)


class OsShim:
    """`os`, except read() can be armed to raise EIO (yanked USB CDC port)."""

    def __init__(self, real):
        self._real = real
        self.eio = 0

    def __getattr__(self, k):
        return getattr(self._real, k)

    def read(self, fd, n):
        if self.eio > 0:
            self.eio -= 1
            raise OSError(5, "Input/output error")
        return self._real.read(fd, n)


SHIM = OsShim(os)


class FakeC3:
    """A pty fed like the C3's USB CDC port, published as a symlink."""

    def __init__(self, link=LINK):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.link = link
        self.pts = os.ttyname(self.slave)
        tmp = link + ".tmp"
        try:
            os.unlink(tmp)
        except OSError:
            pass
        os.symlink(self.pts, tmp)
        os.replace(tmp, link)          # atomic repoint = re-enumeration
        self.got = bytearray()

    def pump(self):
        """Drain host->C3 bytes so the host's writes never block."""
        while True:
            try:
                d = os.read(self.master, 4096)
            except (BlockingIOError, OSError):
                return
            if not d:
                return
            self.got += d

    def feed(self, data):
        self.pump()
        try:
            os.write(self.master, data)
        except OSError:
            pass

    def yank(self):
        try:
            os.close(self.master)
        except OSError:
            pass

    def wcount(self, ch):
        self.pump()
        return bytes(self.got).count(ch)


REAL_OPEN = mod.open_serial        # genuine raw-termios open, saved up front


def install(fail_next=0, candidates=None, probe=None):
    """Swap the module's open_serial for the real one plus, optionally, a run of
    failed attempts (a re-enumerating device). `candidates`/`probe` replace the
    port scan so the harness never touches real hardware."""
    st = {"tries": [], "failed": 0}

    def fake_open(port, baud):
        st["tries"].append((port, time.time()))
        if st["failed"] < fail_next:
            st["failed"] += 1
            raise OSError(19, "No such device")
        return REAL_OPEN(port, baud)

    mod.open_serial = fake_open
    if candidates is not None:
        mod._candidate_ports = lambda port=None: list(candidates)
    if probe is not None:
        mod._probe_is_c3 = probe
    return st


def ropen(state):
    return getattr(state, "reader_open", None)


def rcount(state):
    return getattr(state, "reconnects", None)


def shutdown(rdr):
    stop = getattr(rdr, "stop", None)
    if stop is not None:
        stop()
    rdr.join(timeout=3)


def wait(pred, timeout, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return pred()


try:
    os.unlink(LINK)
except OSError:
    pass
mod.os = SHIM
print("module:", MODULE_PATH)

# ---------------------------------------------------------------- [1]
print("\n[1] read error (yanked cable) -> self-reconnect -> frames again")
fake = FakeC3()
install(candidates=[])
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(2.2)                       # console grace, then the reader sends 'g'
fake.feed(frame(10, b"\xff\xd8" + b"A" * 500))
fake.feed(frame(11, b"\xff\xd8" + b"B" * 500))
time.sleep(0.6)
check("frames decoded", state.frames == 2, "frames=%d" % state.frames)
check("reader reports open", ropen(state) is True)
check("'g' sent after the console grace", fake.wcount(b"g") >= 1,
      "host wrote %r" % bytes(fake.got)[:32])

# cable pulled: the fd starts returning EIO; the C3 comes back as a NEW device
fake.yank()
fake2 = FakeC3()                      # same port NAME, different pts
SHIM.eio = 1
check("reader noticed the dead fd",
      wait(lambda: ropen(state) is False, 5))
check("reader thread alive after the read error", rdr.is_alive())
for _ in range(12):
    fake2.feed(frame(12, b"\xff\xd8" + b"C" * 500))
    if state.frames >= 3:
        break
    time.sleep(0.4)
check("frames resumed without a restart", state.frames >= 3,
      "frames=%d" % state.frames)
check("reconnect counted", (rcount(state) or 0) >= 1, "n=%s" % rcount(state))
check("reader open again", ropen(state) is True)
check("'g' re-sent on the new connection", fake2.wcount(b"g") >= 1,
      "host wrote %r" % bytes(fake2.got)[:32])
shutdown(rdr)
check("stop() ends the reader thread", not rdr.is_alive())
check("stop() closes the port", ropen(state) is False)

# ---------------------------------------------------------------- [2]
print("\n[2] the old killer: reopen keeps failing (device re-enumerating)")
fake = FakeC3()
install(candidates=[], fail_next=0)
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(2.2)
fake.feed(frame(20, b"\xff\xd8" + b"D" * 400))
time.sleep(0.6)
check("frames decoded before the outage", state.frames == 1,
      "frames=%d" % state.frames)

os.unlink(LINK)                       # the node is gone: every open fails
fake.yank()
SHIM.eio = 1
check("reader back to waiting", wait(lambda: ropen(state) is False, 5))
for _ in range(8):
    if not rdr.is_alive():
        break
    time.sleep(0.4)
check("reader thread survived the outage", rdr.is_alive())
check("still not open while the node is gone", ropen(state) is False)

fake2 = FakeC3()                      # the device comes back
for _ in range(25):
    fake2.feed(frame(21, b"\xff\xd8" + b"E" * 400))
    if state.frames >= 2:
        break
    time.sleep(0.4)
check("recovered without a restart", state.frames >= 2, "frames=%d" % state.frames)
check("thread still alive after recovery", rdr.is_alive())
shutdown(rdr)

# ---------------------------------------------------------------- [3]
print("\n[3] open but silent port is fd-cycled")
mod.USB_SILENCE_CYCLE_S = 1.0
fake = FakeC3()
ost = install(candidates=[])
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(4.0)
check("silent port was cycled (>1 open call)", len(ost["tries"]) > 1,
      "open()s=%d" % len(ost["tries"]))
check("reader alive through the cycling", rdr.is_alive())
fake.feed(frame(30, b"\xff\xd8" + b"F" * 400))
time.sleep(1.0)
check("frames after the cycle", state.frames >= 1, "frames=%d" % state.frames)
shutdown(rdr)
mod.USB_SILENCE_CYCLE_S = 20.0

# ---------------------------------------------------------------- [4]
print("\n[4] a deliberate PAUSE ('x') survives the 'g' keep-alive")
fake = FakeC3()
install(candidates=[])
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
state.reader = rdr
mod.ViewerHandler.state = state
rdr.start()
time.sleep(2.2)
srv4 = ThreadingHTTPServer(("127.0.0.1", 0), mod.ViewerHandler)
p4 = srv4.server_address[1]
threading.Thread(target=srv4.serve_forever, daemon=True).start()


def ctl(cmd):
    with urllib.request.urlopen("http://127.0.0.1:%d/ctl?cmd=%s" % (p4, cmd),
                                timeout=5) as r:
        return json.loads(r.read())


check("ctl x accepted", ctl("x") == {"ok": True})
check("paused flag set", state.paused is True)
before = fake.wcount(b"g")
time.sleep(3.5)                       # the keep-alive would fire at ~2 s
after = fake.wcount(b"g")
check("paused: keep-alive does NOT resume the stream", after == before,
      "g-count %d -> %d" % (before, after))
check("ctl g accepted", ctl("g") == {"ok": True})
check("START clears paused", state.paused is False)
time.sleep(0.3)
check("START sends 'g'", fake.wcount(b"g") > before)
srv4.shutdown()
shutdown(rdr)

# ---------------------------------------------------------------- [5]
print("\n[5] port renamed (ttyACM0 -> ttyACM1): adopt the C3 on the new node")
try:
    os.unlink(LINK2)
except OSError:
    pass
fake = FakeC3()
install(candidates=[LINK2], probe=lambda fd: True)
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(2.2)
fake.feed(frame(40, b"\xff\xd8" + b"G" * 400))
time.sleep(0.6)
check("frames decoded on the original node", state.frames == 1,
      "frames=%d" % state.frames)

os.unlink(LINK)                       # old node gone for good
fake.yank()
fake2 = FakeC3(link=LINK2)            # the C3 now lives on ttyACM1
SHIM.eio = 1
for _ in range(20):
    fake2.feed(frame(41, b"\xff\xd8" + b"H" * 400))
    if state.frames >= 2:
        break
    time.sleep(0.4)
check("adopted the renamed port", rdr.port == LINK2, "port=%s" % rdr.port)
check("frames resumed on the new node", state.frames >= 2,
      "frames=%d" % state.frames)
check("state.port follows the move", state.port == LINK2, state.port)
shutdown(rdr)
try:
    os.unlink(LINK2)
except OSError:
    pass

# ---------------------------------------------------------------- [6]
print("\n[6] live link line: '[rx] link rssi=...' -> /status + reader wiring")
fake = FakeC3()
install(candidates=[])
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(2.2)
# the C3 prints these as console text BETWEEN frames (that is exactly how
# receiver.cpp flushes them), so feed them the same way the real board does
fake.feed(b"\n[rx] link rssi=-61 ch=1 sta=1\n")
fake.feed(frame(50, b"\xff\xd8" + b"I" * 300))
time.sleep(0.8)
link, age = state.link_snapshot()
check("reader parsed the link line", link is not None and link["rssi"] == -61,
      "link=%r" % (link,))
check("channel parsed", link and link["ch"] == 1, "link=%r" % (link,))
check("associated flag parsed", link and link["sta"] is True)
check("report is fresh", age >= 0 and age < 5, "age=%.1f" % age)
check("the frame after the text still decoded", state.frames == 1,
      "frames=%d" % state.frames)
# a not-yet-associated board reports sta=0
fake.feed(b"\n[rx] link rssi=0 ch=1 sta=0\n")
fake.feed(frame(51, b"\xff\xd8" + b"J" * 300))
time.sleep(0.8)
link, age = state.link_snapshot()
check("sta=0 parsed", link and link["sta"] is False, "link=%r" % (link,))
check("second frame decoded after more text", state.frames == 2,
      "frames=%d" % state.frames)
rdr.stop(); rdr.join(timeout=3)

# ---------------------------------------------------------------- [7]
print("\n[7] HTTP: /status JSON + strict /ctl validation")
state = mod.StreamState()
state.reader_open = True
state.reconnects = 3
state.set_frame(b"\xff\xd8jpeg")
state.set_link("[rx] link rssi=-72 ch=6 sta=1")


class FakeReader:
    def __init__(self):
        self.cmds = []

    def send_command(self, c):
        self.cmds.append(c)
        return True


fr = FakeReader()
state.reader = fr
mod.ViewerHandler.state = state
srv = ThreadingHTTPServer(("127.0.0.1", 0), mod.ViewerHandler)
port = srv.server_address[1]
threading.Thread(target=srv.serve_forever, daemon=True).start()
base = "http://127.0.0.1:%d" % port


def get(path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return r.status, r.read()


st, body = get("/status")
j = json.loads(body)
check("/status valid JSON", st == 200)
check("/status reader field", j.get("reader") == "open", repr(j.get("reader")))
check("/status reconnects field", j.get("reconnects") == 3)
check("/status frame count", j.get("frames") == 1, str(j.get("frames")))
check("/status rssi field", j.get("rssi") == -72, repr(j.get("rssi")))
check("/status rssi channel", j.get("rssi_ch") == 6, repr(j.get("rssi_ch")))
check("/status rssi sta flag", j.get("rssi_sta") is True)
check("/status rssi_age present", isinstance(j.get("rssi_age"), float),
      repr(j.get("rssi_age")))
st, body = get("/")
check("/ serves the dashboard", st == 200 and b"<html" in body)
st, body = get("/ctl?cmd=x")
check("/ctl x ok", json.loads(body) == {"ok": True} and fr.cmds == ["x"],
      "cmds=%r" % (fr.cmds,))
st, body = get("/ctl?cmd=drop%20me!!r")
check("/ctl rejects junk", json.loads(body) == {"ok": False},
      "cmds=%r" % (fr.cmds,))
st, body = get("/ctl?cmd=t30")
check("/ctl relays t30", json.loads(body) == {"ok": True} and "t30" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
# fps-priority mode: an idempotent SET the C3 relays 3x (see src/config.h).
st, body = get("/ctl?cmd=F1")
check("/ctl F1 accepted", json.loads(body) == {"ok": True} and "F1" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
check("/status fps_mode = 1", json.loads(get("/status")[1]).get("fps_mode") == 1,
      repr(json.loads(get("/status")[1]).get("fps_mode")))
st, body = get("/ctl?cmd=F0")
check("/ctl F0 accepted", json.loads(body) == {"ok": True} and "F0" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
check("/status fps_mode = 0", json.loads(get("/status")[1]).get("fps_mode") == 0)
for bad in ("F9", "F", "F1x", "f1"):
    st, body = get("/ctl?cmd=" + bad)
    check("/ctl rejects %r" % bad, json.loads(body) == {"ok": False},
          "cmds=%r" % (fr.cmds,))
check("dashboard wires the FPS button",
      b"fpsToggle()" in get("/")[1] and b"id='fpsbtn'" in get("/")[1])
# The fps cap ('V', 1..60) and AUTO ('A'). The point of qualityManual is that an
# explicit choice is NOT silently reverted by the adaptive up-stepper (that
# revert read as "the MEDIUM button is broken"), so the host must track and
# expose whose choice is in force.
st, body = get("/ctl?cmd=V30")
check("/ctl V30 accepted", json.loads(body) == {"ok": True} and "V30" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
_j = json.loads(get("/status")[1])
check("/status fps_target = 30", _j.get("fps_target") == 30,
      repr(_j.get("fps_target")))
check("/status manual after the fps cap", _j.get("quality_manual") is True)
st, body = get("/ctl?cmd=1")
check("/ctl MED accepted", json.loads(body) == {"ok": True} and "1" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
check("/status manual after a quality click",
      json.loads(get("/status")[1]).get("quality_manual") is True)
st, body = get("/ctl?cmd=A")
check("/ctl A accepted", json.loads(body) == {"ok": True} and "A" in fr.cmds,
      "cmds=%r" % (fr.cmds,))
_j = json.loads(get("/status")[1])
check("/status AUTO clears the fps target", _j.get("fps_target") is None)
check("/status AUTO clears manual", _j.get("quality_manual") is False)
for bad in ("V0", "V61", "V99", "Vx", "V123", "a"):
    st, body = get("/ctl?cmd=" + bad)
    check("/ctl rejects %r" % bad, json.loads(body) == {"ok": False},
          "cmds=%r" % (fr.cmds,))
check("dashboard wires the fps cap input",
      b"setFps()" in get("/")[1] and b"id='fpsval'" in get("/")[1])
check("dashboard wires AUTO", b"ctl('A')" in get("/")[1])
st, body = get("/console")
check("/console reachable", st == 200)
check("/status still valid JSON", json.loads(get("/status")[1]))
srv.shutdown()

try:
    os.unlink(LINK)
except OSError:
    pass

# ---------------------------------------------------------------- [8]
print("\n[8] a port held by someone else reports 'port is locked', not EBADF")
holder = FakeC3()                       # flock the device like a pio monitor would
hfd = REAL_OPEN(holder.link, 115200)
mod.open_serial = REAL_OPEN

held_link = LINK + "-held"
tmpfd = os.open(holder.link, os.O_RDWR | os.O_NOCTTY)
try:
    os.symlink(holder.pts, held_link)
except OSError:
    try:
        os.unlink(held_link)
        os.symlink(holder.pts, held_link)
    except OSError:
        pass
os.close(tmpfd)
try:
    mod.open_serial(held_link, 115200)
    msg = "no exception raised"
except RuntimeError as exc:
    msg = str(exc)
except OSError as exc:
    msg = "OSError: %s" % exc
check("held port raises a RuntimeError, not OSError/EBADF",
      msg.startswith("port is locked"), msg)
os.close(hfd)
# and the port is usable again once the holder goes away
fd = mod.open_serial(held_link, 115200)
check("port opens once the holder is gone", fd >= 0)
os.close(fd)
try:
    os.unlink(held_link)
except OSError:
    pass

# ---------------------------------------------------------------- [9]
print("\n[9] keep-alive: the host re-arms the C3 after it falls back to console")
fake = FakeC3()
install(candidates=[])
state = mod.StreamState()
rdr = mod.SerialReader(LINK, 115200, state)
rdr.start()
time.sleep(2.2)
fake.feed(frame(60, b"\xff\xd8" + b"K" * 300))
fake.feed(frame(61, b"\xff\xd8" + b"L" * 300))
time.sleep(0.6)
check("frames decoded before the stall", state.frames == 2,
      "frames=%d" % state.frames)
before = fake.wcount(b"g")
# The C3 falls back to CONSOLE mode (a stray 'x' from another host instance or a
# monitor exit handler, or its own USB glitch): it keeps printing console text
# but sends no binary frames. The host must notice and re-send 'g'.
# REGRESSION: the old keep-alive tested the CUMULATIVE parser.frames, which is
# non-zero forever after the first frame, so it never fired again once the
# stream had worked - the picture then stayed frozen until a manual restart.
for i in range(16):                      # ~8 s of console-only traffic
    fake.feed(b"\n[rx] frame %d ok (3024 B, 13 seg)\n" % (700 + i))
    time.sleep(0.5)
after = fake.wcount(b"g")
check("keep-alive re-sent 'g' while no frames arrived", after - before >= 2,
      "g-count %d -> %d" % (before, after))
check("reader still alive through the console-only stretch", rdr.is_alive())
fake.feed(frame(62, b"\xff\xd8" + b"M" * 300))
time.sleep(1.0)
check("frames flow again after the re-arm", state.frames >= 3,
      "frames=%d" % state.frames)
check("console text did not corrupt the stream", state.errors == 0,
      "errors=%d" % state.errors)
rdr.stop(); rdr.join(timeout=3)

print("\n%s (%d failure(s))" % ("ALL PASS" if not FAILS else
                                "FAILURES: " + ", ".join(FAILS), len(FAILS)))
sys.exit(1 if FAILS else 0)
