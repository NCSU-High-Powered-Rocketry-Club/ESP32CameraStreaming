#!/usr/bin/env python3
"""c3_usb_mjpeg.py - stdlib-only host viewer for the ESP32-C3 dish receiver.

Reads the binary framed JPEG stream that the C3 forwards over USB CDC,
CRC-validates each frame, and re-serves it as MJPEG over HTTP so any
phone/PC browser can watch.

Architecture (see AI_README.md section 8):
    S3 (airborne) --ESP-NOW--> C3 dish --USB CDC (wired)--> this host --> HTTP MJPEG --> phone/PC

Usage:
    python3 c3_usb_mjpeg.py [--port /dev/ttyACM0] [--baud 115200]
                            [--host 0.0.0.0] [--http-port 8080]

If --port is omitted the first of /dev/ttyACM*, /dev/ttyUSB* or a
/dev/serial/by-id/* entry that opens is used.

Framing (MUST stay in sync with receiver/src/receiver_config.h):
    [0]     magic high  = 0xC3
    [1]     magic low   = 0x8C
    [2..5]  payload byte length (big-endian uint32)
    [6..7]  frame id (big-endian uint16)
    [8..11] CRC-32 (zlib/ISO-HDLC polynomial) of the payload, big-endian
    [..]    payload  (JPEG bytes)

Length-prefix + CRC means we can resync by scanning for the magic whenever
the C3 drops a frame mid-stream (slow host or lost ESP-NOW segment): a CRC
mismatch just logs + skips instead of corrupting the stream.

Only the Python standard library is used (os, termios, zlib, http.server...),
so no pip install is required.

USB CDC modes: the C3 boots in **console mode** (readable banner/log lines, and
the `m`/`s` commands work from `pio device monitor`). This script sends `g`
right after opening the port to switch the C3 into **stream mode** (binary
frames), and sends `x` on exit to hand the console back.
"""
import argparse
import collections
import glob
import math
import os
import socket
import sys
import termios
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --- Framing constants (mirror receiver/src/receiver_config.h) ---------------
MAGIC_HI = 0xC3
MAGIC_LO = 0x8C            # video (JPEG) frames
MAGIC_LO_AUDIO = 0x8D      # audio (8 kHz unsigned 8-bit PCM) packets
HEADER = 12            # magic(2) + length(4) + frame id(2) + crc32(4)
MAX_FRAME = 32 * 1024  # RECEIVER_MAX_FRAME_BYTES on the C3 (no PSRAM)
AUDIO_RATE = 8000      # MIC_STREAM_RATE on the sender (Hz)
BAUD_DEFAULT = 115200  # USB_BAUD / monitor_speed

_BAUD_TERMIOS = {
    9600: termios.B9600, 57600: termios.B57600,
    115200: termios.B115200, 230400: termios.B230400,
    460800: termios.B460800,
}


def _candidate_ports(port=None):
    """Ordered list of serial ports to try, most likely first."""
    if port:
        return [port]
    ports = []
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports += sorted(glob.glob(pat))
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    ports += [os.path.realpath(p) for p in by_id]
    seen, ordered = set(), []
    for p in ports:
        if p not in seen:
            seen.add(p)
            ordered.append(p)
    return ordered


def open_serial(port, baud):
    """Open `port` in raw 8N1 mode without depending on pip's pyserial."""
    speed = _BAUD_TERMIOS.get(int(baud))
    if speed is None:
        raise ValueError("unsupported baud %r" % (baud,))
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    try:
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                   termios.ISTRIP | termios.INLCR | termios.IGNCR |
                   termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                   termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB)
        cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
        ispeed = ospeed = speed
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
        # IMPORTANT: TinyUSB on the C3 treats the port as "not connected"
        # until the host asserts DTR, and silently drops every byte we send
        # in that state (and the C3 sees availableForWrite() == 0). Assert
        # DTR/RTS explicitly so the frame stream and console both work.
        try:
            import fcntl
            import struct
            # Exclusive lock: a second host instance holding the port is the
            # classic cause of mystery 'x' commands (its exit handler kills
            # the other instance's stream mode) and split byte streams.
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(fd, termios.TIOCMBIS,
                        struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
        except BlockingIOError:
            os.close(fd)
            raise RuntimeError("port is locked - another c3_usb_mjpeg.py "
                               "instance is probably still running")
        except Exception:
            pass
    except Exception:
        os.close(fd)
        raise
    return fd


class FrameParser:
    """Stateful resync of the C3's length+CRC framed byte stream."""

    def __init__(self, max_frame=MAX_FRAME):
        self.max_frame = max_frame
        self.buf = bytearray()
        self.frames = 0      # valid frames decoded
        self.skips = 0       # resync EVENTS (corrupt header / garbage run)
        self.bytes = 0       # raw bytes received from the C3
        self.text_bytes = 0  # console-text bytes (banner/hints, not frames)
        self.lines = []      # console text (banner/hints) found between frames
        self.audio_out = []  # audio packets: [(seq, pcm_bytes), ...]
        self.gaps = 0        # MISSING video frame ids (true air loss estimate)
        self.last_vid = None
        self._in_garbage = False
        self._txt = bytearray()

    def _note_text(self, b):
        """Collect printable bytes seen while scanning (C3 console output)."""
        if 0x20 <= b < 0x7F or b in (0x0A, 0x0D):
            self.text_bytes += 1
            self._in_garbage = False
            self._txt.append(b)
            if len(self._txt) >= 120:
                self._flush_text()
        else:
            # Non-printable garbage = a real resync, but count RUNS not bytes:
            # scanning one corrupt 3.5 KB payload used to inflate `skips` by
            # ~1300, which made the drop-rate card read ~99% while the C3 was
            # actually delivering nearly every frame.
            if not self._in_garbage:
                self.skips += 1
                self._in_garbage = True
            self._flush_text()

    def _flush_text(self):
        if len(self._txt) >= 4:
            txt = self._txt.decode("ascii", "replace").strip()
            # Only keep runs that look like REAL console output. Every C3
            # diagnostic starts with the "[rx]" prefix; short printable runs
            # found while scanning binary JPEG/PCM payloads are just random
            # bytes (ypjfgox, sb_n, wdQD?@CEB3 ...) and used to show up as
            # fake lines at the top of the console pane.
            if "[rx]" in txt:
                self.lines.append(txt)
            if len(self.lines) > 64:
                del self.lines[:len(self.lines) - 64]
        self._txt.clear()

    def feed(self, data):
        """Hand a chunk of raw bytes; returns [(frame_id, payload), ...]."""
        self.buf += data
        self.bytes += len(data)
        out = []
        start = 0
        n = len(self.buf)
        while n - start >= HEADER:
            if self.buf[start] != MAGIC_HI or \
                    self.buf[start + 1] not in (MAGIC_LO, MAGIC_LO_AUDIO):
                self._note_text(self.buf[start])   # counts text vs. skips
                start += 1
                continue
            length = int.from_bytes(self.buf[start + 2:start + 6], "big")
            if length == 0 or length > self.max_frame:   # implausible -> resync
                self._note_text(self.buf[start])
                start += 1
                continue
            if n - start < HEADER + length:
                break                                   # need more bytes
            self._flush_text()   # a real header ends any console text run
            frame_id = int.from_bytes(self.buf[start + 6:start + 8], "big")
            crc_stored = int.from_bytes(self.buf[start + 8:start + 12], "big")
            payload = bytes(self.buf[start + HEADER:start + HEADER + length])
            if zlib.crc32(payload) & 0xFFFFFFFF == crc_stored:
                if self.buf[start + 1] == MAGIC_LO_AUDIO:
                    # audio packet: hand off separately, don't touch video stats
                    self.audio_out.append((frame_id, payload))
                    start += HEADER + length
                    continue
                # True air-loss estimate: video frame ids are assigned per
                # captured frame on the S3, so a missing id = a frame the C3
                # never completed (or the host never got). This is what the
                # DROP RATE card shows - NOT the byte-scanning resync count.
                if self.last_vid is not None:
                    gap = (frame_id - 1 - self.last_vid) & 0xFFFF
                    if 0 < gap <= 300:
                        self.gaps += gap
                self.last_vid = frame_id
                out.append((frame_id, payload))
                self.frames += 1
                start += HEADER + length
            else:
                # Dropped / corrupt mid-stream: the payload we read likely also
                # swallowed the NEXT frame's header, so don't skip past it -
                # resync by scanning forward one byte at a time for the magic.
                self.skips += 1
                start += 1
        if start:
            del self.buf[:start]
        return out


class StreamState:
    """Latest validated frame shared between the serial reader and the HTTP
    handler(s), guarded by a lock so one slow HTTP reader never blocks either."""

    def __init__(self):
        self._lock = threading.Lock()
        self._bytes = bytes()
        self._seq = 0
        self.frames = 0
        self.skips = 0
        self.errors = 0
        self.raw_bytes = 0
        self.text_bytes = 0
        self.last_frame_ts = 0.0
        self.log = []
        self.port = "?"
        self.reader = None   # SerialReader once attached (used by /ctl)
        # Rolling per-frame metrics window (host-measured, last ~10 s).
        self.win = collections.deque(maxlen=600)   # (ts, payload_len)
        self.skip_events = collections.deque(maxlen=2000)  # ts of each resync
        # Audio: rolling buffer of (arrival_ts, pcm_bytes) for /audio.pcm
        self.audio = collections.deque(maxlen=600)  # ~ 600*240B ≈ 18 s max
        self.audio_pkts = 0
        self.paused = False   # set by /ctl 'x' / 'g' - distinguishes pause vs dead

    # ESP-NOW air-latency estimate: ~4 ms per 250-byte datagram (send + link
    # ACK + turnaround), plus ~40 ms capture/assemble/USB overhead. A rough
    # but useful figure; the TRUE end-to-end latency also includes the
    # viewer's HTTP fetch (~1 frame interval on a LAN).
    SEG_BYTES = 240
    def _est_air_ms(self, plen):
        segs = max(1, (plen + self.SEG_BYTES - 1) // self.SEG_BYTES)
        return segs * 4.0 + 40.0

    def set_frame(self, payload, fid=None):
        with self._lock:
            self._bytes = payload
            self._seq += 1
            self.frames += 1
            self.last_frame_ts = time.time()
            self.win.append((self.last_frame_ts, len(payload), fid))

    def snapshot(self):
        with self._lock:
            return (self._bytes, self._seq, self.frames,
                    self.skips, self.errors, self.raw_bytes,
                    self.last_frame_ts)

    def add_log(self, line):
        """Keep the last N console lines the C3 printed between frames."""
        with self._lock:
            self.log.append(line)
            if len(self.log) > 30:
                del self.log[:len(self.log) - 30]

    def get_log(self):
        with self._lock:
            return "\n".join(self.log)


class AudioDenoise:
    """Real-time noise suppression for the 8 kHz / 8-bit mic stream.

    Stdlib-only (pure Python, ~1 ms per second of audio):
      * one-pole high-pass (~90 Hz) - kills the PDM mic's DC offset and
        low-frequency rumble that the fixed x8 gain makes very audible;
      * peak-envelope noise gate - while the signal stays below `gate`
        (background hiss), gain eases toward `duck`; any peak above the
        gate opens it instantly (speech) and release takes ~250 ms, so
        word tails are not clipped.

    This is deliberately NOT spectral subtraction - at 8-bit/8 kHz the
    hiss is broadband and a gate + HPF is what makes it sound clean.
    Set gate <= 0 to disable entirely (raw pass-through).
    """

    def __init__(self, rate=AUDIO_RATE, gate=10.0, duck=0.0,
                 hp_hz=90.0, rel_s=0.25, noise_gate=None):
        # `noise_gate` is an alias for `gate` (the CLI flag passes it).
        if noise_gate is not None:
            gate = noise_gate
        self.gate = float(gate)
        self.duck = float(duck)
        self.hp_a = math.exp(-2.0 * math.pi * hp_hz / rate)
        self.rel = 1.0 - math.exp(-1.0 / (rate * rel_s))  # gain release rate
        self.hp_x1 = 0.0
        self.hp_y1 = 0.0
        self.env = 0.0       # peak envelope of the high-passed signal
        self.gain = 1.0

    def process(self, pcm):
        if not self.gate or self.gate <= 0:
            return pcm       # disabled: raw pass-through
        out = bytearray(len(pcm))
        x1 = self.hp_x1
        y1 = self.hp_y1
        env = self.env
        g = self.gain
        for i, b in enumerate(pcm):
            x = b - 128
            y = self.hp_a * (y1 + x - x1)     # DC/rumble high-pass
            x1 = x
            y1 = y
            a = x if x >= 0 else -x
            if a > env:                       # envelope: instant attack
                env = a
            else:                             # exponential decay
                env -= env * self.rel * 0.02  # ~ -6 dB/s slow-ish decay
            if env > self.gate:
                g = 1.0                       # speech: full gain instantly
            else:
                g += (self.duck - g) * self.rel   # fade to ducked level
            s = int(y * g)
            if s > 127:
                s = 127
            elif s < -128:
                s = -128
            out[i] = s + 128
        self.hp_x1 = x1
        self.hp_y1 = y1
        self.env = env
        self.gain = g
        return bytes(out)


class SerialReader(threading.Thread):
    """Reads bytes from the C3's USB CDC port and funnels valid frames to
    `state`. Exits when the port disappears or the user stops the program."""

    def __init__(self, port, baud, state, noise_gate=10.0):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.state = state
        self._fd = None          # set once the port is open (used by /ctl)
        self.denoise = AudioDenoise(noise_gate=noise_gate)

    def send_command(self, chars):
        """Write control chars ('g','x','0'..'2', ...) to the C3. Called from
        the HTTP handler thread."""
        fd = self._fd
        if fd is None:
            return False
        try:
            os.write(fd, chars.encode())
            termios.tcdrain(fd)
            return True
        except (OSError, TypeError):
            return False

    def run(self):
        try:
            parser = FrameParser()
            fd = open_serial(self.port, self.baud)
        except Exception as exc:
            print(f"[serial] cannot open {self.port}: {exc}", flush=True)
            self.state.errors += 1
            return
        self.state.port = self.port
        self._fd = fd
        print(f"[serial] opened {self.port} @ {self.baud} baud (raw 8N1)",
              flush=True)
        last_stall_warn = 0.0
        stream_started = False
        open_ts = time.time()
        try:
            while True:
                try:
                    chunk = os.read(fd, 65536)
                except OSError as exc:
                    # USB re-enumerated (C3 reboot / cable glitch): the fd is
                    # dead. Close it and keep trying to reopen the same port -
                    # this is what makes the stream self-recover after a drop
                    # instead of needing a manual restart.
                    print(f"[serial] read error: {exc} - reopening", flush=True)
                    self.state.errors += 1
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    fd = None
                    self._fd = None
                    time.sleep(1.0)
                    try:
                        fd = open_serial(self.port, self.baud)
                        self._fd = fd
                        stream_started = False
                        open_ts = time.time()
                        print(f"[serial] reopened {self.port}", flush=True)
                    except Exception as exc2:
                        print(f"[serial] reopen failed: {exc2}", flush=True)
                    continue
                if not chunk:
                    time.sleep(0.002)
                else:
                    try:
                        for _fid, payload in parser.feed(chunk):
                            self.state.set_frame(payload, _fid)
                        if parser.audio_out:
                            now_a = time.time()
                            with self.state._lock:
                                for _s, apcm in parser.audio_out:
                                    # Lost audio datagrams used to leave a
                                    # hole -> clicks / choppy playback. Each
                                    # datagram is exactly 30 ms (240 B @ 8 kHz)
                                    # and carries a seq, so fill short gaps
                                    # with mid-level silence to keep the
                                    # timeline continuous (loss concealment).
                                    last = getattr(self, "_a_seq", None)
                                    if last is not None:
                                        gap = (_s - 1 - last) & 0xFFFF
                                        if 0 < gap <= 25:
                                            self.state.audio.append(
                                                (now_a, b"\x80" * (240 * gap)))
                                    self._a_seq = _s
                                    # Noise suppression (HPF + gate) before
                                    # the chunk is served to any listener.
                                    self.state.audio.append(
                                        (now_a, self.denoise.process(apcm)))
                                    self.state.audio_pkts += 1
                            parser.audio_out = []
                        with self.state._lock:
                            # Skip events: record a timestamp for each NEW
                            # resync so /status can show a RECENT drop rate
                            # (the cumulative counter alone dilutes over time
                            # and looks like it "isn't updating").
                            if parser.skips > self.state.skips:
                                for _ in range(min(parser.skips - self.state.skips,
                                                   500)):
                                    self.state.skip_events.append(time.time())
                            self.state.skips = parser.skips
                            self.state.frames = parser.frames
                            self.state.raw_bytes = parser.bytes
                            self.state.text_bytes = parser.text_bytes
                        # Echo anything the C3 printed between frames (banner,
                        # hints) so its console is visible in this terminal and
                        # on the web page.
                        while parser.lines:
                            line = parser.lines.pop(0)
                            print(f"[c3] {line}", flush=True)
                            self.state.add_log(line)
                    except Exception as exc:   # never let a bad chunk kill us
                        print(f"[serial] parse error: {exc}", flush=True)
                        self.state.errors += 1

                # Stay in console mode for a moment first: the C3 greets a new
                # USB host with its banner + MAC, which we relay - on-page
                # proof of which board/port we are attached to.
                if not stream_started and time.time() - open_ts >= 1.5:
                    stream_started = True
                    os.write(fd, b"g")
                    try:
                        termios.tcdrain(fd)
                    except Exception:
                        pass
                    last_frame_seen = time.time()

                # Keep-alive: the C3 falls back to CONSOLE mode whenever its
                # USB host re-enumerates (C3 reboot, cable glitch, S3 link
                # loss causing a board reset...). It only re-enters stream
                # mode when it sees 'g' again. Re-send 'g' periodically while
                # no frame has been decoded for a while - it is harmless in
                # stream mode (handleCommand just sets the same flag).
                if parser.frames == 0 or \
                        time.time() - getattr(self, "_last_frame_ok", 0) > 3.0:
                    if time.time() - getattr(self, "_last_g", 0) > 2.0:
                        self._last_frame_ok = getattr(self, "_last_frame_ok", 0)
                        self._last_g = time.time()
                        try:
                            os.write(fd, b"g")
                        except OSError:
                            pass
                if parser.frames:
                    self._last_frame_ok = time.time()
                    self._last_g = time.time()

                # Distinguish "no sender at all" from "bytes flowing but no
                # complete frame" - the latter usually means the S3 stopped
                # mid-stream (e.g. browned out) or the link is dropping.
                now = time.time()
                age = now - (self.state.last_frame_ts or now)
                if parser.bytes > 0 and parser.frames == 0 and age > 10 \
                        and now - last_stall_warn > 10:
                    last_stall_warn = now
                    print(f"[serial] {parser.bytes} bytes seen but 0 valid "
                          f"frames in {age:.0f}s - sender likely stopped "
                          f"mid-stream (check the S3's power/serial log)",
                          flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            # Politely hand the USB port back to console mode (m/s in the
            # pio monitor work again) before we close it.
            try:
                os.write(fd, b"x")
                termios.tcdrain(fd)
            except Exception:
                pass
            try:
                self._fd = None
                os.close(fd)
            except OSError:
                pass


INDEX_HTML = """<!DOCTYPE html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP32-C3 dish receiver</title>
<style>
:root{--bg:#0b0f14;--card:#121820;--line:#1f2a36;--tx:#dbe6f1;--dim:#7d8ea0;
      --ok:#4ade80;--warn:#fbbf24;--bad:#f87171;--acc:#38bdf8}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--tx);font:14px/1.5 system-ui,sans-serif;
     margin:0;padding:16px;max-width:1100px;margin:0 auto}
h1{font-size:18px;margin:0 0 12px;display:flex;align-items:center;gap:10px}
#dot{width:10px;height:10px;border-radius:50%;background:var(--bad);
     box-shadow:0 0 6px var(--bad);transition:all .3s}
#dot.ok{background:var(--ok);box-shadow:0 0 6px var(--ok)}
img{width:100%;max-height:60vh;object-fit:contain;background:#000;
    border:1px solid var(--line);border-radius:8px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
      gap:10px;margin:12px 0}
.card{background:var(--card);border:1px solid var(--line);border-radius:8px;
      padding:10px 12px}
.card .v{font-size:22px;font-weight:600;color:var(--acc)}
.card .l{font-size:11px;color:var(--dim);text-transform:uppercase;
         letter-spacing:.05em;margin-top:2px}
.card .s{font-size:10px;color:var(--dim)}
canvas{width:100%;height:60px;display:block;margin-top:6px}
h3{font-size:13px;color:var(--dim);margin:16px 0 6px}
pre{font:11px/1.45 monospace;color:#7c7;background:var(--card);
    border:1px solid var(--line);border-radius:8px;padding:8px;
    max-height:11em;overflow:auto;white-space:pre-wrap}
button{background:var(--card);color:var(--tx);border:1px solid var(--acc);
       border-radius:6px;padding:6px 12px;font-size:13px;cursor:pointer}
button:hover{background:var(--acc);color:#0b0f14}
.foot{font-size:11px;color:var(--dim);margin-top:8px}
</style></head><body>
<h1><span id='dot'></span>ESP32-C3 dish receiver <span style='color:var(--dim);
font-size:12px' id='port'></span></h1>
<img src='/stream' id='v'>
<div class='grid'>
  <div class='card'><div class='v' id='c_fps'>–</div><div class='l'>FPS (10s avg)</div>
    <canvas id='g_fps'></canvas></div>
  <div class='card'><div class='v' id='c_bw'>–</div><div class='l'>BANDWIDTH</div>
    <div class='s'>MJPEG over USB</div></div>
  <div class='card'><div class='v' id='c_lat'>–</div><div class='l'>EST. LATENCY (air)</div>
    <div class='s'>segs × 4 ms + overhead</div></div>
  <div class='card'><div class='v' id='c_gap'>–</div><div class='l'>FRAME INTERVAL med/p95</div>
    <div class='s'>jitter indicator</div></div>
  <div class='card'><div class='v' id='c_drop'>–</div><div class='l'>DROP RATE</div>
    <div class='s'>air frames lost (frame-id gaps)</div></div>
  <div class='card'><div class='v' id='c_size'>–</div><div class='l'>AVG FRAME</div>
    <div class='s' id='c_tot'></div></div>
</div>
<h3>Controls</h3>
<div class='grid'>
  <div class='card' style='text-align:center'><div class='l'>QUALITY (S3)</div>
    <div style='display:flex;gap:6px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('0')">LOW</button>
      <button onclick="ctl('1')">MED</button>
      <button onclick="ctl('2')">HIGH*</button>
    </div><div class='s'>*capped at MEDIUM on ESP-NOW</div></div>
  <div class='card' style='text-align:center'><div class='l'>USB STREAM</div>
    <div style='display:flex;gap:8px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('g')">▶ START</button>
      <button onclick="ctl('x')">⏸ PAUSE</button>
      <button onclick="ctl('M')">🎤 MIC</button>
    </div><div class='s' id='ctlmsg'></div></div>
</div>
<div class='card' style='margin-bottom:12px'><div class='l'>MICROPHONE (S3, 8 kHz)</div>
  <div style='margin-top:6px'><button onclick='audioToggle()' id='abtn'>🔈 LISTEN</button>
  <span class='s' id='astat' style='margin-left:8px'></span></div>
  <div class='s'>Press 🎤 MIC to toggle capture on the sender, then LISTEN here.
  (Web Audio API: raw PCM from /audio.pcm is scheduled sample-accurately on an
  AudioContext — the old endless-WAV &lt;audio&gt; tag only played after the
  stream ended because browsers buffer unknown-length wav files.)</div>
</div>
<h3>C3 serial console</h3>
<pre id='lg'>(not captured yet)</pre>
<script>
var hist=[];   // recent fps samples for the sparkline
function fmt(n,u){return n==null?'–':(Math.round(n*10)/10)+u}
// --- Real-time audio via Web Audio API --------------------------------------
// /audio.pcm streams raw 8 kHz unsigned 8-bit PCM. Chunks are decoded into
// AudioBuffers and scheduled sample-accurately on an AudioContext, so playback
// is continuous and live (no wav-header buffering deadlock).
var actx=null, aNext=0, aPlaying=false, aReader=null;
function audioToggle(){
  if(aPlaying){ aPlaying=false; try{aReader.cancel();}catch(e){}
    document.getElementById('abtn').textContent='🔈 LISTEN'; return; }
  if(!actx) actx=new (window.AudioContext||window.webkitAudioContext)();
  if(actx.state==='suspended') actx.resume();
  aPlaying=true; aNext=actx.currentTime+0.15;
  document.getElementById('abtn').textContent='⏹ STOP';
  fetch('/audio.pcm',{cache:'no-store'}).then(function(r){
    aReader=r.body.getReader();
    function pump(){
      if(!aPlaying) return;
      aReader.read().then(function(res){
        if(res.done||!aPlaying){aPlaying=false;
          document.getElementById('abtn').textContent='🔈 LISTEN';return;}
        if(res.value&&res.value.length){
          var n=res.value.length;
          var buf=actx.createBuffer(1,n,8000);
          var d=buf.getChannelData(0);
          for(var i=0;i<n;i++) d[i]=(res.value[i]-128)/128;
          // If we fell behind (tab was hidden), skip the backlog instead of
          // fast-forwarding a burst of stale audio.
          if(aNext<actx.currentTime-0.3) aNext=actx.currentTime+0.05;
          var src=actx.createBufferSource();
          src.buffer=buf; src.connect(actx.destination);
          src.start(aNext); aNext+=n/8000;
        }
        pump();
      }).catch(function(){aPlaying=false;
        document.getElementById('abtn').textContent='🔈 LISTEN';});
    }
    pump();
  }).catch(function(){aPlaying=false;
    document.getElementById('abtn').textContent='🔈 LISTEN';});
}
setInterval(function(){
  fetch('/status',{cache:'no-store'}).then(function(r){return r.json()}).then(function(s){
    var live = s.last_age >= 0 && s.last_age < 3;
    var paused = !!s.paused;
    document.getElementById('dot').className = live?'ok':'';
    document.getElementById('c_fps').textContent = live?s.fps.toFixed(1):'–';
    document.getElementById('c_bw').textContent  = live?Math.round(s.kbps)+' Kbps':'–';
    document.getElementById('c_lat').textContent = live?Math.round(s.est_air_ms)+' ms':'–';
    document.getElementById('c_gap').textContent = live?
        Math.round(s.gap_med_ms)+' / '+Math.round(s.gap_p95_ms)+' ms':'–';
    // Drop rate: '–' when NOTHING at all arrived in the window (0.0% there
    // read as "healthy" and was misleading).
    var d=document.getElementById('c_drop');
    d.textContent = s.drop_pct==null ? '–' : s.drop_pct.toFixed(1)+'%';
    d.style.color = s.drop_pct==null?'var(--dim)':
        (s.drop_pct<2?'var(--ok)':(s.drop_pct<15?'var(--warn)':'var(--bad)'));
    document.getElementById('c_size').textContent =
        s.avg_frame ? (s.avg_frame/1024).toFixed(1)+' KB' : '–';
    document.getElementById('c_tot').textContent =
        'frames='+s.frames+' skips='+s.skips+' errs='+s.errors;
    document.getElementById('port').textContent = '@ '+s.port+
        (paused?'  ⏸ paused (button)':'');
    hist.push(live?s.fps:0); if(hist.length>120) hist.shift();
    draw('g_fps',hist);
  }).catch(function(){});
  fetch('/console',{cache:'no-store'})
    .then(function(r){return r.text()})
    .then(function(t){ if(t) document.getElementById('lg').textContent = t; })
    .catch(function(){});
},1000);
function ctl(c){
  fetch('/ctl?cmd='+c,{cache:'no-store'}).then(function(r){return r.json()})
    .then(function(j){
      var m=document.getElementById('ctlmsg');
      m.textContent = j.ok ? 'sent "'+c+'" to C3' : 'port not open';
      setTimeout(function(){m.textContent=''},2000);
    }).catch(function(){});
}
function draw(id,data){
  var c=document.getElementById(id),g=c.getContext('2d');
  c.width=c.clientWidth;c.height=60;
  g=g.getContext('2d');g.clearRect(0,0,c.width,c.height);
  g.strokeStyle='#38bdf8';g.lineWidth=1.5;g.beginPath();
  var mx=Math.max(5,Math.max.apply(null,data));
  data.forEach(function(v,i){
    var x=i*c.width/Math.max(1,data.length-1), y=c.height-2-(v/mx)*(c.height-6);
    i?g.lineTo(x,y):g.moveTo(x,y);});
  g.stroke();
}
</script></body></html>"""


class ViewerHandler(BaseHTTPRequestHandler):
    state = StreamState()

    def log_message(self, fmt, *args):          # quiet, unless useful
        print("[http] " + (fmt % args), flush=True)

    def _send_headers(self, ctype, length=None):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/" or path == "/index.html":
            body = INDEX_HTML.encode()
            self._send_headers("text/html; charset=utf-8", len(body))
            self.wfile.write(body)
        elif path == "/status":
            st = self.state.snapshot()   # bytes,seq,frames,skips,errs,raw,last_ts
            age = (time.time() - st[6]) if st[6] else -1
            now = time.time()
            with self.state._lock:
                win = list(self.state.win)[-600:]
            # Metrics over the trailing ~10 s of validated frames.
            recent = [w for w in win if now - w[0] <= 10.0]
            fps = (len(recent) / 10.0) if recent else 0.0
            kbps = (sum(w[1] for w in recent) * 8 / 1024.0 / 10.0) if recent else 0.0
            avg_b = (sum(w[1] for w in recent) / len(recent)) if recent else 0
            gaps = [b[0] - a[0] for a, b in zip(win, win[1:]) if b[0] - a[0] < 5]
            gaps.sort()
            if gaps:
                med = gaps[len(gaps) // 2] * 1000.0
                p95 = gaps[min(len(gaps) - 1, int(len(gaps) * 0.95))] * 1000.0
            else:
                med = p95 = 0.0
            est_lat = self.state._est_air_ms(avg_b) if avg_b else 0.0
            # RECENT drop rate: skip events in the last 10 s vs. frames decoded
            # in the same window (lifetime counters dilute and look frozen).
            # TRUE drop rate: missing video frame-id gaps among the frames
            # decoded in the last 10 s (sender assigns one id per captured
            # frame, so a gap = a frame lost on the air / never forwarded).
            # The old skip-event metric counted garbage BYTES during parser
            # resync and read ~99% while the link was actually mostly fine.
            recent_fids = [w[2] for w in recent if w[2] is not None]
            gaps_total = 0
            prev = None
            for fid in recent_fids:
                if prev is not None:
                    d = (fid - prev - 1) & 0xFFFF
                    if 0 < d <= 300:
                        gaps_total += d
                prev = fid
            total_recent = gaps_total + len(recent_fids)
            # drop_pct = None when NOTHING at all is arriving in the window -
            # reporting 0.0% there read as "healthy" which was misleading.
            drop_pct = None
            if total_recent:
                drop_pct = 100.0 * gaps_total / total_recent
            body = ('{"seq":%d,"frames":%d,"skips":%d,"errors":%d,'
                    '"bytes":%d,"text":%d,"last_age":%.1f,"port":"%s",'
                    '"fps":%.1f,"kbps":%.0f,"avg_frame":%d,'
                    '"est_air_ms":%.0f,"gap_med_ms":%.0f,"gap_p95_ms":%.0f,'
                    '"paused":%s,"drop_pct":%s}'
                    % (st[1], st[2], st[3], st[4], st[5],
                       getattr(self.state, "text_bytes", 0), age,
                       self.state.port, fps, kbps, avg_b, est_lat,
                       med, p95,
                       "true" if self.state.paused else "false",
                       "null" if drop_pct is None else "%.1f" % drop_pct)).encode()
            self._send_headers("application/json", len(body))
            self.wfile.write(body)
        elif path == "/console":
            body = self.state.get_log().encode("utf-8")
            self._send_headers("text/plain; charset=utf-8", len(body))
            self.wfile.write(body)
        elif path == "/stream":
            self._stream_mjpeg()
        elif path == "/audio.pcm" or path == "/audio.wav":
            # /audio.wav kept as an alias for older bookmarks, but both now
            # serve RAW PCM - the Web Audio API on the page decodes it (the
            # old endless WAV header made browsers buffer until close).
            self._stream_audio()
        elif path == "/ctl":
            # Dashboard controls: /ctl?cmd=g|g/x pause etc. Allowed chars only.
            from urllib.parse import urlparse, parse_qs, unquote
            qs = parse_qs(urlparse(self.path).query)
            cmd = unquote(qs.get("cmd", [""])[0])
            clean = "".join(c for c in cmd if c in "gx012M")
            ok = bool(clean) and self.state.reader and \
                 self.state.reader.send_command(clean)
            # Track pause state so /status can tell the dashboard "stream
            # paused on purpose" apart from "link dead" (issue #4).
            if ok and clean == "g":
                self.state.paused = False     # START resumes the stream
            elif ok and "x" in clean:
                self.state.paused = True      # PAUSE is deliberate
            body = b'{"ok":' + (b"true" if ok else b"false") + b"}"
            self._send_headers("application/json", len(body))
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def _stream_audio(self):
        """Raw PCM stream for the browser's Web Audio API (see INDEX_HTML).

        WHY NOT WAV: /audio.wav used to write a 44-byte RIFF header with huge
        sizes and stream forever - Chrome/Firefox refuse to start playback of
        a wav whose length is unknown until the connection CLOSES, which is
        exactly why audio only played after the script was terminated. Raw
        PCM chunks fed into an AudioContext play in real time with no such
        buffering.

        Each chunk is released LIVE: a new connect flushes anything older than
        0.5 s, and chunks still older than 0.3 s while streaming are dropped
        so backlog never turns into a fast-forwarded burst of static.
        """
        self.send_response(200)
        self.send_header("Content-Type", "audio/octet-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            # Flush stale backlog so playback starts LIVE.
            with self.state._lock:
                now = time.time()
                while self.state.audio and now - self.state.audio[0][0] > 0.5:
                    self.state.audio.popleft()
            while True:
                with self.state._lock:
                    item = self.state.audio[0] if self.state.audio else None
                if item is None:
                    time.sleep(0.01)
                    continue
                ts, pcm = item
                if time.time() - ts > 0.3:      # long-stalled client: resync
                    self.state.audio.popleft()
                    continue
                if ts > time.time():            # not due yet (0.15 s jitter buffer)
                    time.sleep(0.005)
                    continue
                self.state.audio.popleft()
                self.wfile.write(pcm)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def _stream_mjpeg(self):
        """MJPEG multipart handler; keeps serving the newest frame."""
        boundary = "frame"
        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=" + boundary)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                st = self.state.snapshot()
                payload, seq = st[0], st[1]
                if seq != last_seq and payload:
                    last_seq = seq
                    hdr = ("--%s\r\nContent-Type: image/jpeg\r\n"
                           "Content-Length: %d\r\n\r\n" % (boundary, len(payload)))
                    self.wfile.write(hdr.encode())
                    self.wfile.write(payload)
                    self.wfile.write(b"\r\n")
                else:
                    time.sleep(0.01)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        except Exception:
            pass


def _probe_is_c3(fd):
    """Ask a port 'who are you?'. Both boards answer 's', but only the C3
    replies with 'latest frame ...' - so this tells the S3 and the C3 apart
    when several boards share one machine. 'x' first in case the C3 was left
    in stream mode by a previous (crashed) run of this script."""
    try:
        termios.tcflush(fd, termios.TCIFLUSH)
        os.write(fd, b"x")
        time.sleep(0.15)
        termios.tcflush(fd, termios.TCIFLUSH)
        os.write(fd, b"s")
        reply = b""
        deadline = time.time() + 1.0
        while time.time() < deadline:
            try:
                reply += os.read(fd, 4096)
            except OSError:
                break
            if b"latest frame" in reply:
                return True
            time.sleep(0.02)
    except Exception:
        pass
    return False


def main():
    ap = argparse.ArgumentParser(
        description="MJPEG host viewer for the ESP32-C3 dish receiver over USB.")
    ap.add_argument("--port", default=None,
                    help="C3 USB CDC device, e.g. /dev/ttyACM0 "
                         "(default: auto-detect)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT,
                    help="serial baud (USB-CDC ignores it; kept for parity)")
    ap.add_argument("--host", default="", help="HTTP bind host (default all)")
    ap.add_argument("--http-port", type=int, default=8080,
                    help="HTTP/MJPEG port (default 8080)")
    ap.add_argument("--noise-gate", type=float, default=10.0,
                    help="mic noise gate: peak amplitude below which the "
                         "audio is faded toward silence (0 disables; "
                         "default 10; raise if hiss still audible)")
    args = ap.parse_args()

    state = ViewerHandler.state

    # Attach the reader to the port that actually IS the C3. Both boards answer
    # 's', so a blind "first /dev/ttyACM*" pick can grab the S3 when several
    # boards share one machine - probe instead, and fall back if none answers.
    reader = None
    fallback_port = None
    for port in _candidate_ports(args.port):
        try:
            fd = open_serial(port, args.baud)
        except Exception as exc:
            print(f"[serial] {port}: {exc} (skipping)", flush=True)
            continue
        if fallback_port is None:
            fallback_port = port
        if _probe_is_c3(fd):
            print(f"[serial] C3 identified on {port} (answered 's')",
                  flush=True)
            os.close(fd)                  # released so the reader can open it
            reader = SerialReader(port, args.baud, state,
                                  noise_gate=args.noise_gate)
            break
        print(f"[serial] {port}: no C3-style answer to 's' "
              f"(another board? skipping)", flush=True)
        os.close(fd)

    if reader is None:
        if fallback_port is None:
            print("[serial] no usable C3 port found. Plug the C3 in over USB "
                  "and try again (or pass --port).", file=sys.stderr)
            return 1
        print(f"[serial] no port answered like a C3; falling back to "
              f"{fallback_port} (pass --port to override)", flush=True)
        reader = SerialReader(fallback_port, args.baud, state,
                              noise_gate=args.noise_gate)

    # Serve HTTP first so the status page is reachable even before frames flow.
    state.reader = reader            # enables the /ctl dashboard controls
    httpd = ThreadingHTTPServer((args.host, args.http_port), ViewerHandler)
    print(f"[http] MJPEG server at http://{_lan_ip()}:{args.http_port}/ "
          f"(stream: /stream)", flush=True)
    try:
        reader.start()
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[stop] shutting down", flush=True)
    finally:
        httpd.shutdown()
        httpd.server_close()
    return 0


def _lan_ip():
    """Best-effort LAN IP for the startup banner (hostname is fine as fallback)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


if __name__ == "__main__":
    sys.exit(main())