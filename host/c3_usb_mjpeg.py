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

Recovery (why this script no longer needs restarting): the serial reader thread
reconnects forever instead of exiting.
  * read error (unplug, C3 reboot, USB re-enumeration) -> close the fd,
    reconnect;
  * device node gone/renamed (ttyACM0 -> ttyACM1) -> rescan and adopt the port
    that answers the C3 probe (`pass --port` to pin one);
  * port open but zero bytes for USB_SILENCE_CYCLE_S -> cycle the fd, which is
    the only thing that clears a wedged host-side CDC endpoint;
  * port held by a stale instance -> the 'port is locked' message names the
    holding pid;
  * a deliberate PAUSE (`x` from the dashboard) is respected: the keep-alive
    'g' and the silence cycle are both skipped while paused.
The dashboard header shows `⟳ reconnecting…` (with the reconnect count) while
the reader is between connections.

Dashboard controls (HTTP `/ctl?cmd=...` -> the C3's serial console -> ESP-NOW
broadcast -> the S3 sender). Single chars are written to the C3 as-is:
    g / x        start / pause the USB frame stream
    0 / 1 / 2    video quality preset (LOW / MED / HIGH)
    M            toggle the sender's PDM microphone
    r / e        start / stop the microSD recording ON THE SENDER   <- new
    t<secs>      max clip length in seconds (t0 = until stopped)    <- new
    p<0..2>      SD/stream priority (0 = SD first, 1 = balanced, 2 = stream) <- new
The sender reports its recording state back over ESP-NOW; the C3 prints it as an
`[rx] REC state=... idx=... frames=... left=... prio=... sd=...` line, which
this script shows in the console pane AND parses into the SD-recording card.

Live link signal: the C3 also prints `[rx] link rssi=<dBm> ch=<n> sta=<0|1>`
about once a second (WiFi.RSSI()/channel of the S3's AP - the same S3 and the
same channel the ESP-NOW video crosses), which feeds the dashboard's SIGNAL
card. The C3 can only report the AP-level figure: this Arduino core's
`esp_now_recv_cb_t` carries no `rx_ctrl`, so per-datagram RSSI is not available
(see receiver/src/receiver.cpp -> reportLink(), keep the format in sync).
"""
import argparse
import collections
import glob
import math
import os
import re
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
# Recovery timings (see SerialReader.run). A USB CDC port that was closed by an
# unplug/replug, a C3 reboot or a wedged host endpoint can only be revived by
# CLOSING and REOPENING the device - the reader therefore never gives up and
# never lets a dead fd kill its thread ("sometimes I have to restart it").
USB_RECONNECT_DELAY = 1.0    # s between port-open attempts
USB_SILENCE_CYCLE_S = 20.0   # open but zero bytes for this long -> cycle port
# Mirrors RECORD_MAX_SECONDS in src/config.h - only used to prefill the
# dashboard's "max clip length" field so it matches the firmware default.
REC_MAX_SECONDS_DEFAULT_TEXT = "30"

_BAUD_TERMIOS = {
    9600: termios.B9600, 57600: termios.B57600,
    115200: termios.B115200, 230400: termios.B230400,
    460800: termios.B460800,
}

# The C3 prints one of these for every recording-status datagram it receives from
# the S3 sender (see src/stream_espnow.cpp -> sendRecordStatus()). The dashboard
# parses it so the SD-recording card shows the REAL state of the sender.
REC_LINE_RE = re.compile(
    r"\[rx\] REC state=(\w+) idx=(\d+) frames=(\d+) left=(\S+) prio=(\d) sd=(\d)")

# Live link report from the C3 (receiver/src/receiver.cpp -> reportLink()):
# WiFi.RSSI()/channel of the AP it is associated with, which is the same S3 and
# the same channel the ESP-NOW video crosses. Keep in sync with the firmware.
# (Per-DATAGRAM RSSI is not available: this Arduino core's esp_now_recv_cb_t
# carries no rx_ctrl, so the AP-level figure is what the C3 can report.)
LINK_LINE_RE = re.compile(r"\[rx\] link rssi=(-?\d+) ch=(\d+) sta=(\d)")


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


def _port_holder(port):
    """Best-effort (pid, cmdline) of the process holding `port` open.

    Only /proc is used (no pyserial, no lsof). Used to turn the 'port is
    locked' failure into an actionable message: a stale c3_usb_mjpeg.py from a
    previous run keeps both the tty AND (via flock) the port, and that is the
    usual reason a fresh run "needs a restart" to work.
    """
    try:
        real = os.path.realpath(port)
    except OSError:
        real = port
    me = os.getpid()
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return None
    for pid in pids:
        if int(pid) == me:
            continue
        try:
            fds = os.listdir("/proc/%s/fd" % pid)
        except OSError:
            continue
        for f in fds:
            try:
                if os.readlink("/proc/%s/fd/%s" % (pid, f)) != real:
                    continue
                with open("/proc/%s/cmdline" % pid, "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode("utf-8",
                                                                "replace")
            except OSError:
                continue
            return int(pid), (cmd.strip() or "?")
    return None


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
            # Already closed here, so clear the fd: the outer handler must not
            # close it a second time - double-closing raised OSError(EBADF),
            # which REPLACED this RuntimeError and made a held port look like
            # the useless "[Errno 9] Bad file descriptor (skipping)".
            os.close(fd)
            fd = -1
            who = _port_holder(port)
            detail = (" (held by pid %d: %s)" % who) if who else ""
            raise RuntimeError("port is locked%s - another c3_usb_mjpeg.py "
                               "or a serial monitor is holding it" % detail)
        except Exception:
            pass
    except Exception:
        if fd >= 0:
            try:
                os.close(fd)
            except OSError:
                pass
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
        # Serial link health: set by SerialReader. `reader_open=False` means we
        # are between reconnects (unplugged / re-enumerating / locked port) -
        # the dashboard shows that instead of silently looking "dead".
        self.reader_open = False
        self.reconnects = 0
        # Rolling per-frame metrics window (host-measured, last ~10 s).
        self.win = collections.deque(maxlen=600)   # (ts, payload_len)
        self.skip_events = collections.deque(maxlen=2000)  # ts of each resync
        # Audio: rolling buffer of (arrival_ts, pcm_bytes) for /audio.pcm
        self.audio = collections.deque(maxlen=600)  # ~ 600*240B ≈ 18 s max
        self.audio_pkts = 0
        self.paused = False   # set by /ctl 'x' / 'g' - distinguishes pause vs dead
        self.pause_ts = 0.0   # when PAUSE was requested (grace for in-flight frames)
        # SD recording on the SENDER, as reported by the C3's "[rx] REC ..."
        # lines (the S3 -> C3 leg is one-way ESP-NOW, so this is the only
        # feedback path). None = nothing heard yet.
        self.rec = None             # dict(on, idx, frames, left, prio, sd)
        self.rec_ts = 0.0           # when the last report arrived
        # What the dashboard last ASKED for (the S3 does not echo settings back
        # separately): shown next to the buttons so it is clear what was sent.
        self.rec_set = {"seconds": REC_MAX_SECONDS_DEFAULT_TEXT, "prio": None}
        # fps-priority mode / frame-rate cap / manual-quality as last requested
        # by the dashboard (the S3 does not echo settings back - same convention
        # as rec_set). None = never asked.
        self.fps_set = None
        self.fps_target = None
        self.quality_manual = False
        # Live link level from the C3's "[rx] link rssi=..." lines (see
        # LINK_LINE_RE). None = nothing heard yet.
        self.link = None            # dict(rssi, ch, sta)
        self.link_ts = 0.0

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
            # Frames are arriving again, so any earlier 'x' (PAUSE button, or a
            # previous crashed instance's exit handler) is not in effect any
            # more - otherwise the header would claim "paused" forever. Brief
            # grace window: bytes already queued on the USB side when PAUSE was
            # pressed still decode as one or two frames, and those must not
            # cancel a pause that DID take effect on the C3.
            if self.paused and time.time() - self.pause_ts > 2.0:
                self.paused = False
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

    def set_rec(self, line):
        """Parse a C3 '[rx] REC ...' line into the sender's recording state.
        Returns True when the line was one of ours (it is still logged)."""
        m = REC_LINE_RE.search(line)
        if not m:
            return False
        on, idx, frames, left, prio, sd = m.groups()
        # Sanitise for the JSON in /status (the left field is free text:
        # "-", "unlimited" or "17s").
        left = re.sub(r"[^0-9A-Za-z-]", "", left)[:12]
        with self._lock:
            self.rec = {"on": on == "on", "idx": int(idx), "frames": int(frames),
                        "left": left, "prio": int(prio), "sd": sd == "1"}
            self.rec_ts = time.time()
        return True

    def rec_snapshot(self):
        """(recording state dict or None, seconds since the last report)."""
        with self._lock:
            rec = dict(self.rec) if self.rec else None
            age = (time.time() - self.rec_ts) if self.rec_ts else -1.0
        return rec, age

    def set_link(self, line):
        """Parse a C3 '[rx] link rssi=... ' line. Returns True when it matched
        (the line is logged either way)."""
        m = LINK_LINE_RE.search(line)
        if not m:
            return False
        rssi, ch, sta = m.groups()
        with self._lock:
            self.link = {"rssi": int(rssi), "ch": int(ch), "sta": sta == "1"}
            self.link_ts = time.time()
        return True

    def link_snapshot(self):
        """(link dict or None, seconds since the last report)."""
        with self._lock:
            link = dict(self.link) if self.link else None
            age = (time.time() - self.link_ts) if self.link_ts else -1.0
        return link, age


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
    `state`.

    This thread NEVER exits on its own (only on KeyboardInterrupt). A vanished
    device node, a failed reopen, a wedged endpoint or a port held by a stale
    instance all just return it to the connect loop, so the viewer recovers by
    itself instead of needing the script to be restarted:
      * read error (unplug / re-enumeration / C3 reboot) -> close the fd and
        reconnect;
      * device node gone or renamed (ttyACM0 -> ttyACM1) -> rescan and adopt the
        port that answers like a C3;
      * port open but completely silent for USB_SILENCE_CYCLE_S -> cycle the fd
        (a wedged host-side CDC endpoint only clears on close/reopen);
      * every write to the fd goes through _write(), which swallows OSError and
        the fd-is-None case - a control write must never kill the thread.
    """

    def __init__(self, port, baud, state, noise_gate=10.0):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.state = state
        self._fd = None          # set once the port is open (used by /ctl)
        self.denoise = AudioDenoise(noise_gate=noise_gate)
        self._last_frame_ok = 0.0
        self._last_g = 0.0
        self._a_seq = None
        self._connected_once = False
        self._stopping = False    # set by stop() (main thread, Ctrl-C)

    # --- port access --------------------------------------------------------
    def _write(self, data):
        """Write to the C3 if a live fd exists; never raises.

        Every 'g'/'x'/'m' write goes through here. With the old direct
        os.write() calls a single fd-is-None or OSError between reconnects
        raised out of run() and killed the reader thread - the stream then
        stayed dead until the whole script was restarted.
        """
        fd = self._fd
        if fd is None:
            return False
        try:
            os.write(fd, data)
            termios.tcdrain(fd)
            return True
        except (OSError, TypeError, ValueError):
            return False

    def send_command(self, chars):
        """Write control chars ('g','x','0'..'2', ...) to the C3. Called from
        the HTTP handler thread."""
        if isinstance(chars, str):
            chars = chars.encode()
        return self._write(chars)

    def _try_open(self):
        """One attempt at getting a live fd on the C3 (or None).

        Tries the known port first; when that device node is GONE (USB
        re-enumeration gives the C3 a new /dev/ttyACM* number) - or when no
        port is known yet - it rescans and adopts the first port that answers
        the C3 probe.
        """
        if self.port:
            try:
                return open_serial(self.port, self.baud)
            except FileNotFoundError:
                pass                      # node vanished - re-detect below
            except Exception as exc:
                self._log_once("open", "[serial] %s: %s" % (self.port, exc))
                return None
        for cand in _candidate_ports(None):
            if cand == self.port:
                continue
            try:
                fd = open_serial(cand, self.baud)
            except Exception:
                continue
            if _probe_is_c3(fd):
                print("[serial] C3 re-detected on %s (was %s)"
                      % (cand, self.port), flush=True)
                self.port = cand
                return fd
            try:
                os.close(fd)
            except OSError:
                pass
        self._log_once(
            "none",
            "[serial] no C3 found on any candidate port (is it plugged in?)"
            if not self.port else
            "[serial] %s is gone and no other port answers like a C3"
            % self.port)
        return None

    def _log_once(self, key, msg):
        """Rate-limit a repeated connect-loop message (~every 15 s)."""
        last = getattr(self, "_log_key", None)
        if last != key or time.time() - getattr(self, "_log_ts", 0) > 15.0:
            self._log_key = key
            self._log_ts = time.time()
            print(msg, flush=True)

    def stop(self):
        """Ctrl-C path: hand the port back to the C3's console ('x') and let
        the loop end. Without this the 'x' documented above never actually
        reached the C3 - the process just exited and the fd was closed."""
        self._stopping = True
        self._write(b"x")
        fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def run(self):
        parser = FrameParser()
        fd = None
        last_stall_warn = 0.0
        stream_started = False
        open_ts = 0.0
        got = 0                 # frames decoded in THIS loop pass
        # Progress marker for the "open but silent" port cycle.
        prog = (-1, -1)
        prog_ts = 0.0
        try:
            while True:
                if self._stopping:
                    break
                # ---- connect / reconnect (never gives up) -------------------
                if fd is None:
                    fd = self._try_open()
                    if fd is None:
                        self.state.reader_open = False
                        time.sleep(USB_RECONNECT_DELAY)
                        continue
                    self._fd = fd
                    # A fresh connection must not resync against the dead
                    # link's leftover buffer.
                    parser = FrameParser()
                    self._a_seq = None
                    stream_started = False
                    open_ts = time.time()
                    prog = (-1, -1)
                    prog_ts = time.time()
                    self.state.port = self.port
                    self.state.reader_open = True
                    if self._connected_once:
                        self.state.reconnects += 1
                        print(f"[serial] reopened {self.port}", flush=True)
                    else:
                        self._connected_once = True
                        print(f"[serial] opened {self.port} @ {self.baud} baud "
                              f"(raw 8N1)", flush=True)

                try:
                    chunk = os.read(fd, 65536)
                except OSError as exc:
                    if self._stopping:
                        break          # stop() closed the fd under us
                    # USB re-enumerated (C3 reboot / cable glitch): the fd is
                    # dead. Close it and go back to the connect loop, which
                    # retries forever - this is what makes the stream
                    # self-recover after a drop instead of needing a restart.
                    self._log_once(
                        "read:%s" % exc,
                        f"[serial] read error: {exc} - reopening")
                    self.state.errors += 1
                    self.state.reader_open = False
                    self._fd = None
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    fd = None
                    # Short backoff: a device node that is present but already
                    # dead (the C3 re-enumerating) would otherwise be opened
                    # and EIO'd in a tight loop.
                    time.sleep(0.5)
                    continue
                got = 0                      # frames decoded in THIS pass
                if not chunk:
                    time.sleep(0.002)
                else:
                    try:
                        for _fid, payload in parser.feed(chunk):
                            self.state.set_frame(payload, _fid)
                            got += 1
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
                            # "[rx] REC ..." lines carry the SENDER's recording
                            # state -> feed the dashboard's recording card.
                            self.state.set_rec(line)
                            # "[rx] link rssi=... " lines carry the C3's live
                            # link level -> the dashboard's SIGNAL card.
                            self.state.set_link(line)
                    except Exception as exc:   # never let a bad chunk kill us
                        print(f"[serial] parse error: {exc}", flush=True)
                        self.state.errors += 1

                # Stay in console mode for a moment first: the C3 greets a new
                # USB host with its banner + MAC, which we relay - on-page
                # proof of which board/port we are attached to.
                if not stream_started and time.time() - open_ts >= 1.5:
                    stream_started = True
                    self._write(b"g")
                    self._last_g = time.time()

                # Keep-alive: the C3 falls back to CONSOLE mode whenever its
                # USB host re-enumerates (C3 reboot, cable glitch, S3 link
                # loss causing a board reset...), and any stray 'x' (another
                # host instance, a monitor exit handler) puts it there too. It
                # only re-enters stream mode when it sees 'g' again, so re-send
                # 'g' every ~2 s while NO frame is being decoded.
                # NOTE `got` (frames in THIS pass), not `parser.frames`: that
                # counter is cumulative and non-zero forever after the first
                # frame, which used to refresh _last_g on every pass and made
                # this keep-alive dead once the stream had ever worked - a
                # single stray 'x' then froze the picture until a manual
                # restart. Verified live: with `parser.frames` the poke left
                # frames frozen at 96 for 25+ s; with `got` it recovers in ~2 s.
                if got:
                    self._last_frame_ok = time.time()
                    self._last_g = time.time()
                elif time.time() - self._last_frame_ok > 3.0 and \
                        time.time() - self._last_g > 2.0 and \
                        not self.state.paused:
                    self._last_g = time.time()
                    self._write(b"g")

                # Open but COMPLETELY silent (not one byte, not even console
                # text): the host-side CDC endpoint or the C3's USB stack is
                # wedged. resending 'g' cannot clear that - only closing and
                # reopening the device does, which is exactly the manual
                # restart this loop now does for you. Skipped while paused:
                # a deliberate PAUSE is meant to be silent.
                if self.state.paused:
                    prog_ts = time.time()
                elif (parser.frames, parser.bytes) != prog:
                    prog = (parser.frames, parser.bytes)
                    prog_ts = time.time()
                elif time.time() - prog_ts > USB_SILENCE_CYCLE_S:
                    print(f"[serial] no data for {USB_SILENCE_CYCLE_S:.0f}s on "
                          f"{self.port} - cycling the USB port", flush=True)
                    self.state.reader_open = False
                    self._fd = None
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                    fd = None
                    continue

                # Distinguish "no sender at all" from "bytes flowing but no
                # complete frame" - the latter usually means the S3 stopped
                # mid-stream (e.g. browned out) or the link is dropping. A C3
                # that cannot see its S3's AP only prints the '.' WiFi-join
                # dots from setup() and answers no serial command at all, so
                # name that case too - it looks exactly like a dead host link.
                now = time.time()
                age = now - (self.state.last_frame_ts or now)
                if parser.bytes > 0 and parser.frames == 0 and age > 10 \
                        and now - last_stall_warn > 10:
                    last_stall_warn = now
                    print(f"[serial] {parser.bytes} bytes seen but 0 valid "
                          f"frames in {age:.0f}s - the C3 is only printing "
                          f"console text (WiFi-join '.' dots mean it is still "
                          f"waiting for the S3's AP; check the S3's power), or "
                          f"the sender stopped mid-stream", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            # Politely hand the USB port back to console mode (m/s in the
            # pio monitor work again) before we close it. stop() usually did
            # this already; this covers the KeyboardInterrupt-in-this-thread
            # case. fd can be None (between reconnects) - os.close(None) raises
            # TypeError, which used to escape this finally.
            self._stopping = True
            try:
                if fd is not None:
                    self._write(b"x")
                self._fd = None
                if fd is not None:
                    os.close(fd)
            except (OSError, TypeError, ValueError):
                pass
            self.state.reader_open = False


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
  <div class='card'><div class='v' id='c_sig'>–</div><div class='l'>LINK SIGNAL (S3)</div>
    <div class='s' id='c_chan'>RSSI of the C3↔S3 link (WiFi.RSSI on the C3)</div></div>
</div>
<h3>Controls</h3>
<div class='grid'>
  <div class='card' style='text-align:center'><div class='l'>QUALITY (S3)</div>
    <div style='display:flex;gap:6px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('0')">LOW</button>
      <button onclick="ctl('1')">MED</button>
      <button onclick="ctl('2')">HIGH*</button>
      <button onclick="ctl('A')">AUTO</button>
    </div><div class='s'>*capped at MEDIUM on ESP-NOW. LOW/MED/HIGH pin the
    choice (the adaptive up-stepper stops undoing it); AUTO hands quality back
    to the adaptive controller.</div></div>
  <div class='card' style='text-align:center'><div class='l'>USB STREAM</div>
    <div style='display:flex;gap:8px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('g')">▶ START</button>
      <button onclick="ctl('x')">⏸ PAUSE</button>
      <button onclick="ctl('M')">🎤 MIC</button>
    </div><div class='s' id='ctlmsg'></div></div>
  <div class='card' style='text-align:center'><div class='l'>FPS PRIORITY</div>
    <div style='display:flex;gap:8px;justify-content:center;margin-top:6px'>
      <button onclick="fpsToggle()" id='fpsbtn'>⚡ FPS</button>
    </div>
    <div style='display:flex;gap:6px;justify-content:center;margin-top:6px'>
      <input id='fpsval' type='number' min='1' max='60' value='15'
             style='width:70px;background:var(--bg);color:var(--tx);
                    border:1px solid var(--line);border-radius:6px;padding:5px 6px'>
      <button onclick='setFps()'>SET FPS</button>
    </div><div class='s'>fps mode: mic OFF + quality pinned LOW (until you pick a
    quality yourself) + the up-stepper blocked, so the freed airtime goes to
    frame rate instead of a VGA up-step. SET FPS is an absolute cap, 1-60;
    above ~9 fps airtime is the wall, so 60 = "as fast as the link allows".
    Needs the F0/F1 + V build on BOTH boards.</div></div>
</div>
<h3>SD recording on the sender (ESP-NOW build only)</h3>
<p class='foot' style='margin:0 0 6px'>Records the SAME frames the dish receives onto
the S3's microSD card as a real .avi clip. The buttons travel
dashboard → C3 → ESP-NOW → S3; the state below is reported back by the sender.</p>
<div class='grid'>
  <div class='card' style='text-align:center'><div class='l'>RECORDING</div>
    <div style='display:flex;gap:8px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('r')">⏺ START REC</button>
      <button onclick="ctl('e')">⏹ END REC</button>
    </div><div class='s' id='recstat'>no report yet</div></div>
  <div class='card' style='text-align:center'><div class='l'>END AFTER (SECONDS)</div>
    <div style='display:flex;gap:6px;justify-content:center;margin-top:6px'>
      <input id='recsecs' type='number' min='0' max='3600' value='30'
             style='width:80px;background:var(--bg);color:var(--tx);
                    border:1px solid var(--line);border-radius:6px;padding:5px 6px'>
      <button onclick='setDur()'>SET</button>
    </div><div class='s' id='durset'>the clip ends by itself after this many seconds
    (0 = record until ⏹ END REC)</div></div>
  <div class='card' style='text-align:center'><div class='l'>PRIORITY SD ↔ STREAM</div>
    <div style='display:flex;gap:6px;justify-content:center;margin-top:6px'>
      <button onclick="ctl('p0')">SD</button>
      <button onclick="ctl('p1')">BALANCED</button>
      <button onclick="ctl('p2')">STREAM</button>
    </div><div class='s' id='prioset'>whose timing wins when one frame can't afford
    both: SD keeps the clip's rate, STREAM keeps the stream's</div></div>
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
// Watchdog: a <img> MJPEG stream whose connection died (server restart, C3
// reset, Wi-Fi blip) never re-establishes itself - the page would sit on the
// last frame forever even after frames are flowing again. When /status says
// nothing has arrived for a few seconds, reopen /stream until it does.
var lastSeq=-1, staleTicks=0;
function streamWatchdog(seq){
  if(seq!==lastSeq){ lastSeq=seq; staleTicks=0; return; }
  if(++staleTicks>=4){          // 4 s with no new frame -> reopen the stream
    staleTicks=0;
    var v=document.getElementById('v');
    v.src='/stream?t='+Date.now();
  }
}
setInterval(function(){
  fetch('/status',{cache:'no-store'}).then(function(r){return r.json()}).then(function(s){
    streamWatchdog(s.seq);
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
    // --- live link signal (RSSI reported by the C3) -------------------------
    var sg=document.getElementById('c_sig'), chan=document.getElementById('c_chan');
    if(s.rssi==null || s.rssi_age>8){
      sg.textContent='–'; sg.style.color='var(--dim)';
      chan.textContent = s.rssi==null
        ? 'no report yet (older C3 firmware?)'
        : 'stale (no report for '+Math.round(s.rssi_age)+' s)';
    } else {
      sg.textContent = s.rssi+' dBm';
      sg.style.color = !s.rssi_sta ? 'var(--dim)'
        : (s.rssi>-60?'var(--ok)':(s.rssi>-75?'var(--warn)':'var(--bad)'));
      chan.textContent = (s.rssi_sta?'C3 ↔ S3 link':'C3 not associated')+
        ' · ch '+s.rssi_ch;
    }
    document.getElementById('port').textContent = '@ '+s.port+
        (s.reader==='open'?'':'  ⟳ reconnecting…'+
         (s.reconnects?(' ('+s.reconnects+')'):''))+
        (paused?'  ⏸ paused (button)':'');
    // --- SD recording card (state REPORTED BY THE SENDER via the C3) --------
    var rs=document.getElementById('recstat');
    var pn={0:'SD first',1:'balanced',2:'stream first'};
    if(s.rec_age<0 || s.rec_age>8){
      rs.textContent='no report yet - is the S3 running the ESP-NOW build?';
      rs.style.color='var(--dim)';
    } else if(s.rec_on){
      rs.textContent='⏺ REC clip #'+s.rec_idx+' · '+s.rec_frames+' frames · left '+
        (s.rec_left==='unlimited'?'∞':s.rec_left)+' · '+(pn[s.rec_prio]||'?');
      rs.style.color='var(--ok)';
    } else {
      rs.textContent='idle'+(s.rec_sd?'':' · no SD card detected on the S3');
      rs.style.color=s.rec_sd?'var(--dim)':'var(--warn)';
    }
    hist.push(live?s.fps:0); if(hist.length>120) hist.shift();
    draw('g_fps',hist);
  }).catch(function(){});
  fetch('/console',{cache:'no-store'})
    .then(function(r){return r.text()})
    .then(function(t){ if(t) document.getElementById('lg').textContent = t; })
    .catch(function(){});
},1000);
var fpsOn=false;
function setFps(){
  var el=document.getElementById('fpsval');
  var v=parseInt(el.value,10);
  if(!(v>=1&&v<=60)){ alert('fps must be 1-60'); return; }
  ctl('V'+v);
}
function fpsToggle(){
  fpsOn=!fpsOn;
  ctl(fpsOn?'F1':'F0');
  var b=document.getElementById('fpsbtn');
  if(b) b.textContent = fpsOn ? '⚡ FPS ON' : '⚡ FPS';
}
function ctl(c){
  var names={g:'START stream',x:'PAUSE stream',M:'toggle MIC',
             r:'START recording',e:'END recording',
             'F0':'fps mode OFF','F1':'fps mode ON (mic off, LOW pinned)',
             'A':'quality AUTO (adaptive)',
             '0':'quality LOW','1':'quality MED','2':'quality HIGH'};
  if(c.charAt(0)==='t') names[c]='recording ends after '+c.slice(1)+' s';
  if(c.charAt(0)==='p') names[c]='priority '+({0:'SD',1:'balanced',2:'stream'}[c.slice(1)]||'?');
  if(c.charAt(0)==='V') names[c]='fps cap '+c.slice(1);
  if(c.charAt(0)==='F') names[c]=(c==='F1')?'fps mode ON':'fps mode OFF';
  fetch('/ctl?cmd='+c,{cache:'no-store'}).then(function(r){return r.json()})
    .then(function(j){
      var m=document.getElementById('ctlmsg');
      m.textContent = j.ok ? (names[c]||('sent "'+c+'"'))+' → sender' : 'port not open';
      setTimeout(function(){m.textContent=''},2500);
    }).catch(function(){});
}
// "end after N seconds": sends t<N> to the C3, which relays it to the sender.
function setDur(){
  var v=document.getElementById('recsecs').value.replace(/[^0-9]/g,'');
  if(v==='') v='0';
  document.getElementById('durset').textContent =
    (v==='0' ? 'the clip runs until ⏹ END REC is pressed'
             : 'the clip ends by itself after '+v+' s');
  ctl('t'+v);
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
            # SD recording on the sender (from the C3's "[rx] REC ..." lines)
            # plus the window of the last /ctl request, so the page can say
            # whether it is talking to a build that even HAS the recorder.
            rec, rec_age = self.state.rec_snapshot()
            rec_on = "true" if (rec and rec["on"]) else "false"
            rec_sd = "true" if (rec and rec["sd"]) else "false"
            # Live link level reported by the C3 (WiFi.RSSI() of the S3's AP).
            link, link_age = self.state.link_snapshot()
            body = ('{"seq":%d,"frames":%d,"skips":%d,"errors":%d,'
                    '"bytes":%d,"text":%d,"last_age":%.1f,"port":"%s",'
                    '"reader":"%s","reconnects":%d,'
                    '"fps":%.1f,"kbps":%.0f,"avg_frame":%d,'
                    '"est_air_ms":%.0f,"gap_med_ms":%.0f,"gap_p95_ms":%.0f,'
                    '"paused":%s,"drop_pct":%s,'
                    '"rssi":%s,"rssi_ch":%d,"rssi_sta":%s,"rssi_age":%.1f,'
                    '"rec_on":%s,"rec_idx":%d,"rec_frames":%d,"rec_left":"%s",'
                    '"rec_prio":%d,"rec_sd":%s,"rec_age":%.1f,"fps_mode":%s,'
                    '"fps_target":%s,"quality_manual":%s}'
                    % (st[1], st[2], st[3], st[4], st[5],
                       getattr(self.state, "text_bytes", 0), age,
                       self.state.port,
                       "open" if self.state.reader_open else "waiting",
                       self.state.reconnects,
                       fps, kbps, avg_b, est_lat,
                       med, p95,
                       "true" if self.state.paused else "false",
                       "null" if drop_pct is None else "%.1f" % drop_pct,
                       "null" if link is None else "%d" % link["rssi"],
                       (link["ch"] if link else 0),
                       "true" if (link and link["sta"]) else "false",
                       link_age,
                       rec_on, (rec["idx"] if rec else 0),
                       (rec["frames"] if rec else 0),
                       (rec["left"] if rec else "-"),
                       (rec["prio"] if rec else -1), rec_sd, rec_age,
                       "null" if self.state.fps_set is None
                       else "%d" % self.state.fps_set,
                       "null" if self.state.fps_target is None
                       else "%d" % self.state.fps_target,
                       "true" if self.state.quality_manual else "false")).encode()
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
            # Dashboard controls. Single chars (g/x/0/1/2/M/r/e) plus the SD
            # recording commands the C3 relays to the sender:
            #   r / e   start / stop the microSD recording on the S3
            #   t<secs> max clip length (t0 = record until stopped)
            #   p0..p2  recording/streaming priority
            # The command is validated STRICTLY rather than filtered: a stray
            # 'r' inside junk must not start a recording, and the old
            # character-filter let "drop me!!r" through as "rper".
            from urllib.parse import urlparse, parse_qs, unquote
            qs = parse_qs(urlparse(self.path).query)
            raw = unquote(qs.get("cmd", [""])[0]).strip()
            clean = ""
            if re.fullmatch(r"[gx012MreA]", raw):       # single-char commands
                clean = raw
            elif re.fullmatch(r"F[01]", raw):           # fps-priority mode
                clean = raw
            elif re.fullmatch(r"[Vv]\d{1,2}", raw):      # fps cap, validated
                _n = int(raw[1:])
                clean = ("V%d" % _n) if 1 <= _n <= 60 else ""
            elif re.fullmatch(r"t\d{1,5}", raw):        # max clip length
                clean = raw
            elif re.fullmatch(r"p[0-2]", raw):          # SD/stream priority
                clean = raw
            ok = bool(clean) and self.state.reader and \
                 self.state.reader.send_command(clean)
            # Track pause state so /status can tell the dashboard "stream
            # paused on purpose" apart from "link dead" (issue #4).
            if ok and clean == "g":
                self.state.paused = False     # START resumes the stream
            elif ok and clean == "x":
                self.state.paused = True      # PAUSE is deliberate
                self.state.pause_ts = time.time()
            elif ok and clean.startswith("t"):
                # Remember what we asked for - the S3 does not echo settings
                # back separately, and the page shows the requested value.
                self.state.rec_set["seconds"] = clean[1:] or "0"
            elif ok and clean.startswith("p"):
                self.state.rec_set["prio"] = clean[1:] or "?"
            elif ok and clean in ("F0", "F1"):
                # fps-priority mode as last requested (not echoed by the S3).
                self.state.fps_set = 1 if clean == "F1" else 0
                if clean == "F1":
                    self.state.quality_manual = False   # F1 re-pins LOW itself
            elif ok and clean.startswith("V"):
                self.state.fps_target = int(clean[1:])
                self.state.quality_manual = True        # explicit rate pins it
            elif ok and clean == "A":
                self.state.fps_target = None
                self.state.quality_manual = False       # adaptive is back in charge
            elif ok and clean in ("0", "1", "2"):
                self.state.quality_manual = True        # explicit quality pins it
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
        print(f"[serial] {port}: no answer to the C3 probe (a C3 that is still "
              f"joining the S3's AP prints only '.' dots and answers nothing)",
              flush=True)
        os.close(fd)

    if reader is None:
        if fallback_port is None:
            # Nothing on USB right now. Keep running and keep looking: the
            # reader retries forever, so the viewer works as soon as the C3 is
            # plugged in (no script restart needed).
            print("[serial] no serial port found yet - waiting for the C3 to "
                  "appear (pass --port to pin it)", flush=True)
            reader = SerialReader(args.port, args.baud, state,
                                  noise_gate=args.noise_gate)
        else:
            print(f"[serial] no port answered like a C3; falling back to "
                  f"{fallback_port} (pass --port to override)", flush=True)
            reader = SerialReader(fallback_port, args.baud, state,
                                  noise_gate=args.noise_gate)

    # Serve HTTP first so the status page is reachable even before frames flow.
    state.reader = reader            # enables the /ctl dashboard controls
    try:
        httpd = ThreadingHTTPServer((args.host, args.http_port), ViewerHandler)
    except OSError as exc:
        # A socket already bound to this port is almost always a leftover
        # instance from a previous run - the classic "it only works after I
        # restart everything". Say so plainly instead of printing a traceback.
        print(f"[http] cannot bind port {args.http_port}: {exc}\n"
              f"[http] another c3_usb_mjpeg.py is probably still running "
              f"(find it with: ss -ltnp | grep {args.http_port}) - "
              f"or pass --http-port N", file=sys.stderr, flush=True)
        return 1
    print(f"[http] MJPEG server at http://{_lan_ip()}:{args.http_port}/ "
          f"(stream: /stream)", flush=True)
    try:
        reader.start()
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[stop] shutting down", flush=True)
    finally:
        reader.stop()          # sends 'x' so the C3 returns to console mode
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