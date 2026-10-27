# HANDOFF - ESP32-C3 host viewer + link work (2026-10-16 session)

Written at ~300k context. Everything below is verified unless marked otherwise.
Working tree is UNCOMMITTED; 7 files (6 modified + 1 new, see "Files" at the end).

---

## 1. What this session fixed (all verified, live where possible)

### A. Host reader thread died permanently -> "I have to restart it sometimes"
`host/c3_usb_mjpeg.py`: on a read error the fd was closed, and a failed reopen
left `fd = None`; the next pass called `os.read(None, ...)` -> TypeError that
escaped `run()` (only OSError was caught) and raised again inside
`finally: os.close(None)`. Thread dead, HTTP page frozen, restart required.
Reproduced against the fake-pty harness on the old file, exact traceback:
`c3_old.py:453 in run -> chunk = os.read(fd, 65536)` /
`TypeError: 'NoneType' object cannot be interpreted as an integer`.
Fix: the reader never gives up - retry opens every 1 s, reconnect on read error,
re-detect a renamed node (ttyACM0 -> ttyACM1), fd-cycle an open-but-byte-silent
port after `USB_SILENCE_CYCLE_S` (20 s), all fd writes via `_write()` (fd-None /
OSError swallowed), `SerialReader.stop()` sends the documented `x` on Ctrl-C.
Details: TMP_ISSUES.md #3.

### B. The `g` keep-alive was DEAD after the first frame -> "dropped out and didn't recover"
`run()`'s re-arm was gated on `if parser.frames:` - the parser's CUMULATIVE
counter (non-zero forever after frame 1) - so `_last_g`/`_last_frame_ok` were
refreshed every pass and no `g` was ever re-sent. The C3 drops to CONSOLE mode on
any stray `x` (second host instance, `pio device monitor` exit, its own USB
glitch) and then needs a fresh `g`; it never came, so the picture stayed frozen
until the script was restarted.
Live evidence from the C3 while broken (sole-reader probe):
`latest frame #186 len 3034  espnow rx 2446  usb written 953  mode console` with
`[rx] frame N ok (3024 B, 13 seg)` climbing -> air link/reassembly perfect, only
USB forwarding stopped. Sending `g` every second streamed 70 s straight.
Fix: gate on frames decoded in THIS pass (`got`), re-arm every ~2 s when nothing
is decoded (still suppressed while PAUSEd).
A/B (`ka_check.py`, 2 frames then 8 s of console-only traffic):
OLD = 0 `g` bytes re-sent; FIXED = 3. Live: one stray `x` froze frames at 96 for
25+ s before; after, recovered in 2-4 s, and again on a second poke.
Details: TMP_ISSUES.md #10.

### C. Page could not re-open a dead MJPEG stream
`<img src='/stream'>` never reconnects. Added a watchdog: if `/status` `seq` does
not advance for 4 s, the page re-opens `/stream` until it moves.

### D. A held port reported "[Errno 9] Bad file descriptor"
`open_serial()` closed the fd twice; the second close raised EBADF from the outer
handler and replaced the useful RuntimeError. Fixed; a held port now says
"port is locked (held by pid N: ...)" (PID from a /proc scan, no lsof).

### E. Live link RSSI on the dashboard (new feature)
Firmware `receiver/src/receiver.cpp` -> `reportLink()` prints
`[rx] link rssi=<dBm> ch=<n> sta=<0|1>` (WiFi.RSSI() of the S3's AP - same S3,
same channel the ESP-NOW video crosses), ~1 Hz, sooner on >=2 dBm moves.
The old single-slot REC line buffer became a 4-slot queue (`queueUsbLine()` /
`flushUsbLine()`) flushed ONLY between frames (`outState == 0`), so console text
can never land inside a JPEG. Host: `LINK_LINE_RE` -> `/status` (`rssi`,
`rssi_ch`, `rssi_sta`, `rssi_age`) -> SIGNAL card.
Availability, CHECKED not assumed (installed core framework-arduinoespressif32
4.20017.260907): `esp_now_recv_cb_t` is
`(const uint8_t *mac_addr, const uint8_t *data, int data_len)` - no `rx_ctrl`,
so per-datagram RSSI is NOT reachable from the callback (needs promiscuous mode +
MAC filter). Receiver AGC/LNA gain is not software-readable at all; TX power is
(`WiFi.getTxPower()`/`setTxPower()`); the S3's mic gain is compile-time x8
(`src/stream_espnow.cpp`).
Receiver build passes: RAM 34.5 % (113 116 B), Flash 55.8 % (730 778 B).
**NOT FLASHED**: the card shows "-" until
`cd receiver && pio run -t upload -e esp32c3_receiver` (user has not given the
go-ahead; the S3 was streaming at the time).

### Verification commands
    cd "/home/willi/Documents/GitHub/ESP32 Example"
    python3 host/test_reader_recovery.py      # 58 checks, ALL PASS, no hardware
    python3 -m py_compile host/c3_usb_mjpeg.py host/test_reader_recovery.py
    cd receiver && pio run -e esp32c3_receiver
Harness vs `git show HEAD:host/c3_usb_mjpeg.py` (save as c3_old.py): 18 failures
incl. "reader thread survived the outage" and the keep-alive [9] check.

---

## 2. OPEN: the ~2.8-3.3 fps question ("is there any way it can be faster?")

Question asked, investigation only PARTIALLY done (this is where the session
stopped).

### Measurements (live, real board + real link)
- ~3.0-3.5 KB per frame (observed 2577-5164 B), 12-22 segments of 240 B.
- Host `/status`: fps ~2.5-3.3, gap_med ~334 ms, gap_p95 ~674 ms, drop_pct ~16.7 %.
- The C3's own air counter agrees: `espnow rx` 2423 -> 2446 over ~7 s = ~3.3 fps
  (from `link_timeline.py`). So the sender emits ~3 fps; the C3/host are NOT the
  bottleneck.
- Board is the C3 (`/dev/ttyACM0`, Espressif USB JTAG, MAC E8:F6:0A:17:55:BC);
  the S3 sender is airborne and NOT connected to the PC (no S3 serial log).

### Facts gathered from the firmware source
- Presets, `src/camera_hal.cpp` -> `qualityProfileFor()`:
  LOW QVGA 320x240 jpegq35 max_fps **15.0**; MEDIUM VGA 640x480 q20 max_fps 10.0;
  HIGH UXGA 1600x1200 q12 max_fps 5.0.
- `src/config.h:33`: `QUALITY_DEFAULT QUALITY_LOW` -> the S3 boots at LOW.
  `src/config.h:124`: `ESPNOW_MAX_QUALITY QUALITY_MEDIUM` caps adaptive up-stepping.
- `src/config.h:101`: `ESPNOW_SEND_RETRIES 2`, `ESPNOW_RETRY_DELAY_MS 1`;
  per-segment send path is `espNowSendReliable()` (`src/stream_espnow.cpp:227`)
  which waits for the send callback + retries. `sendFrame()` at line ~477 loops
  segments calling it; `espnowPoll()` (~line 500+) has `fpsGate(p.max_fps)`.
- **STRONG HYPOTHESIS (NOT YET CONFIRMED):** `src/stream_espnow.cpp:518`:
  `if (micEnabled && p.max_fps > 3.0f) p.max_fps = 3.0f;`
  The observed ~3.0-3.3 fps matches that cap almost exactly, i.e. the S3 may have
  the PDM microphone ENABLED (the mic toggle is a TOGGLE - an even number of 'M'
  deliveries cancels out, and `x`/`g`/quality commands were flying around during
  this session). If so, 3 fps is BY DESIGN and the fix is to turn the mic OFF
  (dashboard 🎤 MIC button, or 'M' on the S3's serial), not to raise any constant.

### Next steps (in order)
1. Confirm/deny the mic hypothesis:
   * Watch the C3's console pane for `[mic] ...` lines and check `micEnabled`'s
     boot default in `src/config.h` (MIC_* block, ~line 143-155) and
     `src/stream_espnow.cpp` `micTask`/toggle code (~line 50-160).
   * Toggle the mic ONCE from the dashboard (odd number of clicks!) and watch
     whether fps drops below ~2 (mic on => 3 fps cap) or rises (mic off).
   * `host/c3_usb_mjpeg.py`'s `/status` has no mic field; adding
     `"mic":true|false` (relayed via a new status tag, or inferred from the S3's
     `[mic]` console line) would settle it permanently.
2. If the mic is OFF and it is still ~3 fps, the limiter is elsewhere:
   * `fpsGate()` - read it: does it space captures by `1000/max_fps` ms, and does
     the capture itself take longer than the slot?
   * `sendFrame()` + `espNowSendReliable()`: 13 segments x (send + ACK wait +
     2 retries x 1 ms) - measure the real per-frame air time; a serialized
     ACK-waiting segment loop plus a 16.7 % segment loss can easily cost 300+ ms
     per frame. Consider: lossy multi-segment transmit (no per-segment ACK wait,
     like the audio path already does), or a bigger ESP-NOW payload, or fewer
     segments per frame (lower resolution).
   * The 16.7 % frame-id drop rate is worth chasing on its own - it inflates
     perceived latency and wasted airtime (`drop_pct` is computed from frame-id
     gaps over a 10 s window, `/status`).
   * SD recording is inert right now (`sd=0`, no card) so `RECORD_PRIORITY_*` is
     not a factor; if a card is inserted, `RECORD_AVI_FPS 15` and the SD write
     time enter the frame budget (`src/config.h:189-205`, `src/recorder.cpp`).
3. Do NOT flash the S3 or change presets without the user's go-ahead: the S3 is
   airborne and its serial log is not reachable from this PC.

---

## 3. Diagnostics used this session (in /home/willi/.hermes/cache/scratch -
   PRUNED after 24 h idle; ask before relying on them tomorrow)
- `link_timeline.py` - sole-reader timeline: frames/pass, console bytes, and the
  C3's own `latest frame ... espnow rx ... usb written ... mode ...` reply.
- `ka_check.py <module.py>` - keep-alive A/B (2 frames, then 8 s console-only).
- `poke_port.py 'x'|'g'` - writes ONE byte without reading/flocking (simulates a
  stray monitor/second instance). **Use with care: it can disturb a live host.**
- `poll_status.py [port] [n]` - /status timeline.
- `probe_board.py` - port-free check + `x`/`s`/`m` probe. It does NOT flock, so it
  splits bytes with a running host - this is a plausible trigger for the freeze
  the user hit mid-session (disclosed to them).
- `g_probe.py` - sends `g` every second (proved the C3 is not at fault).
- `fw_line_format.c` - compiles the firmware's own snprintf formats so the host
  regexes can be checked against real output (all lines parse, incl. rssi=-128).

## 4. State left behind
- Port `/dev/ttyACM0` FREE, no host instances running (verified via flock +
  pgrep). The C3 was left in console mode by the last instance's `x` (normal).
- `receiver/` firmware: modified but NOT flashed. `host/` script: uncommitted.
- Nothing pushed; no commits made this session.

## 5. Files (git status)
    M AI_README.md                  (host viewer: reader recovery, SIGNAL card)
    M README.md                      (troubleshooting rows, SIGNAL card, sync table)
    M TMP_ISSUES.md                  (#3 rewrite, #9 RSSI, #10 keep-alive freeze)
    M host/c3_usb_mjpeg.py           (A-E above)
    M receiver/src/receiver.cpp      (4-slot USB line queue + reportLink() RSSI)
    M receiver/src/receiver_config.h (LINK_REPORT_* knobs)
    ?? host/test_reader_recovery.py  (NEW: 58 checks, pty fake C3, no hardware)
