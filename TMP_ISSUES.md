# OPEN PROBLEMS / TODO — ESP32-S3 → C3 dish streaming (working notes)

_Originally a scratch file of unresolved issues. Updated 2026-09-14 after a
fix pass — see the per-issue **Status** lines below._

---

## 1. Audio does not play in real time — only after the Python script ends

**Status: FIXED (2026-09-14) — needs on-hardware confirmation.**

**What was wrong:** the `/audio.wav` endpoint wrote a RIFF header with huge
sizes and streamed forever. Browsers refuse to start playback of a
unknown-length wav until the connection CLOSES — hence "plays only when the
script ends".

**Fix:** the `<audio>` WAV player is gone. The server now exposes
`/audio.pcm` (raw 8 kHz u8 PCM, same live backlog-flush pacing as before),
and the dashboard uses the **Web Audio API**: a fetch stream reads chunks,
decodes each into an `AudioBuffer` and schedules it sample-accurately on an
`AudioContext` (with backlog skipping if the tab falls behind). `/audio.wav`
remains as an alias serving the same raw stream for old bookmarks.

---

## 2. ~80% / ~95% video drop rate while the microphone stream is enabled

**Status: FIXED (2026-09-14, second pass) — needs on-hardware confirmation.**

**What was fixed (sender firmware, src/stream_espnow.cpp):**
1. Audio no longer uses `espNowSendReliable` (retries + 2×30 ms ACK waits).
   New `espNowSendLossy()` sends each audio chunk ONCE with no ACK wait and
   drops it on failure — audio tolerates loss; retries were pure airtime
   waste that starved video segments.
2. While the mic streams: video fps is hard-capped at **3** (5 was still too
   many datagrams/s), quality is force-stepped to **LOW** when the mic turns
   on, and the adaptive controller is NOT allowed to climb back up. Net air
   load ≈ 3 fps × ~25 seg + 33 audio dg/s — well under the ~250 dg/s ceiling.
3. `micTask` no longer WAITS when the audio ring is full (that built up to
   ~0.5 s of latency); it drops the OLDEST sample instead. Ring shrunk
   4096→1600 B so worst-case queue delay is ~200 ms.

**Host side (host/c3_usb_mjpeg.py):** lost audio datagrams are now concealed
— short sequence gaps are filled with 30 ms-per-packet mid-level silence, so
single losses no longer click/pop ("choppy" audio). Longer stalls still skip
forward (both sides drop backlog rather than fast-forwarding).

**Third pass — the 99% drop rate was mostly a METRIC bug.** The host parser
counted every non-printable byte during a resync as a "skip": one corrupt
header made it scan ~3.5 KB of JPEG payload ≈ ~1300 "skips", which read as
~99% while the C3's own log showed nearly every frame delivered. Now:
- `skips` counts resync EVENTS (runs), not bytes;
- the DROP RATE card is computed from **video frame-id gaps** (the sender
  assigns one id per captured frame, so a missing id = one frame genuinely
  lost on the air / never forwarded), over a rolling 10 s window.

**Noise suppression (host, `AudioDenoise`):** a ~90 Hz one-pole high-pass
(removes PDM DC/rumble from the x8-gain mic) plus a peak-envelope noise gate
(fast attack / ~250 ms release, ducks to silence below the gate floor).
Applied to every 30 ms chunk before `/audio.pcm` serves it. Tune with
`--noise-gate N` (default 10; 0 disables; raise if hiss persists).

---

## 3. C3 keeps falling back to console mode / mystery 'x' commands

**Status: ADDRESSED + VERIFIED (2026-10-16).** The REAL root cause of "sometimes
I have to restart the host script" was not the C3 at all: the host's reader
thread died on its own.

- **The bug (host/c3_usb_mjpeg.py):** any read error on the USB fd made the
  reader close the port and try to reopen it. When that reopen failed (a
  re-enumerating device is missing for a second or two), it `continue`d with
  `fd = None`, and the next loop pass called `os.read(None, ...)` -> `TypeError`,
  which escaped `run()` (only `OSError` was caught) and then raised again inside
  `finally: os.close(None)`. The thread died silently; the HTTP server kept
  serving a frozen page until the script was restarted. Reproduced against the
  fake-pty harness - old code, verbatim traceback:
  `c3_old.py:453 in run -> chunk = os.read(fd, 65536)` /
  `TypeError: 'NoneType' object cannot be interpreted as an integer`, then
  `c3_old.py:588 in run -> os.close(fd)`.
- **Fix:** the reader now never gives up - failed opens retry every 1 s; a read
  error closes the fd, backs off 0.5 s and reconnects; a gone/renamed node
  (`ttyACM0` -> `ttyACM1`) is re-detected with the `s`-probe and adopted; an open
  but byte-silent port is fd-cycled every `USB_SILENCE_CYCLE_S` (20 s), which is
  the only thing that clears a wedged host-side CDC endpoint; every fd write goes
  through `_write()` (fd-is-None and OSError are swallowed, so a control write
  can never kill the thread); Ctrl-C now actually sends the documented `x`
  (`SerialReader.stop()`); a locked port names the holding PID (`/proc` scan, no
  lsof).
- Two related host fixes fell out of the same pass: the `g` keep-alive used to
  undo the dashboard's PAUSE within 3 s (now suppressed while paused), and a
  bound HTTP port / a late-plugged board used to abort startup with a traceback
  (now a clear message + the reader waits for the C3).
- Also fixed while probing the live board: a HELD port reported
  `[Errno 9] Bad file descriptor` instead of "port is locked". `open_serial()`
  closed the fd twice - the second `os.close()` raised EBADF from the outer
  handler and replaced the good `RuntimeError`, so the one message that names the
  holding process was the one nobody ever saw.
- **Verified:** `python3 host/test_reader_recovery.py` (58 checks, fake C3 on a
  pty behind a symlink, no hardware) - old file: 18 failures including the dead
  reader thread; fixed file: all pass. Live, with the S3 streaming: the real C3
  on `/dev/ttyACM0` opens, `frames` climb, `/stream` serves real JPEG
  (`Content-Length: 3526`, JFIF), `/status` reports `"reader":"open"`, and a
  side-by-side run of the OLD script on the same live link gives the same
  numbers (39 frames / 43 skips / 16.7 % drop over ~14 s) - no metric regression.

The original flock + `g` keep-alive questions are unchanged; the remaining
hardware test (single clean host instance vs. a stray `pio device monitor`) is
still open. Separate finding from the same session: the C3 on the bench answers
`x`/`s`/`m` with nothing but `.` - it is stuck in `setup()`'s `while (WiFi.status()
!= WL_CONNECTED)` join loop because the S3 sender is off. Its USB console is
therefore dead until the S3's AP exists; only then can anything be verified on
the wire.

---

## 4. Dashboard minor issues

**Status: FIXED (2026-09-14).**
- Drop-rate card shows `–` (grey) when NOTHING arrived in the 10 s window
  instead of a misleading `0.0%` (`/status` returns `drop_pct: null`).
- `/status` now carries a `paused` flag (set by the dashboard ⏸/▶ buttons);
  the header shows "⏸ paused (button)" so deliberate pauses are distinguish-
  able from a dead link.
- `EST. LATENCY (air)` stays a formula estimate — noted, by design.

---

## 5. Microphone fidelity / open hardware questions

- PDM pin orientation (`MIC_CLK_GPIO 42` / `MIC_DATA_GPIO 41`) still
  unverified for this board revision — if silence persists, swap them.
- Gain is still a fixed ×8 with clipping; no AGC. 8-bit @ 8 kHz stays
  intentionally lo-fi.
- Audio now uses the lossy single-shot send path (see #2) — the retry-waste
  part of this item is resolved.

---

## 6. Serial quality/recording keys — dispatch is split between two readers

**Status: WORKED AROUND (2026-10-13) — underlying duplication still present.**

`main.cpp`'s loop drains `Serial` completely before `espnowPoll()` /
`browserPoll()` run, so `espnowHandleSerialQ()` and `browserHandleSerialQ()`
never see a character — i.e. the README's "serial `q0`/`q1`/`q2` works in both
targets" is not actually true today. The new microSD recorder keys (`r`, `e`,
`D<secs>`, `P<0..2>`) were therefore dispatched from `main.cpp` into
`espnowHandleSerialKey()`, which also finally makes `q0/q1/q2` reach the ESP-NOW
stream module. Left as-is because unifying the dispatch touches the browser
target too. If you want one reader, move each target's serial handling into a
single per-target hook called from `main.cpp` and delete both `*HandleSerialQ()`.

---

## 7. microSD recording (merged from camera.zip) — what is verified, what is not

**Status: IMPLEMENTED + UNIT/INTEGRATION VERIFIED (2026-10-13); needs hardware.**

- Merged the standalone AVI-recorder sketch into `src/recorder.cpp/.h`; the
  ESP-NOW loop captures once and feeds the card and the air from the same JPEG.
  `RECORD_PRIORITY_*` chooses whose timing is protected (SD / BALANCED / STREAM).
- Dashboard controls + live status (see README §4 / AI_README §11). `/ctl` now
  validates commands strictly instead of filtering characters — before this
  change, junk like `"drop me!!r"` was accepted as `"rper"` (it could have
  started a recording).
- Verified without hardware: both `STREAM_TARGET` builds and the receiver build
  clean; `recorder.cpp` compiled natively and executed (settings clamping, EOI
  trimming, clip numbering, duration auto-stop) with its `.avi` output checked by
  `ffprobe` and a direct RIFF/`idx1` parse; the host script driven over real HTTP
  (parser, `/status`, every `/ctl` command, malformed-command rejection,
  firmware↔host status-line format sync).
- **Not verified:** the S3↔C3 control/status datagrams over the air, the real SD
  card (speed / FAT32 / J3 pad), the dashboard buttons end-to-end, and the
  priority behaviour under a genuinely saturated link.

---

## 8. RESOLVED: "camera failed" after the recorder merge was a 16 MHz XCLK

**Status: FIXED (2026-10-13) — measured on the board, not guessed.**

Symptom: `esp_camera_init -> ESP_OK`, sensor detected (OV3660, SCCB addr 0x3C),
then every `esp_camera_fb_get()` returned nothing and the frame count never rose.

Evidence, in order:

1. `'t'` bus sweep on the live board: **20 MHz PSRAM/DRAM and 10 MHz PSRAM/DRAM
   all frame a real JPEG; 16 MHz frames NOTHING (both buffer locations).**
   So the module/ribbon data path is ALIVE.
2. The build with `ENABLE_SD_RECORDING 0` (SD mount never runs) still failed at
   16 MHz, boot included -> **the microSD recorder was not the cause.**
3. Git history: `CAMERA_XCLK_HZ 16000000` is absent from the initial commit and
   arrived with the pre-existing uncommitted work, i.e. it was never part of the
   recorder merge.

Fix: `CAMERA_XCLK_HZ 20000000` (the board default). Re-flashed and verified:
`capture failures = 0`, the ESP-NOW TX counter climbed (`frames=6→40`), and the
recorder reported a mounted card (`REC state=off ... sd=1`).

Lesson: 16 MHz on the ESP32-S3 switches on the experimental PSRAM-DMA path that
breaks JPEG capture. It looks exactly like a dead camera module (this is the
second time on this project that "dead camera" was not hardware). Always run `t`
before handling the ribbon.

Note from the same session: at the earlier (16 MHz) boot the sender reported
`sd=0` (card not mounted) and it reports `sd=1` now; the camera being in its
broken 16 MHz DMA state at boot is the likeliest reason the SD/SPI mount failed
then, but that is inferred, not measured - if it recurs, check the card is
seated/FAT32 and watch the `[rec] mounting microSD...` lines at boot.

---

## 9. Live link level (RSSI) on the dashboard — implemented; what the radio cannot give

**Status: IMPLEMENTED + HOST-VERIFIED (2026-10-16); the C3 firmware needs a flash
before the card shows anything.**

- `receiver/src/receiver.cpp` -> `reportLink()` prints
  `[rx] link rssi=<dBm> ch=<n> sta=<0|1>` (WiFi.RSSI()/channel of the S3's AP) at
  `LINK_REPORT_PERIOD_MS` (1 s), earlier when the level moves by
  `LINK_REPORT_MIN_DELTA` (2 dBm). Host: `LINK_LINE_RE` -> `/status` (`rssi`,
  `rssi_ch`, `rssi_sta`, `rssi_age`) -> dashboard SIGNAL card (green > -60 dBm,
  amber > -75, red below).
- Frame safety: the line goes through a 4-slot USB line queue (`queueUsbLine()` /
  `flushUsbLine()`) flushed only while the pump is BETWEEN frames
  (`outState == 0`); the old single-slot REC mechanism was generalised into it, so
  REC and link lines no longer fight over one buffer.
- **Availability, checked not assumed** (installed core
  `framework-arduinoespressif32` 4.20017.260907): `esp_now_recv_cb_t` in
  `tools/sdk/esp32c3/include/esp_wifi/include/esp_now.h` is
  `void (*)(const uint8_t *mac_addr, const uint8_t *data, int data_len)` - **no
  `rx_ctrl`, so per-datagram RSSI is not reachable from the receive callback**.
  Getting it would mean promiscuous mode + a MAC filter. Receiver **gain**
  (AGC/LNA) is not software-readable at all on this PHY; TX power is
  (`WiFi.getTxPower()`/`setTxPower()`, currently untouched); the S3's mic gain is
  compile-time ×8 in `src/stream_espnow.cpp`.
- Host verification without the flash: the harness feeds real console text
  between frames and checks the parse + `/status` fields, and the firmware's
  `snprintf` formats were compiled with gcc and piped through the host regexes
  (including `rssi=-128` and `sta=0`) - all lines parse.
- **Not verified:** the on-board RSSI value itself (needs
  `cd receiver && pio run -t upload -e esp32c3_receiver`). Receiver build passes:
  RAM 34.5 % (113 116 B), Flash 55.8 % (730 778 B).

---

## 10. "It dropped out and didn't recover" — the `g` keep-alive was dead, and the page could not re-open its stream

**Status: FIXED + REPRODUCED LIVE (2026-10-16).**

Two separate faults, both host-side, both invisible in the logs the user sees:

1. **The keep-alive never fired again once the stream had ever worked.**
   `run()`'s re-arm was gated on `if parser.frames:` — the parser's *cumulative*
   frame counter, which is non-zero forever after the first frame. So every loop
   pass refreshed `_last_g`/`_last_frame_ok`, the "no frames for 3 s" branch was
   never entered, and no `g` was ever re-sent. The C3 falls back to CONSOLE mode
   on any stray `x` (second host instance, `pio device monitor` exit handler, a
   C3 USB glitch) and then needs a new `g` to stream again - it never got one, so
   the picture stayed frozen until the script was restarted. `g_probe.py` sending
   `g` every second streamed 70 s straight with no dropout, which is what proved
   the C3 is not at fault.
   **Fix:** gate on frames decoded in *this* pass (`got`), so the re-arm fires
   every ~2 s whenever nothing is being decoded (still suppressed while PAUSEd).
   A/B with `ka_check.py` (2 frames, then 8 s of console-only traffic):
   old = 0 `g` bytes re-sent, fixed = 3. Live, real board: a single stray `x`
   byte froze `frames` at 96 for 25+ s before the fix; after it, the stream was
   moving again 2-4 s later with no intervention.
2. **The browser never re-opened a dead MJPEG stream.** `<img src='/stream'>`
   does not reconnect on its own, so even after frames resume the page can sit on
   the last frame forever (independent of #1 - server restart, Wi-Fi blip, or a
   closed connection).
   **Fix:** page watchdog - when `/status` shows the same `seq` for 4 consecutive
   seconds, the page re-opens `/stream` until frames move again.

Evidence from the C3 itself while broken (`link_timeline.py`, sole reader):
`latest frame #186 len 3034  espnow rx 2446  usb written 953  mode console` with
`[rx] frame N ok (3024 B, 13 seg)` climbing - the air link and reassembly were
perfect, only the USB forwarding was stopped (`usb written` frozen).

Regression tests: `host/test_reader_recovery.py` [9] covers the keep-alive
(68 checks total after issue #11 added 9).

---

## 11. "Is there any way it can be faster?" - the 3 fps cap is real, but turning the mic off makes it SLOWER

**Status: DIAGNOSED + MEASURED LIVE (2026-10-16); the new FPS-priority mode is
built and host-verified, but NOT flashed - it needs both boards, so it has never
executed.**

The question started as a HYPOTHESIS in HANDOFF.md: `src/stream_espnow.cpp:518`
caps `p.max_fps` at 3.0 while the mic is enabled, and the observed ~3 fps matched
that cap. The hypothesis was right about the cause and wrong about the fix.

### A. The cap is real, and the mic was on (confirmed without taking the port)

- Source: `src/config.h:149` `ENABLE_MIC 1`, `stream_espnow.cpp:60`
  `micEnabled = true` at boot, `QUALITY_DEFAULT QUALITY_LOW` (profile `max_fps`
  15.0) -> line 518 clamps it to 3.0.
- **Live proof that needs no serial access**: `/status.bytes` is
  `parser.bytes` = EVERY byte off the USB CDC port. Subtract the video payload
  (`frames x avg_frame`), the 12-byte video headers and `text`, and the
  remainder is 8402-8414 B/s. Quantised: `168305 / 252 = 667.9` datagrams over
  20.0 s = **33.39 pkt/s** against the designed `8000/240 = 33.33` pkt/s
  (100.17 %). Magic-scan resync garbage cannot land on a whole number of
  252-byte datagrams at the design rate, and the 57 skips counted in that window
  would each have to be ~2953 B long.
- `gap_med 334 ms` = `1000/3.0` exactly, `p95 669` = 2x. A saturated
  ACK-waiting send loop smears; exact multiples of the slot is a *gate*.
- Toggled the mic ONCE through the viewer's own `/ctl` (never touched the port;
  the serial probe was refused by the flock guard and wrote zero bytes):
  residual 8387 -> 4 -> 8397 B/s. Cause confirmed on hardware.

### B. Turning the mic off alone is a NET LOSS (measured 0.80x)

    A mic as found   fps=2.65  gap_med=334  drop= 6.7%  avg_frame= 2915 (13 seg)
    B mic OFF        fps=2.12  gap_med=334  drop=32.3%  avg_frame=10223 (43 seg)
    C restored       fps=2.50  gap_med=334  drop= 0.0%  avg_frame= 2914

With `micEnabled` false the block at line 622 is gone, so `adaptiveStep()`
climbs LOW -> MEDIUM. `cameraApplyQuality()` calls `adaptiveReset()`, and
`adaptiveStep()` fires once `aq_count >= 8`; at ~39 segment reports/s that is
**~0.2 s**. `avg_frame` 2915 -> 10223 = 3.5x the bytes, and a 43-segment frame
needs every segment to assemble -> 32.3 % frame loss. A mic-off S3 therefore
sits at VGA and is SLOWER than mic-on LOW.

### C. LOW cannot be held from the dashboard either

13 x `'0'` over 26 s left `avg_frame` at 10347. The relay path is NOT at fault
(checked in source: the C3 sends `{'Q', ch}` and the S3 wants `{'Q', '0'..'2'}` -
they match). The climb simply needs only 8 reports (~0.2 s), so any button or
script interval is ~10x too slow, and `ESPNOW_MAX_QUALITY = QUALITY_MEDIUM` is
where a mic-off sender always ends up.

### D. The real ceiling: ~7.8 ms per ACK-waited segment

Derived two independent ways and consistent: LOW = 13 seg -> ~101 ms of airtime
(the 334 ms gate binds); MEDIUM = 43 seg -> ~335 ms (airtime binds and *lands on*
the same 334 ms). That is why `gap_med` never moved across all three phases -
two different limiters printing the same number, which is exactly what made the
mic-only theory look airtight.

Consequence: with the mic off AND LOW actually held, the 66.7 ms gate stops
binding and airtime permits **~9-10 fps** (13 x 7.8 ms = 101 ms). Still a model
prediction - see the verification note below.

### E. The fix: FPS-priority mode (`'F1'` / `'F0'`)

mic OFF + quality PINNED at LOW + the up-stepper blocked, so the freed airtime
goes to frame rate. An **idempotent SET, deliberately not a toggle** - ESP-NOW
broadcasts are not ACKed, and the `'M'` mic toggle's even-delivery desync is
exactly the trap being avoided here.

`dashboard [fps] -> /ctl?cmd=F1 -> C3 handleCommand('F') + readDecArg ->
relayToSender({'F','1'}, re-sent 3x) -> S3 espNowOnRecv -> setFpsPriority(true)`.

- `src/config.h`: `ESPNOW_FPS_PRIORITY_MAX_FPS` (15.0), `FPS_PRIORITY_DEFAULT`.
- `src/stream_espnow.cpp`: `fpsPriority` state, the `'F'` receive branch,
  `setFpsPriority()`, the poll-side pin + `max_fps` override + up-stepper block,
  and `'f'` on the S3's own console for bench work.
- `receiver/src/receiver_config.h` / `receiver.cpp`: `FPS_CTRL_TAG_F`, the relay
  branch, the console banner.
- `host/c3_usb_mjpeg.py`: `/ctl` accepts `F[01]`, `/status` carries `fps_mode`,
  the `[fps]` dashboard button.
- Drops audio by design: the link cannot carry both (that is why the mic cap
  exists at all).

### F. Verified / not verified

**Verified.** Both builds clean: S3 RAM 15.6 % / Flash 25.4 %; C3 RAM 34.5 % /
Flash 55.8 % (730 996 B, +218 B = the new relay branch).
`host/test_reader_recovery.py` reports **68 checks, 0 failures**, including 9 new
ones: `/ctl?cmd=F1|F0` accepted and relayed, `/status fps_mode` 1 then 0, and
`F9` / `F` / `F1x` / `f1` all rejected. The live viewer was untouched throughout
(0 reconnects, 0 errors); the change is confirmed NOT live by the running
viewer's `/status` having no `fps_mode` key.

**Not verified.** The firmware behaviour itself. Neither board carries this
build, so `setFpsPriority()`, the LOW pin and the `max_fps` override have never
run, and **~9-10 fps remains a prediction, not a measurement**. First on-bench
check: with fps mode on, `avg_frame` should sit at ~2900 and `fps_mode` should
read 1; if the S3 ever reports ~10300 with fps mode on, the pin lost the race and
the climb is faster than measured here.

**Next lever if more is wanted.** The ~7.8 ms/segment is mostly the ACK wait plus
retries (2 attempts, 30 ms timeout each). A straight port of the audio path's
fire-and-forget trick to video is NOT safe: at 13 segments a 10 % per-segment
loss leaves only `0.9^13 = 25 %` of frames assembling. Lossy video needs FEC or
fewer segments covering the frame, not fewer retries.

---

## Context snapshot (updated 2026-10-16)

- 2026-10-16: host viewer made self-healing (reader thread no longer dies - that
  was the "restart it" bug, issue #3), the `g` keep-alive actually re-arms the C3
  again (issue #10), live link RSSI on the dashboard (issue #9), regression
  harness `host/test_reader_recovery.py` (68 checks, no hardware).
- 2026-10-16 (later): the "can it be faster" question answered by measurement -
  issue #11. The 3 fps cap IS the mic (confirmed live via the USB byte residual),
  but turning the mic off makes it SLOWER (0.80x: the up-stepper spends the freed
  airtime on VGA). New FPS-priority mode (`'F1'`/`'F0'`) pins LOW + mic-off
  instead; built and host-verified, NOT yet flashed.
- Video S3→C3→USB→dashboard works; C3 reassembles 8–14 KB frames reliably.
- Audio chain end-to-end; host serves `/audio.pcm`, dashboard plays it live
  via Web Audio (issue #1 fix). Audio air cost reduced (lossy send + 5 fps
  video cap while mic on, issue #2 fix).
- Recent fixes already in tree: mid-frame USB pump stall/resume, latestBuf
  mutex, WiFi watchdog on C3, port flock, g keep-alive, x8 mic gain,
  /audio.wav backlog flush (superseded by /audio.pcm).
- Firmware build (`pio run -e esp32s3_sense_sender`) passes; host script
  syntax-checked and HTTP endpoints smoke-tested without hardware.
- 2026-10-13: microSD recording merged in from `camera.zip` (issues #6, #7);
  recorder lives in the ESP-NOW build only, driven from the dashboard or the
  S3's serial console, with a selectable SD-vs-stream priority.
