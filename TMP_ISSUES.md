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

**Status: PARTIALLY ADDRESSED, NOT VERIFIED (no code change this pass).**

flock-exclusive port + `g` keep-alive are in place; the known remaining test
is a single clean host instance vs. a stray `pio device monitor`, and if it
still reproduces, instrumenting the C3 to log WHICH port the 'x' came from
(`Serial0` vs `Serial`). Requires hardware to verify.

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

## Context snapshot (updated 2026-10-13)

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
