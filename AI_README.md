# AI Handoff README — ESP32-S3 Sense camera streaming

> **Purpose:** This file lets a *fresh* AI assistant (or engineer) pick up this
> project quickly, understand exactly what exists, and implement the next task
> without re-discovering everything. Read this first.

---

## 1. Project at a glance

Live camera streaming from a **Seeed Studio XIAO ESP32-S3 Sense** (OV3660 /
OV2640-family camera, 8 MB PSRAM). Two receiver targets are selectable via a
single flag. The project uses **PlatformIO + arduino-esp32 core 2.0.17**
(PlatformIO framework version `4.20017`).

```
.
├── platformio.ini          # SENDER project (env: esp32s3_sense_sender, board seeed_xiao_esp32s3)
├── src/                    # sender =====
│   ├── main.cpp            #   entry point; wires STREAM_TARGET
│   ├── config.h            #   <-- THE FLAG + all tunables
│   ├── camera_pins.h       #   XIAO S3 Sense GPIO->camera pin map (verified)
│   ├── camera_hal.cpp/.h   #   capture, quality profiles, adaptive engine, cameraReinit
│   ├── camera_debug.cpp/.h #   gated boot + camera diagnostics (DEBUG_CAMERA)
│   ├── recorder.cpp/.h     #   microSD MJPEG (.avi) recorder + priority policy
│   ├── stream_browser.cpp/.h   # HTTP MJPEG target (phone/PC browser)
│   └── stream_espnow.cpp/.h    # segmented ESP-NOW target (C3 dish)
├── receiver/               # receiver "dish" side ===== (NESTED sub-project)
│   ├── platformio.ini      #   env: esp32c3_receiver, board seeed_xiao_esp32c3
│   └── src/
│       ├── receiver.cpp          #   ESP-NOW receive + reassemble -> USB CDC forward
│       └── receiver_config.h
├── host/                   # host viewer ===== (stdlib-only Python, no deps)
│   └── c3_usb_mjpeg.py     #   opens C3 USB CDC, CRC-validates, MJPEG at :8080
│                           #   + dashboard controls (quality/mic/SD recording)
```

**Important structure note:** PlatformIO does **not** support per-environment
`src_dir`, so the receiver is a **separate nested project** in `receiver/` with
its own `platformio.ini`.

## 2. The flag (what selects the receiver)

`src/config.h`:

```c
#define STREAM_TARGET  STREAM_TARGET_BROWSER   // or STREAM_TARGET_ESP32C3
```

- `STREAM_TARGET_BROWSER` = HTTP MJPEG server (phone/PC). Current default.
- `STREAM_TARGET_ESP32C3`  = segmented ESP-NOW to the C3 dish receiver.

There is a compile-time `#error` guard for invalid values.

## 3. Build commands (verified working)

`pio` CLI: `/home/willi/.platformio/penv/bin/platformio` (PlatformIO Core 6.2.0).

```bash
# Sender (S3 Sense) - from project root
pio run -e esp32s3_sense_sender && pio run -t upload -e esp32s3_sense_sender

# Receiver (C3 dish) - from receiver/ folder (its own project)
cd receiver
pio run -e esp32c3_receiver && pio run -t upload -e esp32c3_receiver
```

Both build **cleanly with zero warnings**. `STREAM_TARGET` was verified to build
for **both** values (browser + esp32c3).

## 4. Environment / toolchain facts (critical)

- **Host:** Linux; `pio` at `/home/willi/.platformio/penv/bin/platformio`.
- **Framework actually used:** `framework-arduinoespressif32 @ 4.20017` = upstream
  arduino-esp32 **2.0.17**. Installed under
  `~/.platformio/packages/framework-arduinoespressif32/`.
- **ESP-NOW API:** include `#include <esp_now.h>` (underscore; the `ESP_NOW`
  Arduino lib doesn't ship in this core). Functions return **`esp_err_t`**
  (`ESP_OK==0`), callback recv signature is `(const uint8_t* mac, const uint8_t* data, int len)`.
- **Camera API:** `esp_camera.h` is baked into the core SDK (no `lib_deps`
  needed). Struct field is `camera_config_t.fb_location` (not `fb_loc`).
- **Gotcha:** my own helpers `bitGet/bitSet/bitClear` clashed with Arduino core
  macros; renamed to `segBitGet/segBitSet/segBitClear` in the receiver.
- **C3 pins (XIAO_ESP32C3 variant):** TX=21, RX=20, SDA=6, SCL=7, D0..D10 on
  2,3,4,5,6,7,21,20,8,9,10.
- `.gitignore` covers `.pio/` and `compile_commands.json`; there are `.cache/`
## 5. History of fixes (so you don't regress)

1. **`src_dir` not per-env** → split receiver into nested sub-project.
2. **`espnow.h` not found** → use `esp_now.h`.
3. **`fb_loc` wrong** → `fb_location`.
4. **Arduino `bitSet/bitClear` macro collision** (receiver) → `segBit*`.
5. **lwIP "Invalid mbox" crash on boot in browser mode** → `browserBegin()` now
   starts the soft AP **before** the WiFiServer (WiFi/lwIP stack must be up first).
6. **Quality buttons navigated page away** → page now uses AJAX `fetch()` to
   `/quality?level=N`; server made **non-blocking / time-sliced** so `/stream`
   and `/quality` are serviced concurrently (blocking `streamTo()` replaced with
   incremental `streamTick()` pushing ≤2 KB per loop pass).
7. **HIGH (1600×1200) "power-cycle to recover" freeze** → root cause was blocking
   frame writes starving the server + camera. Fixed via: non-blocking chunked
   writes (`SEND_SLICE_BYTES=2048`), frame-size cap + auto-downscale
   (`BROWSER_MAX_FRAME_BYTES=(256UL*1024)` in `config.h`), and a camera recovery
   (`cameraReinit(level)` in `camera_hal`, triggered after 100 consecutive NULL
   captures).
8. **USB CDC console "silent" / no frames** (USB refactor) → TinyUSB's CDC
   drops **every** `Serial` write while the USB host has not asserted DTR, and
   `availableForWrite()` reads 0, so boot text vanished and the frame pump
   stalled. Fixes: host asserts DTR/RTS via `ioctl(TIOCMBIS)`, firmware
   re-greets on the USB-connect rising edge (`usbAnnounced` in `loop()`), waits
   up to 2 s at boot for an already-open monitor, and reports WiFi progress on
   both ports.
9. **"Which board is this port?" + console visibility** (USB refactor polish) →
   the host probes each port with `x`+`s` and keeps the one that answers
   `latest frame ...` (the S3 answers `s` differently), waits 1.5 s so the C3's
   banner/MAC is captured *before* sending `g`, relays printable console text
   between frames to the terminal and a `/console` endpoint shown on the page,
   and counts console text separately from real dropped bytes so `skips` only
   reports genuine framing loss.

## 6. Current architecture

The C3 receiver used to do **both**: receive segmented ESP-NOW frames from the
S3 (RF receive), **and** serve them as MJPEG over Wi-Fi TCP (RF transmit).
Wi-Fi is **half-duplex** (TDD), so the C3's radio had to time-slice TX/RX on the
same channel → airtime contention → frame loss at higher quality.

That contention is now **gone**: the C3 no longer runs any Wi-Fi server. It
reassembles ESP-NOW frames and forwards them over the wired USB CDC link to a
host:

```
S3 (airborne) --ESP-NOW--> C3 dish --USB CDC (wired)--> host (Pi/laptop)
                              | RF receive + reassemble | decodes + HTTP MJPEG
                              (no RF TX while serving)   -> phone/PC viewers
```

**C3 output stage:** for each reassembled JPEG, the C3 writes a small framing
header over `Serial` (USB CDC on this board) — magic `0xC3 0x8C`, 32-bit
payload length (big-endian), frame id, and a CRC-32 of the payload — then the
JPEG bytes. The writer is **non-blocking** and driven by
`Serial.availableForWrite()` (it is 0 when no host is attached), so a slow host
causes the C3 to *drop/skip* frames instead of ever blocking RF reception.

The USB CDC is **dual-mode** so the `pio device monitor` stays usable:
- **console (default):** readable banner, `[rx]` logs, and the `m` (MAC) /
  `s` (status) commands answered on USB. Commands are accepted on both USB and
  the hardware UART (`Serial0`, GPIO21/20) and always mirrored there.
- **stream:** the host viewer sends `g` after opening the port; the C3 then
  streams framed JPEG only and mirrors text to `Serial0` (never USB, so log
  text can't corrupt the stream). `x` from either port — or the USB host
  closing the port — returns to console mode.

**Host viewer (`host/c3_usb_mjpeg.py`, stdlib-only):** auto-detects the C3's
port (e.g. `/dev/ttyACM0`), parses/CRC-validates the framing, and re-serves an
MJPEG stream at `http://<host>:8080/`. It resyncs by scanning for the magic so a
frame dropped mid-stream is just logged + skipped.

**Quality presets** (`camera_hal.cpp`): LOW=QVGA 320×240/jpegq35, MED=VGA
640×480/jpegq20 (default), HIGH=UXGA 1600×1200/jpegq12. Runtime change: serial
`q0/q1/q2` or page buttons. ESP-NOW path also has an adaptive quality controller.

## 7. ESP-NOW framing (must stay in sync sender↔receiver)

Shared constants float in `src/config.h` and `receiver/src/receiver_config.h`
(keep in sync manually):
- `ESPNOW_MAX_PAYLOAD = 250` (protocol max), `NOW_HEADER_BYTES = 10`,
  `NOW_PAYLOAD_BYTES = 240`.
- Segment header (big-endian): `magic(0xA3C3) frameId seg total plen` = 10 bytes.
- Sender does per-segment ACK + retry (`ESPNOW_SEND_RETRIES`), drops frames (never
  queues), auto-downscales on `RECEIVER_MAX_FRAME_BYTES` (32 KB) overflow.
- Receiver: two ring buffers + segment bitmap, stale-frame discard
  (`FRAME_ASSEMBLE_TIMEOUT_MS=200`), one lost datagram costs one frame.
- The C3 has **no PSRAM**, so ~32 KB frame cap is tight → useful only at LOW/MED.

**Second, the USB CDC output framing** (NEW, only on the C3 → host leg). These
constants float in `receiver/src/receiver_config.h` and `host/c3_usb_mjpeg.py`
(keep those two in sync manually; they cross-reference each other in comments):
- `USB_MAGIC_HI=0xC3`, `USB_MAGIC_LO=0x8C`, `USB_HEADER_BYTES=12`,
  `USB_BAUD=115200`.
- Layout: `[hi lo len(4BE) frameId(2BE) crc32(4BE)] + JPEG payload`.
- CRC-32 is the standard ISO-HDLC/zlib polynomial (matches Python `zlib.crc32`).
- The C3 writer is non-blocking (`availableForWrite()`), and the host parser
  resyncs by scanning for the magic when a frame is dropped mid-stream.

The ESP-NOW air format above is **untouched** — this USB format is purely an
additional output stage on the C3 receiver.

## 8. Done: forward frames over USB CDC to a host (was "PENDING TASK")

**Goal (completed):** the C3 receiver now forwards reassembled frames over a
**wired USB serial link** instead of running its own Wi-Fi MJPEG server, which
eliminates the RF TX/RX half-duplex contention. A host (laptop/Pi) serves the
MJPEG.

Final architecture:
```
S3 (airborne) --ESP-NOW--> C3 dish --USB CDC (wired, full-duplex)--> host (Pi/laptop)
                              | RF receive + reassemble + forward   | decodes + HTTP MJPEG
                              (no RF TX for serving)                -> phone/PC viewers
```

What changed (per the old sub-tasks):
1. **C3 firmware** (`receiver/src/receiver.cpp`): removed the `WiFiServer` /
   `handleHttp` / `INDEX_HTML` code. ESP-NOW receive + reassembly is unchanged.
   Added a non-blocking USB CDC pump that frames each complete JPEG as
   `[0xC3 0x8C][len:4BE][frameId:2BE][crc32:4BE] + payload` and writes it via
   `Serial` (USB CDC). `Serial.availableForWrite()` gates it, so a slow host
   drops/skips frames instead of stalling RF reception. The USB CDC is
   dual-mode: it boots as a readable console (`m`/`s` + logs work in the
   `pio device monitor`) and the host switches it to the frame stream with
   `g` (`x`, or the port closing, returns to console). Text is always mirrored
   to `Serial0` (hardware UART GPIO21/20) and in stream mode goes there ONLY,
   so it never corrupts the binary stream. Added a `crc32()` (zlib poly) helper.
2. **Host viewer** (`host/c3_usb_mjpeg.py`, stdlib-only): auto-detects the C3's
   USB CDC port (`/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/serial/by-id/*`), opens it
   in raw 8N1 via `termios` (no pyserial), parses + CRC-validates the framing in
   a resyncing `FrameParser`, and serves MJPEG at `http://<host>:8080/` with an
   `<img src='/stream'>`. Dropped/corrupt frames are logged + skipped.
3. **Docs:** `README.md` + this file updated; USB framing constants synced
   between `receiver/src/receiver_config.h` and `host/c3_usb_mjpeg.py`
   (cross-referenced in comments). `platformio.ini` files untouched.
4. **Verification:** receiver builds clean with `pio run` (zero warnings).
   Sender verified for **both** `STREAM_TARGET` values (see section 9). Host
   parser unit-tested against a captured byte stream (including injected byte
   drops); real-hardware link still requires the C3 plugged in.

Rules honored: ESP-NOW wire format + sender untouched; no new runtime deps on
the C3; host is stdlib only; framing constants synced both ways.

## 9. Status of the receivers

| Target | Status |
|--------|--------|
| `STREAM_TARGET_BROWSER` | Working. MJPEG over AP Web, non-blocking, quality live-switchable. Unchanged by section 11 (no recorder in this build). |
| `STREAM_TARGET_ESP32C3` | Working (FW). C3 reassembles ESP-NOW and forwards framed JPEG over USB CDC to `host/c3_usb_mjpeg.py`, which re-serves MJPEG at :8080. Needs hardware to confirm live pixels. |
| `STREAM_TARGET_ESP32C3` + `ENABLE_SD_RECORDING` | Working (FW + host). Records the same frames to microSD as `.avi`, controlled from the dashboard; SD-vs-stream priority selectable. Built-in tests pass (section 11 §verification); the air control/status link and the real card still need hardware. |

`STREAM_TARGET` is currently `BROWSER` (unchanged).
  (clangd) and `.vscode/` artifacts that are also ignored.
  (clangd) and `.vscode/` artifacts that are also ignored.

## 10. Debug flag: `DEBUG_CAMERA` (boot + camera diagnostics)

`src/config.h`:

```c
#define DEBUG_CAMERA  1   // 0=off, 1=boot+self-test, 2=+per-frame logging
```

Added because a board with a **bad camera module** looks identical to a firmware
bug (black stream / `[cam] init FAILED`). The flag prints everything needed to
tell the two apart, and is fully compiled out at `0` (no size/speed/noise cost).

- New module: `src/camera_debug.cpp/.h`. Hooks are called from `camera_hal.cpp`
  (init/re-init result + every capture) and `main.cpp` (boot dump + serial `d`),
  so both stream targets are covered without touching `stream_*` files.
- Prints: reset reason, chip/flash/heap/PSRAM, the GPIO->camera pin map, the
  decoded `esp_camera_init()` result, the sensor SCCB identity (`PID`/`MID`/`VER`
  + part name) and a live capture probe (dimensions, bytes, `FF D8` check).
- The smoking gun for a bad module: `esp_camera_init()` ->
  `ESP_ERR_CAMERA_NOT_DETECTED (0x20001)` and/or `PID` = `0x00`/`0xFF`
  (nothing answers on SCCB) -> swap/reseat the camera. **Swapping just the
  camera fixed exactly this here.**
- Serial command **`d`** re-dumps on demand (both targets); `s` adds the running
  capture-failure count; the first capture failure auto-dumps the sensor probe.
- Serial command **`t`** runs a camera **bus sweep** (XCLK 20/16/10 MHz x
  PSRAM/DRAM, 3 capture attempts each): the definitive test for "SCCB ID reads
  fine but it never frames" -> dead DATA path (module/ribbon). A frame only
  counts if it is >= 512 B and starts `FF D8` (a smaller buffer is a stale
  DMA buffer, not a picture). The normal clock is `CAMERA_XCLK_HZ` and the
  frame-buffer location is `CAMERA_FB_LOCATION` (PSRAM default) in `config.h`.
  **Measured on this board's OV3660 (2026-10-13): 20 MHz and 10 MHz frame,
  16 MHz does NOT (init OK, 0 bytes) - 16 MHz is the ESP32-S3 PSRAM-DMA path
  that breaks JPEG capture.** So when the sweep says 4/6 modes work, the
  "camera failed" is a `config.h` clock problem, not a dead module; set
  `CAMERA_XCLK_HZ 20000000`.
- Sweep takes ~2.5 min when every mode fails (each failed capture waits for the
  esp_camera timeout). `gdma_disconnect` error lines during the sweep are the
  driver's de-init noise and are harmless.
- Builds verified clean (zero warnings) for `STREAM_TARGET` = BROWSER and
  ESP32C3, and for `DEBUG_CAMERA` = 0, 1 and 2.

## 11. microSD recording merged in (from the standalone `camera.zip` sketch)

**What was merged.** `camera.zip` was a standalone XIAO ESP32S3-Sense **AVI
recorder** sketch (SD mount + Motion-JPEG `.avi` container writer + serial menu).
It is now `src/recorder.cpp/.h` in this repo's sender: SD mount (CS=GPIO21,
SPI 7/8/9, 20 MHz with a 4 MHz retry), the AVI writer (`RIFF`/`hdrl`/`movi`/`idx1`),
the EOI frame trim (`recorderJpegLength()`), the clip report, and the settings
(`max seconds`, `priority`). The zip's own project files were **not** copied in:
the ESP-NOW sender already owns the camera, so the recorder is a module its
streaming loop feeds, not a second project.

**Scope rule (important).** The recorder is wired into the **ESP-NOW
(`STREAM_TARGET_ESP32C3`) path only**. `src/config.h` `#undef`s
`ENABLE_SD_RECORDING` to 0 for the browser/phone target, so that build contains
no SD code, never mounts the card and has no recorder commands.
Measured cost: browser build 761,545 B → **761,569 B** (+24 B, the shared EOI
trim helper inside the always-compiled `stream_espnow.cpp`); ESP-NOW build
**849,777 B**. Both build with zero warnings.

**One capture, two consumers** (`stream_espnow.cpp` → `espnowPoll()`). Each pass
captures one frame and hands that same JPEG (trimmed once, used by both) to the
SD writer and/or the air link. Whose timing is protected when one frame slot
(`1000/max_fps` ms) cannot afford both is `RECORD_PRIORITY_*`:

| Priority | Behaviour |
|----------|-----------|
| `SD` (0) | card first; if the write ate the slot the frame is NOT streamed |
| `BALANCED` (1, default) | always both (card first, then air); loop rate absorbs it |
| `STREAM` (2) | air first; if the send ate the slot the frame is NOT written |

A frame too large for the C3 (`RECEIVER_MAX_FRAME_BYTES`) is still recorded under
SD/balanced (valid picture) and only skipped on the air, which also triggers the
existing auto-downscale.

**Control / status protocol** (all tiny, lossy, one-way per direction; tags
documented in `src/config.h` §5b and `receiver/src/receiver_config.h`):

| Direction | Payload | Meaning |
|-----------|---------|---------|
| C3 → S3 (broadcast) | `{'Q', lvl}` | quality preset (existing) |
| C3 → S3 | `{'M'}` | mic toggle (existing, sent ONCE - not idempotent) |
| C3 → S3 | `{'R', '0'/'1'}` | stop / start SD recording |
| C3 → S3 | `{'T', secHi, secLo}` | max clip seconds (0 = until stopped) |
| C3 → S3 | `{'P', '0'..'2'}` | SD/stream priority |
| S3 → C3 (peer MAC) | `{'S', on, sd, prio, frames(2BE), left(2BE), idx(2BE)}` | recording status |

The `R`/`T`/`P` relays are sent 3× (broadcasts are not ACKed) via
`relayToSender()`; `M` stays single-shot. Start/stop requests only set a flag in
`onRecv()` (WiFi task) and are serviced by `espnowPoll()` (loop task) - SD I/O
must not run on the RF receive path. A start is serviced on the next captured
frame because the AVI header needs the frame's real width/height.

**Dashboard.** `host/c3_usb_mjpeg.py` gained an "SD recording" card: START REC /
END REC, "END AFTER (seconds)" + SET, and SD/BALANCED/STREAM. Commands travel
dashboard → `/ctl` → C3 serial (`r`, `e`, `t<secs>`, `p<0..2>`) → ESP-NOW → S3.
`/ctl` now **validates** with `re.fullmatch` instead of filtering characters, so
junk containing an `r` can no longer start a recording (it used to: the filter
turned `"drop me!!r"` into `"rper"`).

**Status back to the page.** S3 → C3 status datagram every
`RECORD_STATUS_PERIOD_MS` (2 s) → the C3 prints
`[rx] REC state=on idx=3 frames=142 left=17s prio=1 sd=1` → the page parses it
with `REC_LINE_RE` and shows clip number, frames, seconds left, priority and
card-present. Because `onRecv()` runs in the WiFi task, it only *queues* that
line; `loop()` flushes it to USB only when the pump is between frames
(`outState == 0`), so console text can never land inside a half-sent JPEG.

**S3 serial keys** (dispatched from `main.cpp`, the only `Serial` reader, into
`espnowHandleSerialKey()`): `r` start, `e` stop, `D<secs>` max length,
`P<0..2>` priority, plus `q0/q1/q2`. `espnowHandleSerialQ()` and
`browserHandleSerialQ()` are effectively dead code (`main.cpp` drains `Serial`
first), which is exactly why the new keys are dispatched from `main.cpp`.

**Verification actually run** (nothing below is assumed - all executed):

1. `pio run` for the sender in **both** `STREAM_TARGET` values and for the
   receiver: clean, zero warnings.
2. `src/recorder.cpp` compiled **natively** (g++ against tiny Arduino/SD/SPI
   stubs) and executed: mount, settings + clamping (3600 s cap, invalid
   priority → default), EOI trimming of a padded frame, a 24-frame clip, a
   second clip not overwriting the first, auto-stop at the length limit, an
   unlimited clip surviving `recorderTick()`, status lines, and the
   start/write/stop refusals. The `.avi` files it wrote were then checked with
   `ffprobe` (mjpeg, correct width/height/rate, `nb_frames`) **and** by parsing
   RIFF/`avih`/`strh`/`strf`/`movi`/`idx1` directly: header frame count and
   `idx1` count == frames written, uniform trimmed chunk size, every chunk
   starting `FF D8` and ending `FF D9`.
3. `host/c3_usb_mjpeg.py` driven over real HTTP in-process: frame parser with
   interleaved console text + audio + a corrupt header (resync), `/status` JSON,
   every `/ctl` command forwarded verbatim, malformed commands rejected, and the
   firmware `snprintf` format re-formatted and matched against `REC_LINE_RE`.
4. **Still needs hardware** (no boards attached here): the ESP-NOW control/status
   datagrams over the air, the real SD card (speed, FAT32, the J3 pad), the
   dashboard buttons end-to-end, and the actual priority behaviour under load.
