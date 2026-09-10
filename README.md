# ESP32-S3 Sense camera streaming

Live camera streaming for the **Seeed Studio XIAO ESP32-S3 Sense** (OV3660 / OV2640-family
camera, 8 MB PSRAM). Two receivers are supported, selected by a single **flag**
in `src/config.h`.

| Source | Receiver | When to use |
|--------|----------|-------------|
| `STREAM_TARGET_BROWSER` | Phone / laptop web browser | Short‑range testing, low‑latency viewing, zero extra hardware |
| `STREAM_TARGET_ESP32C3` | ESP32‑C3 with dish (high‑gain antenna) | Long‑range / field link, then re‑served to a browser by the C3 |

```
.
├── platformio.ini          # SENDER project  (env: esp32s3_sense_sender)
├── src/                    # ======== sender ========   (board seeed_xiao_esp32s3)
│   ├── main.cpp            #   entry point (wires the selected target)
│   ├── config.h            #   <-- STREAM_TARGET flag lives here
│   ├── camera_pins.h       #   XIAO S3 Sense GPIO->camera pin map (verified)
│   ├── camera_hal.cpp/.h   #   capture, quality profiles, adaptive engine
│   ├── stream_browser.cpp/.h   # HTTP MJPEG target (phone/browser)
│   └── stream_espnow.cpp/.h    # segmented ESP-NOW target (C3)
└── receiver/               # ======== "dish" side ========  (separate nested project)
    ├── platformio.ini      #   RECEIVER env:  esp32c3_receiver
    └── src/                #   board seeed_xiao_esp32c3
        ├── receiver.cpp
        └── receiver_config.h
```

> The receiver is a **separate nested sub-project** because PlatformIO has no
> per-environment source directory option; the two boards can't share one
> `platformio.ini` cleanly, so each has its own.

---

## 1) Pick the receiver (the flag)

Open `src/config.h` and set `STREAM_TARGET`:

```cpp
#define STREAM_TARGET STREAM_TARGET_BROWSER   // or STREAM_TARGET_ESP32C3
```

There is a compile-time `#error` guard if you type anything else.

---

## 2) Build & flash

Requires [PlatformIO](https://platformio.org). The camera `esp_camera` driver is
**bundled in the arduino‑esp32 core**, so there is **no extra `lib_deps`** for the
camera (this is how Seeed's own examples build).

```bash
# Sender (the S3 Sense with the camera)  - run from the project root
pio run -e esp32s3_sense_sender
pio run -t upload -e esp32s3_sense_sender

# Receiver (the C3 with the dish)  - run from the receiver/ folder
cd receiver
pio run -e esp32c3_receiver
pio run -t upload -e esp32c3_receiver
cd ..
```

Both boards act as / join an access point **"XIAO-Sense"** (credentials in
`src/config.h` / `receiver/src/receiver_config.h`).

### Browser mode
1. Flash the S3 with `STREAM_TARGET_BROWSER`.
2. Phone/PC joins **XIAO-Sense** and opens `http://192.168.4.1/`.
3. Quality: buttons on the page, or serial `q0`/`q1`/`q2`.

### ESP-NOW (dish) mode
1. Flash the receiver C3 first. After it connects it prints its **MAC** on serial
   (or type `m`).
2. Put that MAC in `src/config.h` → `ESPNOW_PEER_MAC` and flash the S3 with
   `STREAM_TARGET_ESP32C3`.
3. Phone/PC joins **XIAO-Sense** and opens `http://192.168.4.2/` — that page shows
   what the C3+dish received.

> Tune the ESP‑NOW link with the dish: point the high‑gain element at the S3's
> antenna. Range, orientation and the 2.4 GHz environment dominate throughput.

---

## 3) Adjustable quality

Quality is a set of three presets (defined in `src/camera_hal.cpp`), each mapping
a resolution + JPEG quality + frame rate:

| Level | Resolution | JPEG q (0-63, lower=better) | FPS cap |
|-------|------------|-----------------------------|---------|
| `q0` LOW   | QVGA 320x240   | 35 | 15 |
| `q1` MED   | VGA 640x480    | 20 | 10 |
| `q2` HIGH  | UXGA 1600x1200 | 12 | 5  |

Set the boot default with `QUALITY_DEFAULT` in `config.h`. Change at runtime:
- Browser mode: `GET /quality?level=0|1|2` (or the buttons on the page).
- Both modes: serial `q0` / `q1` / `q2` (works even mid-stream).
- ESP-NOW mode (automatic): an adaptive controller steps quality DOWN on a lossy
  link and steps back UP when it recovers (`ADAPTIVE_*` knobs in `config.h`).

## 4) Optimization & error tolerance

**Browser target**
- Frames are captured straight into PSRAM with `CAMERA_GRAB_LATEST`, so a slow
  client always gets the newest frame (stale frames are skipped, never queued).
- Streaming is **non-blocking and time-sliced**: each loop pass pushes only a
  small bounded slice, so a huge UXGA frame or a slow client can never stall the
  server or wedge the board - it just lowers the stream's frame rate.
- Oversized frames (> `BROWSER_MAX_FRAME_BYTES`) are automatically downscaled.
- If the camera stops producing frames, it is automatically re-initialised.

**ESP-NOW target**
- Each JPEG is split into <=240-byte datagrams carrying a binary header
  (magic, rolling frame id, segment index, total, payload len).
- Every datagram rides ESP-NOW's link-layer ACK; a segment that times out is
  retransmitted up to `ESPNOW_SEND_RETRIES` times.
- Frames are **dropped, never queued**, so the camera never backlogs.
- If a frame exceeds the C3's RAM cap (`RECEIVER_MAX_FRAME_BYTES`) the sender
  auto-downscales. A sliding window of per-segment failures drives the adaptive
  quality controller.

**Receiver (C3)**
- Two ring buffers (assembling / last-complete) + a received-segment bitmap;
  incomplete or stale frames are discarded after `FRAME_ASSEMBLE_TIMEOUT_MS`, so
  one dropped datagram costs a single frame, not the stream.
- Re-serves the latest complete frame as MJPEG for a browser on the dish side.

## 5) Config cheat-sheet

| Symbol | File | Meaning |
|--------|------|---------|
| `STREAM_TARGET` | `src/config.h` | **The flag**: `_BROWSER` or `_ESP32C3` |
| `QUALITY_DEFAULT` | `src/config.h` | Boot quality preset |
| `WIFI_SSID` / `WIFI_PASS` / `WIFI_CHANNEL` | both configs | AP credentials + channel (must match) |
| `ESPNOW_PEER_MAC` | `src/config.h` | C3 receiver MAC |
| `RECEIVER_MAX_FRAME_BYTES` | both configs | Frame cap (receiver RAM limit) |
| `BROWSER_MAX_FRAME_BYTES` | `src/config.h` | Browser frame cap (auto-downscale above this) |
| `ADAPTIVE_*` | `src/config.h` | Auto quality-step thresholds |
| `ESPNOW_SEND_RETRIES` | `src/config.h` | Per-segment retransmit budget |
| `FRAME_ASSEMBLE_TIMEOUT_MS` | `receiver/src/receiver_config.h` | Stale-frame discard time |

## 6) Notes & limitations

- `camera_pins.h` uses the authenticated pin set for this board from Seeed's
  CameraWebServer (XCLK=10, SIOD=40, SIOC=39, VSYNC=38, HREF=47, PCLK=13,
  Y2..Y9 = 15,17,18,16,14,12,11,48, PWDN/RESET = -1).
- The C3 has **no PSRAM**, so keep frames small on the ESP-NOW path; presets
  LOW/MED are the practical sweet spot for a dish link.
- ESP-NOW throughput is dominated by ACK round-trips; expect roughly 1-5 FPS over
  the air depending on quality and environment.
- The browser target serves **one MJPEG stream at a time**, but its handler is
  non-blocking/time-sliced, so the quality buttons and serial `q0`/`q1`/`q2` stay
  responsive live during a stream (quality changes via the page use AJAX and do
  not reload the page).
- The optional NACK channel (sender `ESPNOW_USE_NACK` + receiver
  `ENABLE_ESPNOW_NACK`) is currently a stub hook for future loss-feedback; the
  active error-tolerance is per-segment ACK + retry + adaptive quality.