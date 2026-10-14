# ESP32-S3 Sense camera streaming

Live camera streaming for the **Seeed Studio XIAO ESP32-S3 Sense** (OV3660 / OV2640-family
camera, 8 MB PSRAM). Two receivers are supported, selected by a single **flag**
in `src/config.h`.

| Source | Receiver | When to use |
|--------|----------|-------------|
| `STREAM_TARGET_BROWSER` | Phone / laptop web browser | Short‑range testing, low‑latency viewing, zero extra hardware |
| `STREAM_TARGET_ESP32C3` | ESP32‑C3 with dish → **wired USB to a host that serves the MJPEG** | Long‑range / field link; keeps the C3 RF receive‑only (no half‑duplex TX contention) |

In dish mode the C3 does **not** serve video over Wi-Fi any more. It reassembles
the ESP-NOW frames and forwards them over **USB CDC** to a host (Pi/laptop),
which runs `host/c3_usb_mjpeg.py` to re-serve an MJPEG stream to any browser:

```
S3 (airborne) --ESP-NOW--> C3 dish --USB CDC (wired)--> host (Pi/laptop)
                              | RF receive + reassemble | decodes + HTTP MJPEG
                              (no RF TX while serving)   -> phone/PC viewers
```

```
.
├── platformio.ini          # SENDER project  (env: esp32s3_sense_sender)
├── src/                    # ======== sender ========   (board seeed_xiao_esp32s3)
│   ├── main.cpp            #   entry point (wires the selected target)
│   ├── config.h            #   <-- STREAM_TARGET flag lives here
│   ├── camera_pins.h       #   XIAO S3 Sense GPIO->camera pin map (verified)
│   ├── camera_hal.cpp/.h   #   capture, quality profiles, adaptive engine
│   ├── camera_debug.cpp/.h #   gated boot + camera diagnostics (DEBUG_CAMERA)
│   ├── stream_browser.cpp/.h   # HTTP MJPEG target (phone/browser)
│   └── stream_espnow.cpp/.h    # segmented ESP-NOW target (C3)
├── receiver/               # ======== "dish" side ========  (separate nested project)
│   ├── platformio.ini      #   RECEIVER env:  esp32c3_receiver
│   └── src/                #   board seeed_xiao_esp32c3
│       ├── receiver.cpp            # reassemble + forward over USB CDC
│       └── receiver_config.h
└── host/                   # ======== host viewer ========  (stdlib-only Python)
    └── c3_usb_mjpeg.py     #   opens the C3 USB CDC port, CRC-validates,
                            #   re-serves MJPEG at http://<host>:8080/
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

**What goes where:** only the **C3 dish receiver** is plugged into the computer
running `host/c3_usb_mjpeg.py`. The **S3 sender is a separate airborne device** —
it powers itself and streams over the air. Plug the S3 into a computer only to
flash it or watch its serial monitor (unplug it afterwards; two boards on one
USB host is why the port name can bounce between `/dev/ttyACM0/1/2`).

1. Flash the receiver C3 first. Open `pio device monitor -e esp32c3_receiver`
   (or press reset after opening it) — the C3 greets you with its **MAC** in
   console mode; you can also type `m` there.
2. Put that MAC in `src/config.h` → `ESPNOW_PEER_MAC` and flash the S3 with
   `STREAM_TARGET_ESP32C3`.
3. Plug the C3 into the host over **USB** (its USB CDC port shows up as
   `/dev/ttyACM0` on Linux). On the host run the stdlib-only viewer:

   ```bash
   cd host
   python3 c3_usb_mjpeg.py              # auto-detects the port, serves :8080
   # or pin a specific device:  python3 c3_usb_mjpeg.py --port /dev/ttyACM0
   ```

4. Open `http://<host>:8080/` on any phone/PC — that page shows what the
   C3+dish received and re-serves it as MJPEG. It also shows the **C3's serial
   console** (banner, hints, frame logs) under "C3 serial console", so you can
   debug the link without a serial terminal.

**What the C3 forwards:** each reassembled JPEG frame is written to `Serial`
(USB CDC) as a length-prefixed + CRC-32 framed record (magic `0xC3 0x8C`,
32-bit length, frame id, CRC-32 of the payload, then the payload). The host
script parses and CRC-validates every frame and **gracefully skips** any frame
that was dropped mid-stream (slow host or a lost ESP-NOW segment) by rescanning
for the magic — it never needs to reread. Because the C3 uses
`Serial.availableForWrite()`, a slow host can never stall RF reception.

**Diagnostics & commands (`m`=MAC, `s`=status, `g`=start stream, `x`=stop)
are accepted on BOTH serial ports.** The C3's USB CDC boots in **console
mode**, so `pio device monitor` shows the banner/`[rx]` log lines and typing
`m` / `s` there just works. `host/c3_usb_mjpeg.py` sends `g` right after
opening the port to switch the C3 into **stream mode** (binary frames only;
log text is then mirrored to the hardware UART so it can't corrupt the JPEG
stream), and sends `x` on exit — or you can just close the monitor — which
hands the readable console back.

> Tune the ESP‑NOW link with the dish: point the high‑gain element at the S3's
> antenna. Range, orientation and the 2.4 GHz environment dominate throughput.

#### Troubleshooting the USB link

| Symptom | Cause / fix |
|---------|-------------|
| `pio device monitor` shows **nothing at all** | TinyUSB drops all bytes until the host asserts DTR. Open the monitor first, **then press the C3's reset button** — the firmware re-greets whenever a monitor attaches, so you can also just re-open the monitor. |
| Monitor works, but the host script shows 0 frames | Only one program can hold the port. Close the monitor, then run `python3 c3_usb_mjpeg.py` (it asserts DTR/RTS and sends `g` itself). Check the page's status line (`frames=/skips=`) — nonzero `skips` means data was arriving before the parser synced. |
| Several boards share one machine (`/dev/ttyACM0`, `ACM1`, ...) | The script now *probes* each port and reports `C3 identified on <port>`; pass `--port /dev/ttyACMx` to pin it. If it says "no C3-style answer", the C3 was still joining Wi-Fi or is the wrong board. |
| Host console shows `[c3] streaming, but NO ESP-NOW frames yet ...` | The C3's USB link is fine but nothing is arriving over the air. On the S3: check `ESPNOW_PEER_MAC` in `src/config.h` matches this C3's MAC (type `m` in the C3 monitor), and check the S3 for brownout. |
| `Resource busy` / port grabbed | A desktop service (ModemManager, brltty) may claim `/dev/ttyACM0` when it appears. Stop it (`sudo systemctl stop ModemManager`) or add a udev rule, then re-run. |
| Status shows `skips` growing but `frames=0` | Real framing loss: the C3 is forwarding but the host never gets a whole frame — slow host or the sender keeps tearing frames (see the row above). Console text (banner/hints) is **not** counted as skips any more, so a large `skips` means genuine binary loss. |
| The S3 gets **hot** and dies after ~10 s | That's the sender retry-blasting ESP-NOW segments at a peer that never ACKs (the `[q] set LOW` message is the adaptive controller reacting to 100% segment failures). Fix the MAC/channel match first; meanwhile set `ESPNOW_SEND_RETRIES 1` and `QUALITY_DEFAULT QUALITY_LOW` in `src/config.h`, and power it from a real 5 V/2 A source. If it stays hot even with no peer configured, suspect hardware (camera flex shorting / board fault). |
| No frames at all, and the monitor shows only `.` dots | The C3 is still joining the S3 AP (`XIAO-Sense`). Bring the S3 sender up first; ESP-NOW needs both radios on the same channel. |
| Status shows `bytes` climbing then stopping, `frames=0`, and the S3's LED goes out | **Sender brownout.** The OV2640 + PSRAM + Wi-Fi TX on the S3 Sense peaks well above 500 mA. Run `pio device monitor -e esp32s3_sense_sender` on the S3 — a `Brownout detector was triggered` line confirms it. Fix the supply (≥1 A USB adapter or powered hub, short/thick cable, or a LiPo on the S3's battery pad). Typing `q0` on the S3's monitor drops it to QVGA, which shrinks the peaks for bench testing. |

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
- Forwards each complete frame over **USB CDC** to the host viewer, framed with
  magic + 32-bit length + frame id + CRC-32. The forwarder is non-blocking and
  uses `Serial.availableForWrite()`, so a slow host simply makes the C3 drop
  frames - it can never stall RF reception. There is no more Wi-Fi MJPEG on the
  C3 (that was the half-duplex contention this design removes).

## 5) Config cheat-sheet

| Symbol | File | Meaning |
|--------|------|---------|
| `STREAM_TARGET` | `src/config.h` | **The flag**: `_BROWSER` or `_ESP32C3` |
| `DEBUG_CAMERA` | `src/config.h` | **Debug flag**: 0=off, 1=boot+self-test, 2=+per-frame ('d' dumps live) |
| `CAMERA_XCLK_HZ` / `CAMERA_FB_LOCATION` | `src/config.h` | Camera clock + frame-buffer location (PSRAM/DRAM) |
| `QUALITY_DEFAULT` | `src/config.h` | Boot quality preset |
| `WIFI_SSID` / `WIFI_PASS` / `WIFI_CHANNEL` | both configs | AP credentials + channel (must match) |
| `ESPNOW_PEER_MAC` | `src/config.h` | C3 receiver MAC |
| `RECEIVER_MAX_FRAME_BYTES` | both configs | Frame cap (receiver RAM limit) |
| `BROWSER_MAX_FRAME_BYTES` | `src/config.h` | Browser frame cap (auto-downscale above this) |
| `ADAPTIVE_*` | `src/config.h` | Auto quality-step thresholds |
| `ESPNOW_SEND_RETRIES` | `src/config.h` | Per-segment retransmit budget |
| `FRAME_ASSEMBLE_TIMEOUT_MS` | `receiver/src/receiver_config.h` | Stale-frame discard time |
| `USB_MAGIC_HI/LO`, `USB_HEADER_BYTES`, `USB_BAUD` | `receiver/src/receiver_config.h` **+** `host/c3_usb_mjpeg.py` | USB CDC framing (keep in sync) |

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
- In ESP-NOW mode the C3's USB CDC (`Serial`) is **dual-mode**: it boots as a
  readable console (banner, `[rx]` logs, `m`/`s` — so `pio device monitor`
  works out of the box) and the host viewer switches it to the binary frame
  stream by sending `g` (`x`, or the host closing the port, returns it to the
  console). In stream mode all text output is mirrored to the hardware UART
  (`Serial0`, GPIO21 TX / GPIO20 RX) only, which is what keeps log text from
  corrupting the framed JPEG bytes that `host/c3_usb_mjpeg.py` expects.
- The host viewer's HTTP server is multithreaded (`ThreadingHTTPServer`), so it
  can serve several phones/PCs at once; the C3 itself is never a TCP server.

---

## 7) Debugging a bad camera (`DEBUG_CAMERA` flag)

If the camera looks dead (black/no stream, `[cam] init FAILED`, or endless
`capture returned no frame`), turn on the debug flag in `src/config.h`:

```cpp
#define DEBUG_CAMERA  1   // 0 = quiet, 1 = boot + camera self-test, 2 = also per-frame
```

Flash, then open the serial monitor (`pio device monitor -e esp32s3_sense_sender`).
You get a full **boot + camera diagnostic dump**:

- **Reset reason** — a `BROWNOUT` or `PANIC` here often masquerades as a camera fault.
- **Chip / flash / heap / PSRAM** — `camera_config_t.fb_location` needs PSRAM; the
  dump warns loudly if PSRAM is not found.
- **GPIO → camera pin map** — spot a loose/miswired ribbon or wrong board variant.
- **`esp_camera_init()` result**, decoded:
  - `0x20001 ESP_ERR_CAMERA_NOT_DETECTED` → **the sensor does not answer on SCCB**.
    This is the classic bad-module / bad-ribbon signature: **swap or reseat the
    camera module** (swapping just the camera fixed exactly this on our board).
  - `0x20002` / `0x20003` → sensor answered but rejected the mode.
- **Sensor SCCB identity** — `PID`, `MID`, `VER`, I2C address and part name
  (e.g. OV2640 = `0x26`, OV3660 = `0x3660`). A PID of `0x00`/`0xFF` means nothing
  is on the bus → hardware fault, not firmware.
- **Live capture probe** — one frame: resolution, byte length, pixel format and
  whether it starts with the JPEG SOI marker `FF D8`.

The same dump can be requested any time by typing **`d`** in the serial monitor
(works in both targets, even mid-stream). `s` now also reports the running count
of failed captures. With `DEBUG_CAMERA 2` every captured frame is logged, and a
non-JPEG frame logs a `WARNING` at level 1 too.

### "SCCB ID reads fine but it never frames" — the `t` bus sweep

If the module initialises and its SCCB identity is correct (right PID / part name)
but the capture probe still fails, the **data bus** is the suspect, not the
control bus. Type **`t`**: it re-inits the sensor across XCLK = 20/16/10/24 MHz
and both frame-buffer locations (PSRAM/DRAM), tries a capture in each, and prints
every mode that produced a real JPEG:

- **Some mode works** → set `CAMERA_XCLK_HZ` in `config.h` to that value
  (`16000000` also enables EDMA mode on the S3).
- **0/modes work** → the module's data path (sensor / ribbon / flex contacts) is
  dead. Clean + reseat the ribbon contacts and retry; if it still fails the
  module is not repairable in firmware — replace it (swapping just the camera is
  the fix).

The sweep leaves the camera re-initialised at `QUALITY_DEFAULT`, so streaming
continues afterwards. Two details:

- The sweep only accepts a frame as **real** if it is **≥ 512 bytes** *and* starts
  with `FF D8`; a smaller buffer is a stale/partial DMA buffer, not a picture
  (this filter prevents a false "WORKS!").
- The normal frame-buffer location is `CAMERA_FB_LOCATION` in `config.h`
  (`CAMERA_FB_IN_PSRAM` by default). Set it to `CAMERA_FB_IN_DRAM` to test a
  module that frames only on the internal-RAM path.

`DEBUG_CAMERA 0` compiles every diagnostic out (no size/speed cost, no output) —
use that for a "production" build once the hardware is known good.
