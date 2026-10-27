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
│   ├── recorder.cpp/.h     #   microSD MJPEG (.avi) recorder (ESP-NOW target only)
│   ├── stream_browser.cpp/.h   # HTTP MJPEG target (phone/browser)
│   └── stream_espnow.cpp/.h    # segmented ESP-NOW target (C3)
├── receiver/               # ======== "dish" side ========  (separate nested project)
│   ├── platformio.ini      #   RECEIVER env:  esp32c3_receiver
│   └── src/                #   board seeed_xiao_esp32c3
│       ├── receiver.cpp            # reassemble + forward over USB CDC
│       └── receiver_config.h
└── host/                   # ======== host viewer ========  (stdlib-only Python)
    └── c3_usb_mjpeg.py     #   opens the C3 USB CDC port, CRC-validates,
                            #   re-serves MJPEG at http://<host>:8080/ and has
                            #   the dashboard controls (quality, mic, SD recording)
    └── test_reader_recovery.py  # regression test: reader must survive unplug /
                            #   port rename / dead fd (pty fake, no hardware)
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

The same page also drives the **sender's microSD recorder** (`r` / `e` / `t<secs>`
/ `p<0..2>`, relayed to the S3 over ESP-NOW) and shows the recording state the S3
reports back — see **section 4**.

**Live link signal (SIGNAL card).** The C3 prints `[rx] link rssi=<dBm> ch=<n>
sta=<0|1>` about once a second — `WiFi.RSSI()` of the S3's AP, which is the same
S3 and the same channel the ESP-NOW video crosses — and the host parses it into a
SIGNAL card (green > −60 dBm, amber > −75, red below; `–` = no report). It is
flushed **between frames only**, so it never corrupts the JPEG stream. This is
what the radio actually exposes: per-datagram RSSI is not available on this
Arduino core (its `esp_now_recv_cb_t` has no `rx_ctrl`), and receiver **gain**
(AGC/LNA) is not software-readable at all. TX power is (`WiFi.getTxPower()` /
`setTxPower()`), and the S3's mic gain is compile-time ×8. Tune the report rate
with `LINK_REPORT_PERIOD_MS` / `LINK_REPORT_MIN_DELTA` in
`receiver/src/receiver_config.h` — the C3 firmware must be re-flashed for the
card to show anything.

> Tune the ESP‑NOW link with the dish: point the high‑gain element at the S3's
> antenna. Range, orientation and the 2.4 GHz environment dominate throughput.

#### Troubleshooting the USB link

| Symptom | Cause / fix |
|---------|-------------|
| `pio device monitor` shows **nothing at all** | TinyUSB drops all bytes until the host asserts DTR. Open the monitor first, **then press the C3's reset button** — the firmware re-greets whenever a monitor attaches, so you can also just re-open the monitor. |
| Monitor works, but the host script shows 0 frames | Only one program can hold the port. Close the monitor, then run `python3 c3_usb_mjpeg.py` (it asserts DTR/RTS and sends `g` itself). Check the page's status line (`frames=/skips=`) — nonzero `skips` means data was arriving before the parser synced. |
| Several boards share one machine (`/dev/ttyACM0`, `ACM1`, ...) | The script *probes* each port and reports `C3 identified on <port>`; pass `--port /dev/ttyACMx` to pin it. "no answer to the C3 probe" means that board is not the C3 firmware (or the C3 is still joining Wi-Fi and only printing `.` dots). |
| Stream dies after an unplug / C3 reboot / cable glitch and never returns | **Fixed (2026-10-16).** The reader thread used to die on the first failed reopen (`os.read(None)` → `TypeError` inside `finally: os.close(None)`), so only restarting the script brought the stream back. It now reconnects forever: reopen on read error, re-detect a renamed node (`ttyACM0` → `ACM1`), cycle an open-but-silent fd every 20 s (that is the only thing that clears a wedged host-side CDC endpoint), and name the PID holding a locked port. The header shows `⟳ reconnecting…` with the reconnect count until frames return. Regression test, no hardware needed: `python3 host/test_reader_recovery.py`. |
| **Picture freezes and stays frozen even though the C3 is fine** (console pane keeps printing `[rx] frame N ok …`, `/status` `frames` stuck) | **Fixed (2026-10-16).** The C3 drops to CONSOLE mode on any stray `x` — a second host instance, a `pio device monitor` exit, its own USB glitch — and needs a new `g` to stream again; the host's re-arm was gated on the *cumulative* frame counter, so after the first frame it never sent `g` again. Now the re-arm fires every ~2 s whenever no frame is being decoded (suppressed while PAUSEd), and the page re-opens `/stream` if `seq` stalls for 4 s. Verify with `python3 host/test_reader_recovery.py` (test [9]). |
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

## 4) Recording to microSD (ESP-NOW / dish target only)

The S3 can record the **same** JPEG frames it streams to the dish onto the Sense
board's microSD slot, as a real `.avi` (Motion-JPEG) clip — so the footage
survives a link failure and can be pulled off the card afterwards. One capture
feeds both consumers, so recording costs no extra camera time.

> **ESP-NOW build only.** Set `STREAM_TARGET` to `STREAM_TARGET_ESP32C3` in
> `src/config.h` and leave `ENABLE_SD_RECORDING` at 1. With the phone/browser
> target the recorder is compiled out entirely (`config.h` forces the flag to 0):
> no SD mount, no recorder code, no controls.

Hardware: the Sense expansion board's slot — SPI **SCK=GPIO7, MISO=GPIO8,
MOSI=GPIO9** (arduino-esp32 defaults) + **CS=GPIO21**, card formatted **FAT32**.
Mount is retried at 4 MHz when the high speed fails; a missing card only disables
recording, the air link keeps running.

### Dashboard controls (host page)

`host/c3_usb_mjpeg.py` serves an **SD recording** card next to the quality/mic
controls:

| Control | Command sent | Path |
|---------|--------------|------|
| ⏺ START REC / ⏹ END REC | `r` / `e` | dashboard → C3 (USB) → ESP-NOW → S3 |
| END AFTER (seconds) + SET | `t<secs>` (`t0` = record until stopped) | same |
| SD / BALANCED / STREAM | `p0` / `p1` / `p2` | same |

The card shows the sender's **real** state — clip number, frames written,
seconds left, priority, card present — because the S3 reports it back over
ESP-NOW and the C3 prints it as an `[rx] REC state=…` line, which the page
parses. "no report yet - is the S3 running the ESP-NOW build?" means the sender
is on the browser build (or off).

### Priority between the SD card and the stream

Both consumers want the same frame, and one frame slot is `1000 / max_fps` ms.
`RECORD_PRIORITY_*` in `src/config.h` decides whose timing is protected:

| Priority | When one slot cannot afford both |
|----------|----------------------------------|
| `SD` (0) | Write to the card first; if that eats the whole slot the frame is **not** streamed — the clip keeps its frame rate, the stream degrades |
| `BALANCED` (1, default) | Always do both (card first, then the air); the loop rate absorbs the sum of the two costs |
| `STREAM` (2) | Send over the air first; if the send owns the slot the frame is **not** written — the stream keeps its rate, the clip's real fps drops |

A frame that is too big for the C3 (`RECEIVER_MAX_FRAME_BYTES`) is still written
to the card under SD/balanced priority (it is a valid picture) and only skipped
on the air — that path also triggers the usual auto-downscale.

### Clips on the card

`/video00001.avi`, `/video00002.avi`, … using the **first free index**, so clips
from an earlier power cycle are never overwritten. Each file is a genuine AVI
container (RIFF + `hdrl` + `movi` + `idx1` index), so seeking and playback work
out of the box:

```bash
ffprobe /media/card/video00001.avi
ffplay  /media/card/video00001.avi
```

Every frame is **trimmed at its JPEG EOI marker** (the OV3660 can return
concatenated JPEGs or padding in one buffer), so each AVI frame *and* each
streamed frame is a single self-contained image.

### Serial control on the S3 console

Handy when bench-testing the sender without the dashboard:

```
r               start recording          e               stop recording
D<seconds>      max clip length (D0 = record until stopped)
P<0..2>         priority (0 = SD first, 1 = balanced, 2 = stream first)
s               status, incl. "REC state=... frames=... left=..."
```

Each clip also prints a report when it ends:

```
[rec] === saved /video00001.avi : 148 frames, 4720 KB, 10012 ms (14.8 fps) ===
[rec]   frame size : avg 32.1 KB, min 28.9 KB, max 35.5 KB
[rec]   SD cost    : 23.4 ms/frame, 471 KB/s written
```

`RECORD_MAX_SECONDS` (default 30 s) sets the clip length used until the dashboard
changes it; `0` = record until STOP. A clip with a length also ends **by
itself** once that many seconds have elapsed.

---

## 5) Optimization & error tolerance

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

## 6) Config cheat-sheet

| Symbol | File | Meaning |
|--------|------|---------|
| `STREAM_TARGET` | `src/config.h` | **The flag**: `_BROWSER` or `_ESP32C3` |
| `ENABLE_SD_RECORDING` | `src/config.h` | **microSD recorder** switch (forced 0 unless the target is ESP-NOW) |
| `RECORD_PRIORITY_DEFAULT` | `src/config.h` | SD vs. stream priority (`SD` / `BALANCED` / `STREAM`) |
| `RECORD_MAX_SECONDS` | `src/config.h` | Default clip length in seconds (0 = until stopped) |
| `SD_CS_PIN` / `SD_SPI_FREQUENCY` | `src/config.h` | microSD slot wiring (CS=21; SCK/MISO/MOSI 7/8/9) |
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
| `REC_STATUS_TAG`, `REC_CTRL_TAG_R/T/P` | `src/config.h` **+** `receiver/src/receiver_config.h` | Recording control/status datagrams (keep in sync) |
| `[rx] REC state=…` line format | `receiver/src/receiver.cpp` **+** `host/c3_usb_mjpeg.py` (`REC_LINE_RE`) | Recording status the dashboard parses (keep in sync) |
| `[rx] link rssi=… ch=… sta=…` line format | `receiver/src/receiver.cpp` (`reportLink`) **+** `host/c3_usb_mjpeg.py` (`LINK_LINE_RE`) | Live link level for the SIGNAL card (keep in sync) |
| `LINK_REPORT_PERIOD_MS` / `LINK_REPORT_MIN_DELTA` | `receiver/src/receiver_config.h` | How often / how eagerly the C3 reports RSSI |

## 7) Notes & limitations

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
- The microSD recorder exists in the **ESP-NOW build only** (section 4) and is
  fed from the same capture as the stream, so it costs no extra camera time;
  `RECORD_PRIORITY_*` decides whose timing is protected when one frame slot
  cannot afford both. Clips are written on the **sender** (the C3 has no card
  slot and no PSRAM), and a clip is always a valid AVI even if it is cut short.
- The recording state shown on the page is reported by the sender over ESP-NOW
  (`[rx] REC …` lines the C3 prints). If the page says "no report yet", the
  sender is still on the browser build (`STREAM_TARGET_BROWSER`) or switched off.

---

## 8) Debugging a bad camera (`DEBUG_CAMERA` flag)

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

Measured on this board's OV3660 with the sweep (2026-10-13):

| Mode | Result |
|------|--------|
| XCLK 20 MHz, PSRAM buffers | frames a real JPEG |
| XCLK 20 MHz, DRAM buffers  | frames a real JPEG |
| XCLK 10 MHz, PSRAM buffers | frames a real JPEG |
| XCLK 10 MHz, DRAM buffers  | frames a real JPEG |
| XCLK 16 MHz, both          | init OK, **zero frames** |

So "SCCB ID reads fine but it never frames" is not automatically a dead module:
if the sweep shows *some* clock working, it is a settings problem - put a working
value in `CAMERA_XCLK_HZ`. 16 MHz is the ESP32-S3 PSRAM-DMA path and breaks JPEG
capture on this hardware; keep the clock at 20 MHz (or 10 MHz). A camera that
fails every capture while the sweep works at 20 MHz is a `config.h` problem, not
a hardware one - which is exactly how a wrong XCLK masquerades as a dead module.

`DEBUG_CAMERA 0` compiles every diagnostic out (no size/speed cost, no output) —
use that for a "production" build once the hardware is known good.
