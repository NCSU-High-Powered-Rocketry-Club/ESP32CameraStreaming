// config.h - master configuration for the ESP32-S3 Sense camera sender.
//
// THIS FILE CONTAINS THE "FLAG" THAT SELECTS WHO RECEIVES THE STREAM:
//
//     #define STREAM_TARGET  STREAM_TARGET_BROWSER   -> phone / laptop web browser
//     #define STREAM_TARGET  STREAM_TARGET_ESP32C3   -> ESP32-C3 "dish" receiver
//
// Everything else in this file is tunable (WiFi, quality, ESP-NOW framing...).
#pragma once

// ---------------------------------------------------------------------------
// 1) STREAM TARGET (the flag you asked for)
// ---------------------------------------------------------------------------
#define STREAM_TARGET_BROWSER  1   // HTTP MJPEG server - open in any phone/PC browser
#define STREAM_TARGET_ESP32C3  2   // Segmented ESP-NOW frames to the dish ESP32-C3

// <-- CHANGE THIS ONE LINE TO PICK WHO RECEIVES THE VIDEO -->
#define STREAM_TARGET  STREAM_TARGET_ESP32C3

#if (STREAM_TARGET != STREAM_TARGET_BROWSER) && (STREAM_TARGET != STREAM_TARGET_ESP32C3)
#error "STREAM_TARGET must be STREAM_TARGET_BROWSER or STREAM_TARGET_ESP32C3"
#endif

// ---------------------------------------------------------------------------
// 2) Camera hardware (Seeed XIAO ESP32-S3 Sense, OV3660/OV2640-compatible)
// ---------------------------------------------------------------------------
#define CAMERA_MODEL_XIAO_ESP32S3   // selects the pin map in camera_pins.h

// Default quality on boot. Can be changed at runtime from the serial monitor:
//   q0 = LOW    (QVGA 320x240 - tiny frames, high frame rate)
//   q1 = MEDIUM (VGA 640x480 - balanced)                <-- default
//   q2 = HIGH   (UXGA 1600x1200 - detailed, lower frame rate)
#define QUALITY_DEFAULT  QUALITY_LOW

// Camera XCLK (master clock) in Hz. 20 MHz is the Seeed default. If a module
// initialises (SCCB ID reads correctly) but NEVER returns a frame, the parallel
// data bus may be marginal - trying 16 MHz (also enables EDMA mode on the S3)
// or 10 MHz sometimes revives a flaky module. The serial 't' command sweeps
// these automatically (see DEBUG_CAMERA).
//
// MEASURED on this board's OV3660 with the 't' sweep (2026-10-13): 20 MHz
// PSRAM/DRAM and 10 MHz PSRAM/DRAM all frame a real JPEG, 16 MHz does NOT
// (init OK, zero bytes captured) - 16 MHz switches on the experimental
// PSRAM-DMA path that breaks JPEG capture on the ESP32-S3. Keep this at
// 20000000 unless a sweep on YOUR module says otherwise.
#define CAMERA_XCLK_HZ   20000000

// Frame-buffer location. CAMERA_FB_IN_PSRAM (default) uses the 8 MB PSRAM
// (EDMA path). If a module initialises and its SCCB ID reads fine but it only
// delivers frames when the buffer lives in internal RAM, switch this to
// CAMERA_FB_IN_DRAM - some marginal sensors/boards fail on the PSRAM path only.
// NOTE: internal RAM is limited; keep QUALITY_DEFAULT at LOW/MED if you use DRAM.
#define CAMERA_FB_LOCATION  CAMERA_FB_IN_PSRAM

// ---------------------------------------------------------------------------
// 3) WiFi access point (this board acts as an AP for both targets)
//    The S3 Sense creates its own AP so the browser / the C3 can always connect
//    with a known address, even in the field with no internet.
// ---------------------------------------------------------------------------
#define WIFI_SSID         "XIAO-Sense"
#define WIFI_PASS         "changeme1234"   // >= 8 chars
#define WIFI_CHANNEL      1                // must match the C3's channel (ESP-NOW)
#define SERVER_PORT       80

// ------ AP static addressing (harmless, makes addresses predictable) ------
#define AP_IPADDR     IPAddress(192, 168, 4, 1)
#define AP_NETMASK    IPAddress(255, 255, 255, 0)

// ---------------------------------------------------------------------------
// 4) Quality-of-service knobs (applied to BOTH targets)
// ---------------------------------------------------------------------------
#define STREAMING_GRAB_MODE   CAMERA_GRAB_LATEST  // always grab the newest frame

// Browser target: longest time we keep trying to write to a dead client before
// dropping it (error tolerance).
#define BCAST_WRITE_TIMEOUT_MS  2000

// Browser target: largest single JPEG frame we will put on the wire. frames
// that come out of the camera bigger than this would clog a slow client's link
// and stall the server, so the streamer auto-downscales instead. Generous to
// keep genuine HIGH quality working, but bounded so one huge frame can't wedge
// the MJPEG path.
#define BROWSER_MAX_FRAME_BYTES  (256UL * 1024)

// ESP-NOW target: receiver buffer cap (see receiver/receiver_config.h).
// The C3 has NO PSRAM, so frames must be small - keep this conservative.
// If a captured frame is bigger than this the sender auto-downscales.
#define RECEIVER_MAX_FRAME_BYTES  (32UL * 1024)

// ---------------------------------------------------------------------------
// 5) ESP-NOW framing (used only when STREAM_TARGET_ESP32C3)
//    ESP-NOW datagrams carry max 250 bytes, so each JPEG frame is chopped into
//    segments. Every segment carries a sequence header so the receiver can
//    reassemble and discard incomplete frames safely.
// ---------------------------------------------------------------------------
#define ESPNOW_MAX_PAYLOAD  250U          // hard protocol limit
#define NOW_HEADER_BYTES    10U           // our segment header (must equal sizeof below)
#define NOW_PAYLOAD_BYTES   (ESPNOW_MAX_PAYLOAD - NOW_HEADER_BYTES)  // 240

// Retransmission / error tolerance (per segment).
#define ESPNOW_SEND_RETRIES      2        // attempts per segment (latency: 3 x wait hurts)
#define ESPNOW_RETRY_DELAY_MS    1        // pause between retries

// MAC address of the ESP32-C3 dish receiver (fill in YOUR receiver's MAC).
// If left 0xFF it is ignored and the C3's address is shown on its serial monitor.
// See the receiver build for how to read it.
#define ESPNOW_PEER_MAC  { 0xE8, 0xF6, 0x0A, 0x17, 0x55, 0xBC }

// If you set this to 1 and enable ENABLE_ESPNOW_NACK in the receiver, the
// receiver will reply with the frame-id of incomplete frames so the sender can
// skip-ahead / downscale faster. Optional; leave 0 for a purely one-way link.
#define ESPNOW_USE_NACK   0

// Adaptive quality: when too many segments/timeouts fail we automatically step
// the quality down. When the link recovers we slowly step it back up.
#define ADAPTIVE_QUALITY_ENABLED  1
#define ADAPTIVE_DOWN_THRESHOLD   0.60f   // ratio of failures that triggers a step down
#define ADAPTIVE_UP_THRESHOLD     0.15f   // ratio below which we step back up
#define ADAPTIVE_WINDOW           64      // samples in the sliding window

// Ceiling for the adaptive up-stepper on the ESP-NOW target. HIGH (UXGA) frames
// are ~160 KB - far beyond what the C3 can receive (32 KB, no PSRAM) - and
// UXGA capture has been observed to crash the S3. MEDIUM is the practical max.
#define ESPNOW_MAX_QUALITY  QUALITY_MEDIUM

// ---------------------------------------------------------------------------
// 5b) Control / status back-channel (ESP-NOW, tiny tagged datagrams)
//     Control  (C3 -> S3, broadcast, MUST match the tags in
//               receiver/src/receiver.cpp - keep both sides in sync):
//       {'Q', level}                 quality 0..2  ('0'..'2')
//       {'M'}                        toggle the PDM mic
//       {'R', '0'|'1'}               stop / start the SD recording   <- new
//       {'T', secHi, secLo}          max recording seconds (0 = unlimited) <- new
//       {'P', '0'..'2'}              recording/streaming priority   <- new
//     Status   (S3 -> C3, sent to the peer MAC, printed by the C3 as an
//               "[rx] REC ..." line that host/c3_usb_mjpeg.py shows and parses):
//       {'S', on, sd, prio, frames(2BE), left(2BE), idx(2BE)}
//     Both are lossy fire-and-forget; a lost control datagram is re-sent by
//     pressing the button again, and a lost status line is simply not shown.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 6) Microphone (PDM mic on the XIAO S3 Sense expansion board)
//    16 kHz captured, decimated to 8 kHz, 8-bit unsigned PCM = 8 KB/s, which
//    fits in the spare ESP-NOW airtime next to the video segments.
//    NOTE: if you hear silence but no errors, swap the two pins - board
//    revisions have used both orientations.
// ---------------------------------------------------------------------------
#define ENABLE_MIC             1
#define MIC_CLK_GPIO           42         // PDM clock  (ws_io_num)
#define MIC_DATA_GPIO          41         // PDM data   (data_in_num)
#define MIC_CAPTURE_RATE       16000      // I2S PDM capture rate (Hz)
#define MIC_STREAM_RATE        8000       // after 2:1 decimation (Hz)
#define MIC_CHUNK_BYTES        240        // one ESP-NOW audio datagram = 30 ms

// ---------------------------------------------------------------------------
// 7) microSD recorder (ESP-NOW / "dish" target ONLY)
// ---------------------------------------------------------------------------
// The S3 records the very same JPEG frames it streams to the C3 onto the Sense
// board's microSD slot, as a real .avi (Motion-JPEG) clip - so the footage
// survives a link failure and can be pulled off the card afterwards. It is
// wired into the ESP-NOW streaming path ONLY: with STREAM_TARGET_BROWSER this
// section is inert (nothing mounts the card, no recorder commands exist).
//
//   0 = recorder compiled out of the loop (no mount, no commands)
#define ENABLE_SD_RECORDING    1

// ...but the recorder belongs to the ESP-NOW path ONLY: for the phone/browser
// target the flag is forced to 0 here, so that build contains no SD code, never
// mounts the card and has no recorder commands (the phone path is untouched).
#if (STREAM_TARGET != STREAM_TARGET_ESP32C3)
  #undef  ENABLE_SD_RECORDING
  #define ENABLE_SD_RECORDING  0
#endif

// microSD slot on the Sense expansion board: SPI SCK=GPIO7, MISO=GPIO8,
// MOSI=GPIO9 (the arduino-esp32 variant defaults) plus this chip-select.
// Mount is retried at 4 MHz if the high speed fails.
#define SD_CS_PIN              21
#define SD_SPI_FREQUENCY       20000000

// Clips are written to the card root as /video00001.avi, /video00002.avi, ...
#define RECORD_FILE_PREFIX     "/video"
#define RECORD_FILE_EXTENSION  ".avi"

// Nominal frame rate written into the AVI container header. The real rate is
// whatever the capture + air + SD path manages; players only use this for
// seeking. (The dashboard's frame-rate expectation is the quality preset.)
#define RECORD_AVI_FPS         15

// Default clip length, in seconds, used until the dashboard sets another one.
//   0 = record until STOP is pressed.
#define RECORD_MAX_SECONDS     30

// Priority arbitration between the SD writer and the ESP-NOW streamer for the
// one captured frame they both want (see stream_espnow.cpp for the code):
//   RECORD_PRIORITY_SD       -> card first. If the write eats the whole frame
//                               slot the frame is NOT streamed, so the clip
//                               keeps its frame rate and the stream degrades.
//   RECORD_PRIORITY_BALANCED -> both always get the frame (card first, then the
//                               air); the loop rate simply adapts to the sum of
//                               the two costs.
//   RECORD_PRIORITY_STREAM   -> air first. If the send owns the slot the frame
//                               is NOT written, so the stream keeps its rate
//                               and the clip's real fps drops instead.
#define RECORD_PRIORITY_SD        0
#define RECORD_PRIORITY_BALANCED  1
#define RECORD_PRIORITY_STREAM    2
#define RECORD_PRIORITY_DEFAULT   RECORD_PRIORITY_BALANCED

// How often the sender reports its recording state to the C3 (the dashboard
// shows it live). One tiny 10-byte datagram; 0 disables the reports.
#define RECORD_STATUS_PERIOD_MS   2000

// ---------------------------------------------------------------------------
// 8) DEBUG FLAG - bring-up / camera diagnostics
// ---------------------------------------------------------------------------
// <-- FLIP THIS when a camera (or the whole board) looks dead. It is the switch
//     that turns a silent black stream into a readable diagnosis on Serial.
//
//   0 = quiet production build (all diagnostics compiled out)
//   1 = boot dump + one-shot camera self-test:
//         reset reason, chip/flash/heap/PSRAM, the GPIO->camera pin map, the
//         esp_camera_init() result code, the sensor's SCCB identity (PID / part
//         name) and a live capture probe.
//   2 = everything in 1, PLUS a one-line summary for EVERY captured frame.
//
// Why this exists: a bad camera module almost always fails on the SCCB bus -
// esp_camera_init() returns ESP_ERR_CAMERA_NOT_DETECTED (0x20001) and/or the
// sensor PID reads 0x00 / 0xFF because nothing answers. That is a hardware
// (module / ribbon) fault, not firmware - swapping the module fixes it. This
// flag prints exactly that instead of leaving you guessing.
//
// The same dump is available on demand: type 'd' in the serial monitor.
#define DEBUG_CAMERA  1

