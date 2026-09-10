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
#define STREAM_TARGET  STREAM_TARGET_BROWSER

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
#define QUALITY_DEFAULT  QUALITY_MEDIUM

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
#define ESPNOW_SEND_RETRIES      3        // attempts per segment
#define ESPNOW_RETRY_DELAY_MS    2        // pause between retries

// MAC address of the ESP32-C3 dish receiver (fill in YOUR receiver's MAC).
// If left 0xFF it is ignored and the C3's address is shown on its serial monitor.
// See the receiver build for how to read it.
#define ESPNOW_PEER_MAC  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

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
