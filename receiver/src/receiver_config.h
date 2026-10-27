// receiver_config.h - config for the ESP32-C3 "dish" receiver side.
// Mirrors the framing constants used by the S3 sender, so keep them in sync.
#pragma once

// --- Connect to the S3 Sense access point (keeps ESP-NOW on the same channel) --
#define WIFI_SSID         "XIAO-Sense"
#define WIFI_PASS         "changeme1234"

// Static address inside the S3 AP subnet (AP is 192.168.4.1).
#define CLIENT_IPADDR     IPAddress(192, 168, 4, 2)
#define AP_IPADDR         IPAddress(192, 168, 4, 1)   // gateway
#define CLIENT_NETMASK    IPAddress(255, 255, 255, 0)

#define WIFI_CHANNEL          1          // must match the sender

// --- ESP-NOW framing (MUST match sender's config.h) --------------------------
#define ESPNOW_MAX_PAYLOAD   250U
#define NOW_HEADER_BYTES     10U
#define NOW_PAYLOAD_BYTES    (ESPNOW_MAX_PAYLOAD - NOW_HEADER_BYTES)

// Incoming frame buffer cap. The C3 has NO PSRAM, so keep this modest.
#define RECEIVER_MAX_FRAME_BYTES  (32UL * 1024)

// Air magic for audio datagrams (MUST match AUDIO_MAGIC in src/config.h).
#define AUDIO_AIR_MAGIC  0xA3A5

// Max time we keep assembling a frame before giving up (auto-skip stale).
#define FRAME_ASSEMBLE_TIMEOUT_MS  200UL

// --- Live link report (RSSI) --------------------------------------------------
// The C3 is a STA of the S3's AP and sends/receives ESP-NOW on that channel, so
// WiFi.RSSI() is the live level of the very link the video crosses. It is
// printed as "[rx] link rssi=<dBm> ch=<n> sta=<0|1>" between frames (never
// inside one) and parsed by host/c3_usb_mjpeg.py into the SIGNAL card.
// NOTE: this core's esp_now_recv_cb_t has no rx_ctrl, so per-datagram RSSI is
// not available here; the AP-level figure is what the PHY exposes.
#define LINK_REPORT_PERIOD_MS   1000UL   // heartbeat report interval
#define LINK_REPORT_MIN_DELTA   2        // report sooner when |ΔRSSI| >= this

// ---------------------------------------------------------------------------
// USB CDC output framing (wired link to the host viewer).
// MUST stay in sync with host/c3_usb_mjpeg.py (the C constants mirror the
// Python HEADER_SIZE / MAGIC_* below). This is a NEW format that only the
// host understands - it has nothing to do with the ESP-NOW air framing above.
//
// Each reassembled JPEG frame is emitted on Serial (USB CDC) as:
//
//     [0]     magic high  = USB_MAGIC_HI
//     [1]     magic low   = USB_MAGIC_LO
//     [2..5]  payload byte length (big-endian uint32)
//     [6..7]  frame id (big-endian uint16)
//     [8..11] CRC-32 (ISO-HDLC / zlib polynomial, big-endian) of the payload
//     [..]    payload  (the JPEG bytes)
//
// Length-prefixed + CRC, so the host can resync by scanning for the magic
// when a frame is dropped mid-stream (slow host or a lost ESP-NOW segment).
// ---------------------------------------------------------------------------
#define USB_MAGIC_HI     (0xC3u)
#define USB_MAGIC_LO     (0x8Cu)
#define USB_HEADER_BYTES 12U          // magic(2) + length(4) + frame id(2) + crc32(4)
#define USB_BAUD         115200UL     // nominal baud (USB-CDC ignores baud; kept for the host/monitor)

// Optional NACK back-channel: tell the sender which frame we failed to complete
// so it can downscale faster. Requires ESPNOW_USE_NACK=1 on the sender too.
#define ENABLE_ESPNOW_NACK  0

// ---------------------------------------------------------------------------
// Control / status tags (ESP-NOW, tiny tagged datagrams).
// MUST stay in sync with src/config.h in the sender project (section 5b) and
// with host/c3_usb_mjpeg.py (the 'R'/'T'/'P'/'F' dashboard buttons).
//
//   C3 -> S3 (broadcast, idempotent "set" commands, sent a few times):
//     {'Q', '0'..'2'}                 video quality
//     {'R', '0'|'1'}                  stop / start the microSD recording
//     {'T', secHi, secLo}             max recording seconds (0 = unlimited)
//     {'P', '0'..'2'}                 recording/streaming priority
//     {'F', '0'|'1'}                  fps-priority mode: mic OFF + quality
//                                     pinned LOW on the sender (idempotent)
//     {'V', fps}                      frame-rate cap 1..60 (idempotent)
//     {'A'}                           quality/rate back to AUTO (idempotent)
//     {'M'}                           toggle the sender's mic (NOT idempotent,
//                                     so this one is sent exactly once)
//   S3 -> C3 (to the peer MAC):
//     {'S', on, sd, prio, frames(2BE), left(2BE), idx(2BE)}
//   The C3 prints each status datagram as "[rx] REC state=... idx=... ...",
//   which host/c3_usb_mjpeg.py surfaces (and parses) on the dashboard.
// ---------------------------------------------------------------------------
#define REC_STATUS_TAG   'S'
#define REC_CTRL_TAG_R   'R'
#define REC_CTRL_TAG_T   'T'
#define REC_CTRL_TAG_P   'P'
// fps-priority mode ('F1' on / 'F0' off). Idempotent, so it is re-sent with
// the other "set" commands (unlike the mic toggle, which must never repeat).
#define FPS_CTRL_TAG_F   'F'
// Frame-rate cap ('V<char>') and AUTO ('A'). Both idempotent SETs.
#define FPS_CTRL_TAG_V   'V'
#define QUAL_CTRL_TAG_A  'A'

// --- Audio (PDM mic stream from the sender) ----------------------------------
// Air audio datagram: [0xA3 0xA5][seq:2BE] + PCM bytes (8 kHz unsigned 8-bit).
// Forwarded over USB with the SAME 12-byte header framing as video but with
// AUDIO magic so the host can tell the streams apart. MUST stay in sync with
// host/c3_usb_mjpeg.py (USB_AUDIO_MAGIC) and src/config.h (AUDIO_MAGIC).
#define USB_AUDIO_MAGIC_HI  (0xC3u)
#define USB_AUDIO_MAGIC_LO  (0x8Du)
#define AUDIO_FIFO_BYTES    (8UL * 1024)   // ~1 s of 8 kHz PCM; overflow = drop

// Sender MAC (only needed for NACK replies)
#define SENDER_MAC  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }