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

#define SERVER_PORT           80
#define WIFI_CHANNEL          1          // must match the sender

// --- ESP-NOW framing (MUST match sender's config.h) --------------------------
#define ESPNOW_MAX_PAYLOAD   250U
#define NOW_HEADER_BYTES     10U
#define NOW_PAYLOAD_BYTES    (ESPNOW_MAX_PAYLOAD - NOW_HEADER_BYTES)

// Incoming frame buffer cap. The C3 has NO PSRAM, so keep this modest.
#define RECEIVER_MAX_FRAME_BYTES  (32UL * 1024)

// Max time we keep assembling a frame before giving up (auto-skip stale).
#define FRAME_ASSEMBLE_TIMEOUT_MS  200UL

// Optional NACK back-channel: tell the sender which frame we failed to complete
// so it can downscale faster. Requires ESPNOW_USE_NACK=1 on the sender too.
#define ENABLE_ESPNOW_NACK  0

// Sender MAC (only needed for NACK replies)
#define SENDER_MAC  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }