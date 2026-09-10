// receiver.cpp - ESP32-C3 "dish" side: reassembles segmented ESP-NOW frames
// from the S3 Sense and re-serves them as an MJPEG stream.
//
// Usage:
//   1. Wire the C3's antenna to the dish / high-gain element for the link.
//   2. Print this board's MAC (serial 'm') after it connects.
//   3. Put that MAC into the sender (config.h -> ESPNOW_PEER_MAC).
//   4. Connect a phone/PC to the S3 AP ("XIAO-Sense") and open  http://192.168.4.2/
//
// Serial:  m = print MAC, s = status
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "receiver_config.h"

// ---------------------------------------------------------------------------
// Reassembly state (two buffers: assembling + last complete frame).
// ---------------------------------------------------------------------------
static uint8_t  assembleBuf[RECEIVER_MAX_FRAME_BYTES];
static uint8_t  latestBuf[RECEIVER_MAX_FRAME_BYTES];
static uint32_t latestLen = 0;
static uint16_t latestFrameId = 0;

#define MAX_SEGMENTS  ((RECEIVER_MAX_FRAME_BYTES + NOW_PAYLOAD_BYTES - 1) / NOW_PAYLOAD_BYTES + 1)
static uint8_t  segBitmap[(MAX_SEGMENTS + 7) / 8];

static uint16_t asmFrameId = 0xFFFF;
static uint16_t asmTotal   = 0;
static uint16_t asmTailPlen = 0;    // payload len of the highest segment seen
static uint32_t asmCount   = 0;     // distinct segments seen
static uint32_t asmLastMs  = 0;     // for stale-frame timeout

static bool segBitGet(unsigned idx) { return (segBitmap[idx >> 3] & (uint8_t)(1u << (idx & 7))) != 0; }
static void segBitSet(unsigned idx) { segBitmap[idx >> 3] |= (uint8_t)(1u << (idx & 7)); }
static void segBitClear() { for (unsigned i = 0; i < sizeof(segBitmap); ++i) segBitmap[i] = 0; }

void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < NOW_HEADER_BYTES) return;

  uint16_t magic   = (uint16_t)((data[0] << 8) | data[1]);
  uint16_t frameId = (uint16_t)((data[2] << 8) | data[3]);
  uint16_t seg     = (uint16_t)((data[4] << 8) | data[5]);
  uint16_t total   = (uint16_t)((data[6] << 8) | data[7]);
  uint16_t plen    = (uint16_t)((data[8] << 8) | data[9]);

  if (magic != 0xA3C3 || total == 0 || seg >= total) return;
  size_t payloadLen = (size_t)(len - NOW_HEADER_BYTES);

  // New/stale frame id -> drop old partial state and start fresh (skip-ahead).
  if (frameId != asmFrameId) {
    asmFrameId = frameId;
    asmTotal   = total;
    asmTailPlen = 0;
    asmCount   = 0;
    segBitClear();
  }

  // Bounds check (never write past available RAM).
  size_t off = (size_t)seg * NOW_PAYLOAD_BYTES;
  if (off + payloadLen > RECEIVER_MAX_FRAME_BYTES) return;

  if (!segBitGet(seg)) {
    memcpy(assembleBuf + off, data + NOW_HEADER_BYTES, payloadLen);
    segBitSet(seg);
    asmCount++;
  }
  if (seg + 1 == total) asmTailPlen = plen;   // remember the short tail
  asmLastMs = millis();

  // Frame complete when the last segment arrives with all segments present.
  if (seg + 1 == total && asmCount >= total) {
    // Real byte count = all-but-last full segments + short tail.
    uint32_t realLen = (uint32_t)(total - 1) * NOW_PAYLOAD_BYTES + asmTailPlen;
    memcpy(latestBuf, assembleBuf, realLen);
    latestLen = realLen;
    latestFrameId = frameId;
    asmFrameId = 0xFFFF;   // next new frame restarts cleanly
    Serial.printf("[rx] frame %u ok (%u B, %u seg)\n", frameId, realLen, total);
  }
}

// Drop a frame that has been assembling too long (stale / partial).
void checkAssembleTimeout() {
  if (asmFrameId != 0xFFFF && (millis() - asmLastMs) > FRAME_ASSEMBLE_TIMEOUT_MS) {
    if (ENABLE_ESPNOW_NACK) {
      // (optional) tell the sender which frame died so it can downscale faster.
    }
    asmFrameId = 0xFFFF;
  }
}

// ---------------------------------------------------------------------------
// Minimal MJPEG server so a phone/PC on the S3 AP can view the dish stream.
// ---------------------------------------------------------------------------
static WiFiServer srv;
static const char INDEX_HTML[] =
  "<!DOCTYPE html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>Dish RX</title></head><body style='background:#000;color:#eee'>"
  "<h2>ESP32-C3 dish receiver</h2>"
  "<img src='/stream' style='width:100%;max-width:640px'><br>"
  "<span id='m'>status</span><script>"
  "f=document.getElementById('m');"
  "setInterval(function(){f.textContent='receiving frames '+(new Date().toTimeString());},1000);"
  "</script></body></html>";

void setup() {
  Serial.begin(115200);
  delay(300);

  // STA on the S3 AP keeps both on the same WiFi channel -> ESP-NOW works.
  WiFi.mode(WIFI_STA);
  WiFi.config(CLIENT_IPADDR, AP_IPADDR, CLIENT_NETMASK);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(200); Serial.print('.'); }
  Serial.println();
  Serial.printf("[rx] connected to %s, ip %s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
  uint8_t d[6]; WiFi.macAddress(d);
  Serial.printf("[rx] my MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                d[0],d[1],d[2],d[3],d[4],d[5]);

  esp_now_init();
  esp_now_register_recv_cb(&onRecv);

  srv = WiFiServer(SERVER_PORT);
  srv.begin();
  Serial.printf("[rx] MJPEG at http://%s/stream\n", WiFi.localIP().toString().c_str());
}

void handleHttp() {
  WiFiClient c = srv.available();
  if (!c) return;
  String line = c.readStringUntil('\n');
  line.trim();

  if (line.startsWith("GET /stream")) {
    c.setNoDelay(true);
    c.write("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; "
            "boundary=frame\r\nCache-Control: no-cache\r\n\r\n");
    while (c.connected()) {
      if (latestLen) {
        char hdr[64];
        int n = snprintf(hdr, sizeof(hdr),
              "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
              (unsigned)latestLen);
        c.write((const uint8_t*)hdr, n);
        c.write(latestBuf, latestLen);
        c.write((uint8_t*)"\r\n", 1);
      } else {
        delay(5);
      }
      delay(40);  // throttle display updates a little
    }
    c.stop();
  } else {
    c.write("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    c.write((const uint8_t*)INDEX_HTML, sizeof(INDEX_HTML) - 1);
    c.stop();
  }
}

void loop() {
  checkAssembleTimeout();
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'm') {
      uint8_t d[6]; WiFi.macAddress(d);
      Serial.printf("MAC %02X:%02X:%02X:%02X:%02X:%02X IP %s\n",
                    d[0],d[1],d[2],d[3],d[4],d[5], WiFi.localIP().toString().c_str());
    } else if (ch == 's') {
      Serial.printf("latest frame #%u len %u\n", latestFrameId, (unsigned)latestLen);
    }
  }
  handleHttp();
}