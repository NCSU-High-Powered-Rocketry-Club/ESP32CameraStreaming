// stream_espnow.cpp - segmented ESP-NOW streaming to the ESP32-C3 dish receiver.
//
// Reliability model:
//   * ESP-NOW requests a link-layer ACK for every datagram we send.
//   * Segments that time out are retransmitted up to ESPNOW_SEND_RETRIES times.
//   * A frame is discarded wholesale if it can't be delivered in reasonable
//     time (we never block the camera backlog - stale frames are dropped).
//   * A sliding-window of per-segment failures drives automatic quality stepping
//     (see adaptiveStep) so the link self-tunes to the worst conditions.
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>       // arduino-esp32 core 3.x: underscore name
#include "config.h"
#include "camera_hal.h"
#include "stream_espnow.h"

// --- ESP-NOW segment header (big-endian for portability) --------------------
typedef struct {
  uint16_t magic;    // NOW_MAGIC - sanity check
  uint16_t frameId;  // rolling frame ID
  uint16_t seg;      // 0-based segment index
  uint16_t total;    // total segment count for this frame
  uint16_t plen;     // bytes of payload in this segment (<= NOW_PAYLOAD_BYTES)
} nowSegHdr;

static const uint16_t NOW_MAGIC = 0xA3C3;
static QualityLevel current    = QUALITY_DEFAULT;
static uint16_t     frameCounter = 0;
static uint8_t      peerMac[6];      // receiver's MAC (copied from config)
static bool         peerKnown = false;

// --- send status ------------------------------------------------------------
static volatile bool sendDone = false;
static volatile bool sendOk   = false;

void espNowOnSend(const uint8_t* mac, esp_now_send_status_t status) {
  sendOk  = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// Send one datagram with per-segment retries + a bounded wait for the ACK.
// NOTE: esp_now_send() returns esp_err_t, ESP_OK == 0 on success.
static bool espNowSendReliable(const uint8_t* buffer, size_t len) {
  for (int attempt = 0; attempt < ESPNOW_SEND_RETRIES; ++attempt) {
    sendDone = false;
    if (esp_now_send(peerMac, buffer, len) != ESP_OK) { delay(ESPNOW_RETRY_DELAY_MS); continue; }
    uint32_t t0 = millis();
    while (!sendDone) {
      if ((millis() - t0) > 30) break;      // give the ACK ample time
      delay(0);
    }
    if (sendDone && sendOk) return true;
    delay(ESPNOW_RETRY_DELAY_MS);
  }
  return false;
}

// Fill a 250-byte datagram with one segment.
static uint16_t buildSegment(uint8_t* out, uint16_t frameId, uint16_t seg,
                             uint16_t total, const uint8_t* frame, uint32_t frameLen) {
  nowSegHdr h;
  h.magic   = NOW_MAGIC;
  h.frameId = frameId;
  h.seg     = seg;
  h.total   = total;
  uint32_t off  = (uint32_t)seg * NOW_PAYLOAD_BYTES;
  uint32_t left = frameLen - off;
  uint16_t plen = left > NOW_PAYLOAD_BYTES ? NOW_PAYLOAD_BYTES : (uint16_t)left;
  h.plen    = plen;

  out[0] = (uint8_t)(h.magic   >> 8); out[1] = (uint8_t)(h.magic);
  out[2] = (uint8_t)(h.frameId >> 8); out[3] = (uint8_t)(h.frameId);
  out[4] = (uint8_t)(h.seg     >> 8); out[5] = (uint8_t)(h.seg);
  out[6] = (uint8_t)(h.total   >> 8); out[7] = (uint8_t)(h.total);
  out[8] = (uint8_t)(h.plen    >> 8); out[9] = (uint8_t)(h.plen);
  memcpy(out + NOW_HEADER_BYTES, frame + off, plen);
  return NOW_HEADER_BYTES + plen;
}

// ---------------------------------------------------------------------------
void espnowBegin() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IPADDR, AP_IPADDR, AP_NETMASK);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  delay(300);

  uint8_t dev[6];
  WiFi.macAddress(dev);
  Serial.printf("[espnow] S3 mac %02X:%02X:%02X:%02X:%02X:%02X chan=%d\n",
                dev[0],dev[1],dev[2],dev[3],dev[4],dev[5], WIFI_CHANNEL);

  uint8_t tmpMac[6] = ESPNOW_PEER_MAC;           // expand the brace macro into an array
  bool fw = (tmpMac[0] == 0xFF && tmpMac[1] == 0xFF &&
             tmpMac[2] == 0xFF && tmpMac[3] == 0xFF &&
             tmpMac[4] == 0xFF && tmpMac[5] == 0xFF);
  if (!fw) { memcpy(peerMac, tmpMac, 6); peerKnown = true; }

  esp_now_init();

  if (peerKnown) {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    for (int i = 0; i < 6; ++i) peer.peer_addr[i] = peerMac[i];
    peer.channel = (uint8_t)WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_AP;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.print("[espnow] sending to ");
    for (int i = 0; i < 6; ++i) Serial.printf("%02X%s", peerMac[i], i < 5 ? ":" : "\n");
  } else {
    Serial.println("[espnow] peer MAC unset - configure ESPNOW_PEER_MAC in config.h");
  }
  esp_now_register_send_cb(&espNowOnSend);
}

static void setQuality(QualityLevel l) {
  if (l < QUALITY_LOW) l = QUALITY_LOW;
  if (l > QUALITY_HIGH) l = QUALITY_HIGH;
  current = l;
  cameraApplyQuality(l);
}

void espnowHandleSerialQ() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'q') { delay(5); if (Serial.available()) { char d = Serial.read(); int lv = d - '0';
      if (lv >= QUALITY_LOW && lv <= QUALITY_HIGH) setQuality((QualityLevel)lv); } }
  }
}

// Frame-level FPS cap.
static bool fpsGate(float maxFps) {
  static uint32_t lastMs = 0;
  uint32_t period = maxFps > 0 ? (uint32_t)(1000.0f / maxFps) : 0;
  if (!period) return true;
  uint32_t now = millis();
  if (now - lastMs < period) return false;
  lastMs = now;
  return true;
}

// ---------------------------------------------------------------------------
// Send one captured frame, fragmented into segments. Returns true if every
// segment made it out (the C3 may still have dropped datagrams - the receiver
// handles that by discarding incomplete frames).
// ---------------------------------------------------------------------------
static bool sendFrame(const uint8_t* frame, uint32_t len) {
  uint32_t total = (len + NOW_PAYLOAD_BYTES - 1) / NOW_PAYLOAD_BYTES;
  if (total > 0xFFFF) { Serial.println("[espnow] frame too large"); return false; }
  uint16_t fId = ++frameCounter;
  bool allOk = true;
  for (uint32_t s = 0; s < total; ++s) {
    uint8_t pkt[ESPNOW_MAX_PAYLOAD];
    uint16_t plen = buildSegment(pkt, fId, (uint16_t)s, (uint16_t)total, frame, len);
    bool ok = espNowSendReliable(pkt, plen);
    adaptiveReport(ok);
    if (!ok) allOk = false;
  }
  return allOk;
}

void espnowPoll() {
  espnowHandleSerialQ();
  if (!peerKnown) {
    static uint32_t warn = 0;
    if (millis() - warn > 5000) { Serial.println("[espnow] peer not configured yet"); warn = millis(); }
    delay(200); return;
  }
  QualityProfile p = qualityProfileFor(current);
  if (!fpsGate(p.max_fps)) { delay(2); return; }

  camera_fb_t* fb = cameraCapture();
  if (!fb) { delay(5); return; }

  // Auto-downscale any frame that the receiver's RAM cannot hold.
  if (fb->len > RECEIVER_MAX_FRAME_BYTES) {
    cameraRelease(fb);
    if (current > QUALITY_LOW) {
      Serial.printf("[espnow] frame too large - downscaling\n");
      setQuality((QualityLevel)(current - 1));
    }
    return;
  }

  bool delivered = sendFrame(fb->buf, fb->len);
  cameraRelease(fb);

  if (!delivered && current > QUALITY_LOW) {
    setQuality((QualityLevel)(current - 1));   // lighten up and drop this frame
    return;
  }

  int next = adaptiveStep(current);
  if (next >= 0) setQuality((QualityLevel)next);
}