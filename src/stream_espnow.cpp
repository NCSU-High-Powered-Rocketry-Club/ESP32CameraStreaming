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
#if ENABLE_MIC
#include <driver/i2s.h>
#endif
#include "camera_hal.h"
#include "recorder.h"      // microSD AVI recorder (ENABLE_SD_RECORDING)
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

// --- TX statistics (printed periodically so a dead air link is visible) ------
static uint32_t txFrames = 0;        // frames we attempted to send
static uint32_t txSegOk  = 0;        // segments ACKed by the C3
static uint32_t txSegFail = 0;       // segments that exhausted all retries
static uint32_t txSendErr = 0;       // esp_now_send() API errors

// --- microSD recorder requests ----------------------------------------------
// Set from the ESP-NOW control channel (C3 relay) or the serial console and
// serviced by espnowPoll(): SD I/O must NOT run inside the WiFi-task receive
// callback, which is on the RF receive path.
static volatile bool recordStartRequest = false;
static volatile bool recordStopRequest  = false;

// FPS-priority mode ('F1'/'F0' from the dashboard, or 'f' on the S3's own
// console): mic OFF + quality pinned LOW + the adaptive up-stepper blocked.
// See ESPNOW_FPS_PRIORITY_MAX_FPS in config.h for why PINNING LOW - and not
// merely turning the mic off - is what actually buys frame rate.
static volatile bool fpsPriority = FPS_PRIORITY_DEFAULT;

// Explicit frame-rate cap ('V'), 0 = follow the profile rate + mic rules, and
// the "the user has chosen" flag that keeps the up-stepper from undoing it.
static volatile int  targetFps     = FPS_TARGET_DEFAULT;
static volatile bool qualityManual = false;

// ---------------------------------------------------------------------------
// Microphone capture (PDM) - capture in a background task into a ring buffer;
// the ACTUAL ESP-NOW audio sends happen here in the loop task (inside
// espnowPoll), so the sendDone/sendOk callback flags are never touched by two
// tasks at once.
// ---------------------------------------------------------------------------
#if ENABLE_MIC
static const int    MIC_RING_SIZE = 1600;   // 8 KB/s -> max ~200 ms of audio;
                                            // keeps worst-case latency bounded
static uint8_t      micRing[MIC_RING_SIZE];
static volatile int micHead = 0, micTail = 0;   // bytes buffered
static volatile bool micEnabled = true;
static uint16_t     audioSeq = 0;

static const uint16_t AUDIO_MAGIC = 0xA3A5;     // air audio datagram tag

static bool espNowSendReliable(const uint8_t* buffer, size_t len);  // below
static bool espNowSendLossy(const uint8_t* buffer, size_t len);     // below

static int micRingFree()  { return (micHead >= micTail) ? MIC_RING_SIZE - 1 - (micHead - micTail)
                                                        : micTail - micHead - 1; }
static int micRingAvail() { return (micHead >= micTail) ? micHead - micTail
                                                        : MIC_RING_SIZE - micTail + micHead; }

static void micTask(void*) {
  int16_t raw[256];
  size_t br;
  while (true) {
    if (!micEnabled) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &br, pdMS_TO_TICKS(50)) != ESP_OK || !br)
      { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    int n = br / 2;                    // 16-bit samples at MIC_CAPTURE_RATE
    for (int i = 0; i < n; i += 2) {   // 2:1 decimation -> MIC_STREAM_RATE
      // Gain x8 with clipping: the onboard PDM mic is quiet, and taking only
      // the high byte of an already-small sample is mostly quantization
      // noise (that's the "static" you hear without gain).
      int32_t s = (int32_t)raw[i] << 3;
      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;
      uint8_t u = (uint8_t)((s >> 8) + 128);          // -> unsigned 8-bit
      // NEVER wait for ring space: under a saturated link the sender loop
      // drains slower than capture, and waiting here made capture fall
      // behind - latency grew up to the whole ring. Instead drop the OLDEST
      // buffered audio (bounded latency; a lost 1/8000 s sample is nothing).
      if (micRingFree() == 0) micTail = (micTail + 1) % MIC_RING_SIZE;
      micRing[micHead] = u;
      micHead = (micHead + 1) % MIC_RING_SIZE;
    }
  }
}

static bool micBegin() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  cfg.sample_rate = MIC_CAPTURE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  i2s_pin_config_t pins = {};
  pins.bck_io_num = I2S_PIN_NO_CHANGE;
  pins.ws_io_num = MIC_CLK_GPIO;          // PDM clock
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_DATA_GPIO;       // PDM data
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) {
    Serial.println("[mic] i2s_driver_install FAILED - mic disabled");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    Serial.println("[mic] i2s_set_pin FAILED - mic disabled");
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
  if (xTaskCreate(micTask, "mic", 4096, NULL, 5, NULL) != pdPASS) {
    Serial.println("[mic] task create FAILED");
    return false;
  }
  Serial.println("[mic] PDM mic up (8 kHz, 8-bit)");
  return true;
}

// Send buffered audio as tagged datagrams. Called from the loop task only -
// returns the number of bytes sent so video pacing can account for it.
static int sendAudioChunks() {
  int sent = 0;
  while (true) {
    int avail = micRingAvail();
    if (avail < MIC_CHUNK_BYTES) break;
    uint8_t pkt[4 + MIC_CHUNK_BYTES];
    pkt[0] = (uint8_t)(AUDIO_MAGIC >> 8);
    pkt[1] = (uint8_t)(AUDIO_MAGIC & 0xFF);
    uint16_t s = ++audioSeq;
    pkt[2] = (uint8_t)(s >> 8); pkt[3] = (uint8_t)(s & 0xFF);
    for (int i = 0; i < MIC_CHUNK_BYTES; ++i) {
      pkt[4 + i] = micRing[micTail];
      micTail = (micTail + 1) % MIC_RING_SIZE;
    }
    // Lossy send: NO retries, NO ACK wait. A dropped 30 ms audio chunk is
    // inaudible; a 60 ms airtime stall per retry starves video segments
    // (this was the main cause of the ~80% drop rate with the mic on).
    if (!espNowSendLossy(pkt, sizeof(pkt))) { txSegFail++; }
    else txSegOk++;
    sent += MIC_CHUNK_BYTES;
    if (sent >= MIC_CHUNK_BYTES * 2) break;   // max 2 chunks per pass
  }
  return sent;
}
#else
static int sendAudioChunks() { return 0; }
static bool micBegin() { return true; }
#endif

// --- send status ------------------------------------------------------------
static volatile bool sendDone = false;
static volatile bool sendOk   = false;

void espNowOnSend(const uint8_t* mac, esp_now_send_status_t status) {
  sendOk  = (status == ESP_NOW_SEND_SUCCESS);
  sendDone = true;
}

// ---------------------------------------------------------------------------
// Control back-channel: the C3 can broadcast a tiny datagram asking for a
// quality change (dashboard button -> host -> C3 serial -> this callback).
// Payload: 'Q' + level ('0'..'2'). One-way video, narrow-band control.
// ---------------------------------------------------------------------------
static void setQuality(QualityLevel l);   // defined below
static void setFpsPriority(bool on);      // defined below (calls setQuality)
void espNowOnRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len >= 2 && data[0] == 'Q' && data[1] >= '0' && data[1] <= '2') {
    int lv = data[1] - '0';
    qualityManual = true;      // explicit choice: hold the auto up-stepper off
    if (lv != (int)current) {
      Serial.printf("[espnow] remote quality -> %d\n", lv);
      setQuality((QualityLevel)lv);
    }
  }
  else if (len >= 2 && data[0] == 'F' && (data[1] == '0' || data[1] == '1')) {
    // Dashboard fps button. An idempotent SET, never a toggle: ESP-NOW
    // broadcasts are not ACKed, so the C3 re-sends this and a lost datagram
    // must not desync the mode (exactly the bug the mic 'M' toggle has).
    setFpsPriority(data[1] == '1');
  }
  else if (len >= 2 && data[0] == 'V') {
    // Dashboard frame-rate cap. Idempotent SET (re-sent by the C3).
    int v = (int)data[1];
    if (v >= FPS_TARGET_MIN && v <= FPS_TARGET_MAX) {
      targetFps = v;
      qualityManual = true;
      Serial.printf("[espnow] fps target -> %d\n", v);
    }
  }
  else if (len >= 1 && data[0] == 'A') {
    // Dashboard AUTO: drop the manual pin and let the adaptive controller run
    // quality AND rate again (same effect as the F1 button re-pinning LOW).
    qualityManual = false;
    targetFps = FPS_TARGET_DEFAULT;
    Serial.println("[espnow] quality AUTO (adaptive)");
  }
#if ENABLE_MIC
  else if (len >= 1 && data[0] == 'M') {          // toggle mic from dashboard
    micEnabled = !micEnabled;
    Serial.printf("[espnow] mic %s\n", micEnabled ? "ON" : "OFF");
    if (micEnabled && current > QUALITY_LOW) {
      // Mic on = the link must carry ~8 KB/s of audio too. Step video down to
      // LOW immediately; the adaptive controller is NOT allowed to climb back
      // up while the mic streams (see espnowPoll).
      setQuality(QUALITY_LOW);
    }
  }
#endif
#if ENABLE_SD_RECORDING
  else if (len >= 2 && data[0] == 'R' && (data[1] == '0' || data[1] == '1')) {
    // Dashboard: start / stop the microSD recording. Only a flag is set here -
    // this callback runs in the WiFi task, so the SD write itself happens in
    // espnowPoll() (see the note at recordStartRequest).
    if (data[1] == '1') {
      recordStartRequest = true;
      recordStopRequest  = false;
      Serial.println("[rec] start requested (dashboard)");
    } else {
      recordStopRequest  = true;
      recordStartRequest = false;
      Serial.println("[rec] stop requested (dashboard)");
    }
  }
  else if (len >= 3 && data[0] == 'T') {
    // Max clip length in seconds (0 = record until stopped).
    recorderSetMaxSeconds((uint32_t)(((uint16_t)data[1] << 8) | (uint16_t)data[2]));
  }
  else if (len >= 2 && data[0] == 'P' && data[1] >= '0' && data[1] <= '2') {
    recorderSetPriority((uint8_t)(data[1] - '0'));
  }
#endif
}

// Send one datagram and WAIT for the link ACK, retrying a few times. Used for
// video segments only - see espNowSendLossy for why audio must not use this.
static bool espNowSendReliable(const uint8_t* buffer, size_t len) {
  for (int attempt = 0; attempt < ESPNOW_SEND_RETRIES; ++attempt) {
    sendDone = false;
    esp_err_t err = esp_now_send(peerMac, buffer, len);
    if (err != ESP_OK) {
      txSendErr++;
      // Only print the first few of each error code - do not flood the console.
      static uint8_t logged[6] = {0};
      int code = (int)err;
      if (code >= 0 && code < 6 && logged[code] < 3) {
        logged[code]++;
        Serial.printf("[espnow] esp_now_send err=0x%x (0x104=peer not found? "
                      "0x103=state? check peer add/init)\n", err);
      }
      delay(ESPNOW_RETRY_DELAY_MS);
      continue;
    }
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

// Fire-and-forget datagram: ONE esp_now_send(), no retries, no ACK wait.
// Audio tolerates loss (a missing 30 ms chunk is inaudible), but every retry
// on the old reliable path cost a full ACK-timeout round trip of airtime -
// with the link already saturated that doubled the audio cost and starved
// video segments (the ~80% drop rate seen with the mic enabled).
static bool espNowSendLossy(const uint8_t* buffer, size_t len) {
  esp_err_t err = esp_now_send(peerMac, buffer, len);
  if (err != ESP_OK) {
    txSendErr++;
    static uint8_t logged[6] = {0};
    int code = (int)err;
    if (code >= 0 && code < 6 && logged[code] < 3) {
      logged[code]++;
      Serial.printf("[espnow] esp_now_send err=0x%x (audio, lossy)\n", err);
    }
    return false;
  }
  return true;   // queued; the link layer may still drop it - that is fine
}

#if ENABLE_SD_RECORDING
// ---------------------------------------------------------------------------
// Recording state report -> the C3, which echoes it as an "[rx] REC ..." line
// that host/c3_usb_mjpeg.py shows (and parses) on the dashboard. Fire-and-
// forget: the link is one-way and a lost report is replaced by the next one.
//   {'S', on, sd, prio, framesHi, framesLo, leftHi, leftLo, idxHi, idxLo}
// ---------------------------------------------------------------------------
static void sendRecordStatus(bool force) {
#if RECORD_STATUS_PERIOD_MS > 0
  static uint32_t lastMs = 0;
  if (!peerKnown) return;
  if (!force && (millis() - lastMs) < RECORD_STATUS_PERIOD_MS) return;
  lastMs = millis();

  uint32_t frames = recorderFrameCount();
  uint16_t idx    = recorderFileIndex();
  // left: 0xFFFF = no limit configured, 0 = not recording, else seconds left.
  uint16_t left = 0;
  if (recorderActive()) {
    left = recorderMaxSeconds() ? (uint16_t)recorderSecondsLeft() : 0xFFFF;
  }

  uint8_t msg[10];
  msg[0] = (uint8_t)'S';
  msg[1] = (uint8_t)(recorderActive() ? '1' : '0');
  msg[2] = (uint8_t)(recorderSdReady() ? '1' : '0');
  msg[3] = (uint8_t)('0' + recorderPriority());
  msg[4] = (uint8_t)((frames > 0xFFFF ? 0xFFFF : frames) >> 8);
  msg[5] = (uint8_t)(frames > 0xFFFF ? 0xFF : frames);
  msg[6] = (uint8_t)(left >> 8);
  msg[7] = (uint8_t)left;
  msg[8] = (uint8_t)(idx >> 8);
  msg[9] = (uint8_t)idx;
  esp_now_send(peerMac, msg, sizeof(msg));
#else
  (void)force;   // status reports disabled
#endif
}
#endif  // ENABLE_SD_RECORDING

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

  esp_err_t iniErr = esp_now_init();
  if (iniErr != ESP_OK) Serial.printf("[espnow] esp_now_init FAILED: 0x%x\n", iniErr);

  if (peerKnown) {
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    for (int i = 0; i < 6; ++i) peer.peer_addr[i] = peerMac[i];
    peer.channel = (uint8_t)WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_AP;
    peer.encrypt = false;
    esp_err_t addErr = esp_now_add_peer(&peer);
    Serial.print("[espnow] sending to ");
    for (int i = 0; i < 6; ++i) Serial.printf("%02X%s", peerMac[i], i < 5 ? ":" : "\n");
    Serial.printf("[espnow] add_peer %s (0x%x)\n",
                  addErr == ESP_OK ? "OK" : "FAILED", addErr);
  } else {
    Serial.println("[espnow] peer MAC unset - configure ESPNOW_PEER_MAC in config.h");
  }
  esp_now_register_send_cb(&espNowOnSend);
  esp_now_register_recv_cb(&espNowOnRecv);
  micBegin();

#if ENABLE_SD_RECORDING
  // Mount the microSD card and apply the recorder defaults. Both are also
  // settable at runtime from the dashboard (control channel) or the serial
  // console; a missing card only disables recording, the stream still runs.
  recorderSetMaxSeconds(RECORD_MAX_SECONDS);
  recorderSetPriority(RECORD_PRIORITY_DEFAULT);
  recorderBegin();
  char st[96];
  recorderStatusLine(st, sizeof(st));
  Serial.printf("[rec] %s\n", st);
#endif
}

static void setQuality(QualityLevel l) {
  if (l < QUALITY_LOW) l = QUALITY_LOW;
  // Never exceed the ceiling for this target (HIGH/UXGA frames are far larger
  // than the C3's receive buffer and have crashed the camera before).
  if (l > ESPNOW_MAX_QUALITY) {
    if (l != current) Serial.println("[espnow] quality capped at max for this target");
    l = (QualityLevel)ESPNOW_MAX_QUALITY;
  }
  current = l;
  cameraApplyQuality(l);
}

// Dashboard "fps" mode: mic OFF, quality pinned LOW, up-stepper blocked.
// Shared by the ESP-NOW 'F1'/'F0' control and the S3 console's 'f' key.
static void setFpsPriority(bool on) {
  fpsPriority = on;
  if (!on) { Serial.println("[espnow] fps priority OFF"); return; }
  // fps mode re-asserts its own LOW pin, so drop any previous manual pin.
  qualityManual = false;
#if ENABLE_MIC
  // Audio and a high frame rate do not both fit on this link: the 3 fps mic
  // cap is deliberate, so fps mode gives the microphone up.
  micEnabled = false;
#endif
  // Pin LOW now; the up-stepper would otherwise spend the freed airtime on VGA.
  setQuality(QUALITY_LOW);
  Serial.println("[espnow] fps priority ON (mic off, quality pinned LOW)");
}

void espnowHandleSerialQ() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'q') { delay(5); if (Serial.available()) { char d = Serial.read(); int lv = d - '0';
      if (lv >= QUALITY_LOW && lv <= QUALITY_HIGH) setQuality((QualityLevel)lv); } }
  }
}

// ---------------------------------------------------------------------------
// Serial control from the S3's own console (main.cpp feeds every received char
// here, because main.cpp is the only place that reads Serial - the old
// espnowHandleSerialQ() drain above never sees anything as a result and is kept
// only for API compatibility).
//   q0/q1/q2   quality      r  start recording      e  stop recording
//   D<secs>    max clip length (0 = unlimited)      P<0..2>  priority
//   f          fps-priority mode (mic off, LOW pinned, up-stepper blocked)
//   v<1..60>   frame-rate cap (explicit; overrides profile rate + mic default)
// Returns true when the char was consumed.
// ---------------------------------------------------------------------------
bool espnowHandleSerialKey(char ch) {
  if (ch == 'f') { setFpsPriority(!fpsPriority); return true; }
  if (ch == 'v') {                       // frame-rate cap, same as the 'V' SET
    delay(5);
    int v = 0, n = 0;
    while (Serial.available() && n < 2) {
      char d = (char)Serial.peek();
      if (d < '0' || d > '9') break;
      Serial.read();
      v = v * 10 + (d - '0');
      ++n;
    }
    if (v >= FPS_TARGET_MIN && v <= FPS_TARGET_MAX) {
      targetFps = v; qualityManual = true;
      Serial.printf("[espnow] fps target %d\n", v);
    } else {
      Serial.println("[espnow] usage: v<1..60>");
    }
    return true;
  }
  if (ch == 'q') {
    delay(5);
    if (Serial.available()) {
      int lv = Serial.read() - '0';
      if (lv >= QUALITY_LOW && lv <= QUALITY_HIGH) { setQuality((QualityLevel)lv); return true; }
    }
    return true;
  }
#if ENABLE_SD_RECORDING
  if (ch == 'r') { recordStartRequest = true;  recordStopRequest = false;
                   Serial.println("[rec] start requested (serial)"); return true; }
  if (ch == 'e') { recordStopRequest  = true;  recordStartRequest = false;
                   Serial.println("[rec] stop requested (serial)");  return true; }
  if (ch == 'D') {
    uint32_t secs = 0;
    bool digits = false;
    while (Serial.available()) {
      char d = Serial.peek();
      if (d < '0' || d > '9') break;
      Serial.read();
      digits = true;
      secs = secs * 10 + (uint32_t)(d - '0');
      if (secs > 100000) break;
    }
    if (!digits) { Serial.println("[rec] usage: D<seconds>  (D0 = until stopped)"); return true; }
    recorderSetMaxSeconds(secs);
    return true;
  }
  if (ch == 'P') {
    delay(5);
    if (Serial.available()) {
      char d = Serial.read();
      if (d >= '0' && d <= '2') { recorderSetPriority((uint8_t)(d - '0')); return true; }
    }
    Serial.println("[rec] usage: P<0..2>  (0 = SD first, 1 = balanced, 2 = stream first)");
    return true;
  }
#endif
  return false;
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
    if (ok) txSegOk++; else txSegFail++;
    adaptiveReport(ok);
    if (!ok) allOk = false;
  }
  return allOk;
}

void espnowPoll() {
  espnowHandleSerialQ();
#if ENABLE_SD_RECORDING
  // Auto-stop a clip whose configured maximum length has elapsed...
  bool wasRecording = recorderActive();
  recorderTick();
  // ...and service a stop request from the dashboard / serial.
  if (recordStopRequest) { recordStopRequest = false; recorderStop(); }
  if (wasRecording && !recorderActive()) sendRecordStatus(true);
#endif
#if ENABLE_MIC
  // Audio first (interleaved between video frames); a couple of 30 ms chunks
  // per pass keeps the ring buffer from overflowing without stealing airtime.
  sendAudioChunks();
#endif
  if (!peerKnown) {
    static uint32_t warn = 0;
    if (millis() - warn > 5000) { Serial.println("[espnow] peer not configured yet"); warn = millis(); }
    delay(200); return;
  }
  QualityProfile p = qualityProfileFor(current);
#if ENABLE_MIC
  // The link cannot carry full-rate video AND ~8 KB/s of audio: cap fps hard
  // at 3 while the mic streams (5 fps was still too many datagrams/s -> ~95%
  // drop rate). Fewer, smaller frames leave real airtime gaps for audio.
  if (micEnabled && p.max_fps > 3.0f) p.max_fps = 3.0f;
#endif
  // fps mode pins LOW only for as long as the user has NOT picked a quality of
  // their own. The pin is what makes fps mode fast (a mic-off sender otherwise
  // climbs to VGA and gets SLOWER), but re-asserting it over an explicit choice
  // reverted MEDIUM every pass - invisible in flight and read as a broken
  // button. qualityManual is set by '0'/'1'/'2'/'V', so from then on the user's
  // choice wins.
  if (fpsPriority && !qualityManual && current != QUALITY_LOW) {
    setQuality(QUALITY_LOW);
  }
  if (fpsPriority && targetFps <= 0) p.max_fps = ESPNOW_FPS_PRIORITY_MAX_FPS;
  // An explicit 'V' outranks the profile rate and the 3 fps mic default: the
  // user asked for a number. See FPS_TARGET_MIN/MAX in config.h.
  if (targetFps > 0) p.max_fps = (float)targetFps;
  if (!fpsGate(p.max_fps)) { delay(2); return; }

  camera_fb_t* fb = cameraCapture();
  if (!fb) { delay(5); return; }

  // ONE capture, TWO consumers. Both the SD clip and the air link get the same
  // JPEG, trimmed at its EOI marker (the OV3660 can hand back concatenated
  // JPEGs / padding, and every consumer wants a self-contained image).
  const size_t jlen = recorderJpegLength(fb->buf, fb->len);

  // -------------------------------------------------------------------------
  // microSD recording + priority arbitration.
  //
  // One frame costs (capture + SD write + air send) and the frame slot is
  // 1000/max_fps ms. Whose timing is protected when the slot is too short for
  // both is exactly what RECORD_PRIORITY_* decides:
  //   SD       - write first, and skip the air copy if the card ate the slot
  //   BALANCED - always do both (card first); the loop rate just adapts
  //   STREAM   - send first, and skip the card if the link ate the slot
  // -------------------------------------------------------------------------
  bool streamThis = true;
  bool delivered  = false;
#if ENABLE_SD_RECORDING
  bool recording = recorderActive();
  if (recordStartRequest) {
    // Serviced here, with a real frame in hand: the AVI header needs its size.
    recordStartRequest = false;
    if (recorderStart(fb->width, fb->height,
                      p.max_fps > 0 ? (uint16_t)(p.max_fps + 0.5f) : RECORD_AVI_FPS)) {
      recording = true;
      sendRecordStatus(true);            // tell the dashboard immediately
    }
  }

  const uint8_t prio = recorderPriority();
  if (recording && prio != RECORD_PRIORITY_STREAM) {
    uint32_t t0 = millis();
    bool wrote = recorderWriteFrame(fb->buf, jlen);
    uint32_t sdMs = millis() - t0;
    if (!wrote) Serial.println("[rec] SD write failed (card removed/full?)");
    if (prio == RECORD_PRIORITY_SD && p.max_fps > 0) {
      uint32_t slotMs = (uint32_t)(1000.0f / p.max_fps);
      if (slotMs && sdMs >= slotMs) {
        streamThis = false;              // the card owns this frame slot
      }
    }
  }
#endif

  // Auto-downscale any frame that the receiver's RAM cannot hold.
  bool tooLargeForAir = (fb->len > RECEIVER_MAX_FRAME_BYTES);
  if (streamThis && tooLargeForAir) {
    streamThis = false;
    if (current > QUALITY_LOW) {
      Serial.printf("[espnow] frame too large - downscaling\n");
      setQuality((QualityLevel)(current - 1));
    }
  }

  if (streamThis) {
    delivered = sendFrame(fb->buf, jlen);
    txFrames++;
  }

#if ENABLE_SD_RECORDING
  if (recording && prio == RECORD_PRIORITY_STREAM) {
    if (streamThis) {
      recorderWriteFrame(fb->buf, jlen); // the link is done: the card gets it now
    } else {
      recorderNoteSkippedFrame();        // link owned the slot (or frame too big)
    }
  }
#endif

  cameraRelease(fb);

#if ENABLE_SD_RECORDING
  // Periodic "REC ..." report for the dashboard (rate-limited internally).
  sendRecordStatus(false);
#endif

  // Periodic one-line TX status so a dead / one-way air link is visible on the
  // sender's serial console (the C3 cannot report anything back - one-way).
  static uint32_t lastStat = 0;
  if (millis() - lastStat > 5000) {
    lastStat = millis();
    Serial.printf("[espnow] tx stats: frames=%u segOk=%u segFail=%u sendErr=%u quality=%d\n",
                  (unsigned)txFrames, (unsigned)txSegOk, (unsigned)txSegFail,
                  (unsigned)txSendErr, (int)current);
  }

  if (!streamThis) return;               // SD-first kept this frame off the air

  if (!delivered && current > QUALITY_LOW) {
    setQuality((QualityLevel)(current - 1));   // lighten up and drop this frame
    return;
  }

  int next = adaptiveStep(current);
#if ENABLE_MIC
  // While the mic is streaming, never let the adaptive controller climb back
  // up - more bytes/frame is exactly what saturated the link before.
  if (micEnabled && next > QUALITY_LOW) next = -1;
#endif
  // ...and under any explicit intent: fps mode, or a quality/rate the user set
  // themselves. Climbing to VGA is precisely what makes the mic-off case
  // SLOWER (43 segments/frame) instead of faster.
  if ((fpsPriority || qualityManual) && next > QUALITY_LOW) next = -1;
  if (next >= 0) setQuality((QualityLevel)next);
}