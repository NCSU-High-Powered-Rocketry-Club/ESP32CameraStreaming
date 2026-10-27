// receiver.cpp - ESP32-C3 "dish" side: reassembles segmented ESP-NOW frames
// from the S3 Sense and forwards each complete JPEG over USB CDC (Serial) to a
// host that re-serves it as MJPEG. The C3 no longer serves video over Wi-Fi
// itself - that was half-duplex and contended with RF reception.
//
// Wiring:
//   S3 (airborne) --ESP-NOW--> C3 dish --USB CDC (wired)--> host (Pi/laptop)
//                               | RF receive + reassemble + forward
//                               (no RF TX while serving)
//
// Usage:
//   1. Wire the C3's antenna to the dish / high-gain element for the link.
//   2. Open `pio device monitor` (or press reset after opening it) - the board
//      greets you in console mode with its MAC ('m' prints it again).
//   3. Put that MAC into the sender (config.h -> ESPNOW_PEER_MAC).
//   4. Plug the C3 into the host over USB, then run:
//        python3 host/c3_usb_mjpeg.py
//      (it sends 'g' to switch USB to the frame stream) and open
//      http://<host>:8080/
//
// USB CDC (Serial = TinyUSB):
//     TWO modes, switched at runtime:
//       console  (default)  -> readable banner/logs + m/s commands (pio monitor)
//       stream              -> binary framed JPEG stream (host sends 'g' to
//                              start it, 'x' or unplug returns to console)
//     availableForWrite() gates the writer so a slow host NEVER blocks the RF
//     receive/reassemble loop - if the USB buffer is full we drop/skip frames.
// Hardware UART (Serial0 = GPIO21 TX): mirrors diagnostics in every mode.
//     m = print MAC, s = status, g = start USB stream, x = stop USB stream
// Dashboard controls relayed to the S3 over ESP-NOW (accepted on both ports,
// also used by host/c3_usb_mjpeg.py):
//     0/1/2 = quality preset, M = toggle sender mic,
//     r / e = start / stop the microSD recording on the S3,
//     t<secs> = max recording length (t0 = until stopped),
//     p<0..2> = recording/streaming priority (0 = SD first, 1 = balanced,
//               2 = stream first)
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
static uint32_t rxFrames = 0;       // ESP-NOW frames completed (any source)

#define MAX_SEGMENTS  ((RECEIVER_MAX_FRAME_BYTES + NOW_PAYLOAD_BYTES - 1) / NOW_PAYLOAD_BYTES + 1)
static uint8_t  segBitmap[(MAX_SEGMENTS + 7) / 8];

static uint16_t asmFrameId = 0xFFFF;
static uint16_t asmTotal   = 0;
static uint16_t asmTailPlen = 0;    // payload len of the highest segment seen
static uint32_t asmCount   = 0;     // distinct segments seen
static uint32_t asmLastMs  = 0;     // for stale-frame timeout

// Mutex guarding latestBuf/latestLen: onRecv() runs in the WiFi task while
// pumpUsb() runs in the loop task, so snapshotting a frame (crc32 over it,
// which takes ~ms) without a lock let a new frame overwrite latestBuf mid-
// snapshot - header CRC of the old buffer + payload of the new one, i.e. EVERY
// frame failed the host's CRC check.
static SemaphoreHandle_t latestMutex = nullptr;

// "WiFi lost" log re-arm flag (shared by the loop() watchdog below).
static bool wifiDownLogged = false;
static void downLoggedReset() { wifiDownLogged = false; }

// Broadcast address for the control back-channel to the S3 sender.
static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ---------------------------------------------------------------------------
// Audio path (8 kHz unsigned 8-bit PCM from the sender's PDM mic).
// onRecv appends tagged audio payloads into a FIFO; pumpUsbAudio() drains it
// to the USB host with the standard 12-byte header (audio magic). Audio never
// blocks: if the host is slow or the FIFO is full, audio is dropped.
// ---------------------------------------------------------------------------
static uint8_t  audioFifo[AUDIO_FIFO_BYTES];
static volatile uint32_t audioIn = 0, audioOut = 0, audioDrop = 0;
static uint16_t audioLastSeq = 0;
static uint8_t  audioHdr[USB_HEADER_BYTES];

static uint32_t audioFifoAvail() { return (audioIn >= audioOut) ? audioIn - audioOut
                                            : AUDIO_FIFO_BYTES - audioOut + audioIn; }
static uint32_t audioFifoFree()  { return AUDIO_FIFO_BYTES - 1 - audioFifoAvail(); }

static bool segBitGet(unsigned idx) { return (segBitmap[idx >> 3] & (uint8_t)(1u << (idx & 7))) != 0; }
static void segBitSet(unsigned idx) { segBitmap[idx >> 3] |= (uint8_t)(1u << (idx & 7)); }
static void segBitClear() { for (unsigned i = 0; i < sizeof(segBitmap); ++i) segBitmap[i] = 0; }

// ---------------------------------------------------------------------------
// USB CDC forwarder state (non-blocking pump; mirrors the sender's time-slice
// pattern so one slow host can never stall the RF receive loop).
// ---------------------------------------------------------------------------
static uint8_t  outBuf[USB_HEADER_BYTES];  // prebuilt framing header
static volatile uint8_t outState = 0;      // 0 idle, 1 delivering a frame
static uint32_t outLen   = 0;              // payload length of frame being sent
static size_t   outOff   = 0;              // bytes of payload already flushed
static uint8_t  outHdrPos = 0;             // header bytes queued so far
static uint32_t usbFrames = 0;             // frames written to the host
static bool     usbStreaming = false;     // false = readable console on USB,
                                          // true = binary frame stream (host sent 'g')

// Max payload bytes pushed per loop pass (availableForWrite() caps us too).
static const size_t USB_SEND_SLICE_BYTES = 1024;

// --- CRC-32 (ISO-HDLC / zlib polynomial 0xEDB88320) --------------------------
// Same standard as Python's zlib.crc32 so the host can validate each frame.
static uint32_t crc32Table[256];
static bool     crc32Ready = false;

static void crc32Init() {
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int k = 0; k < 8; ++k)
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32Table[i] = c;
  }
  crc32Ready = true;
}

static uint32_t crc32(const uint8_t* data, size_t len) {
  if (!crc32Ready) crc32Init();
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i)
    c = crc32Table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// Console lines queued for the USB host (recording status from the S3, live
// link/RSSI reports). onRecv() runs in the WiFi task, so the ring is guarded by
// a critical section; loop() is the only consumer and flushes a line only while
// the USB pump is BETWEEN frames (outState == 0) - console text can therefore
// never land inside a half-delivered JPEG.
// ---------------------------------------------------------------------------
#define USB_LINE_SLOTS 4
#define USB_LINE_LEN   160
static char             usbLineBuf[USB_LINE_SLOTS][USB_LINE_LEN];
static volatile uint8_t usbLineHead = 0, usbLineTail = 0;
static portMUX_TYPE     usbLineMux = portMUX_INITIALIZER_UNLOCKED;

static void queueUsbLine(const char* line) {
  portENTER_CRITICAL(&usbLineMux);
  uint8_t next = (uint8_t)((usbLineHead + 1) % USB_LINE_SLOTS);
  if (next != usbLineTail) {          // ring full -> drop (another follows soon)
    strncpy(usbLineBuf[usbLineHead], line, USB_LINE_LEN - 1);
    usbLineBuf[usbLineHead][USB_LINE_LEN - 1] = '\0';
    usbLineHead = next;
  }
  portEXIT_CRITICAL(&usbLineMux);
}

void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Recording status from the S3 ({'S', on, sd, prio, frames(2BE), left(2BE),
  // idx(2BE)}). Not a video/audio payload - surface it as a console line that
  // the host viewer displays (and parses) on the dashboard.
  if (len >= 10 && data[0] == REC_STATUS_TAG) {
    const bool on   = (data[1] == '1');
    const bool sd   = (data[2] == '1');
    const uint16_t frames = (uint16_t)((data[4] << 8) | data[5]);
    const uint16_t left   = (uint16_t)((data[6] << 8) | data[7]);
    const uint16_t idx    = (uint16_t)((data[8] << 8) | data[9]);
    char leftTxt[16];
    if (!on)                   snprintf(leftTxt, sizeof(leftTxt), "-");
    else if (left == 0xFFFF)   snprintf(leftTxt, sizeof(leftTxt), "unlimited");
    else                       snprintf(leftTxt, sizeof(leftTxt), "%us", (unsigned)left);
    char line[128];
    snprintf(line, sizeof(line),
             "[rx] REC state=%s idx=%u frames=%u left=%s prio=%c sd=%u\n",
             on ? "on" : "off", (unsigned)idx, (unsigned)frames, leftTxt,
             (char)data[3], (unsigned)(sd ? 1 : 0));
    Serial0.print(line);                  // hw UART: immediately, always
    queueUsbLine(line);                   // USB host: flushed between frames
    return;
  }

  // Audio datagrams: [0xA3 0xA5][seq:2BE] + PCM. Never touch video state.
  if (len > 4 && data[0] == (AUDIO_AIR_MAGIC >> 8) &&
      data[1] == (AUDIO_AIR_MAGIC & 0xFF)) {
    uint16_t seq = (uint16_t)((data[2] << 8) | data[3]);
    size_t plen = (size_t)(len - 4);
    audioLastSeq = seq;
    if (audioFifoFree() >= plen) {
      size_t n = (audioIn + plen > AUDIO_FIFO_BYTES)
                 ? AUDIO_FIFO_BYTES - audioIn : plen;      // handle wrap
      memcpy(audioFifo + audioIn, data + 4, n);
      if (n < plen) memcpy(audioFifo, data + 4 + n, plen - n);
      audioIn = (audioIn + plen) % AUDIO_FIFO_BYTES;
    } else {
      audioDrop++;                        // host too slow - drop, never block
    }
    return;
  }
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

    // Only hand the host a NEW frame if we are not mid-delivery over USB -
    // otherwise the pump is reading latestBuf and a write here would tear it.
    if (outState == 0) {
      // Claim the mutex so the pump cannot snapshot this frame while we are
      // still copying it (and vice versa - see note at latestMutex).
      if (latestMutex && xSemaphoreTake(latestMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        asmFrameId = 0xFFFF;               // pump busy with the previous frame
        rxFrames++;
        return;
      }
      memcpy(latestBuf, assembleBuf, realLen);
      latestLen = realLen;
      latestFrameId = frameId;
      asmFrameId = 0xFFFF;   // next new frame restarts cleanly
      rxFrames++;
      if (latestMutex) xSemaphoreGive(latestMutex);
      Serial0.printf("[rx] frame %u ok (%u B, %u seg)\n", frameId, realLen, total);
      if (!usbStreaming)     // console mode: USB monitor can show this too
        Serial.printf("[rx] frame %u ok (%u B, %u seg)\n", frameId, realLen, total);
    } else {
      asmFrameId = 0xFFFF;   // still re-arm, we just skip handing it over
      rxFrames++;
    }
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
// Non-blocking USB CDC pump. Each loop pass we flush a bounded slice of the
// latest frame; if Serial's write buffer is full (slow host / no host plugged
// in) availableForWrite() returns tiny/zero and we drop the frame instead of
// ever blocking the RF path. The host re-syncs by scanning for the magic.
// ---------------------------------------------------------------------------
static void pumpUsb() {
  if (!usbStreaming || !Serial) return;   // console mode / no USB host: no frames

  // Start a fresh frame only when we can fit its whole header without blocking.
  if (outState == 0) {
    if (!latestLen) return;
    if ((size_t)Serial.availableForWrite() < USB_HEADER_BYTES) return; // host busy - defer
    // Lock the frame while we CRC + snapshot it. crc32 over a ~30 KB frame
    // takes ~ms, and onRecv() (WiFi task) overwrites latestBuf freely - this
    // lock is what keeps header/payload/CRC from coming from different frames.
    if (latestMutex && xSemaphoreTake(latestMutex, 0) != pdTRUE) return; // busy - defer
    uint32_t crc = crc32(latestBuf, latestLen);
    outBuf[0] = (uint8_t)USB_MAGIC_HI;
    outBuf[1] = (uint8_t)USB_MAGIC_LO;
    outBuf[2] = (uint8_t)(latestLen >> 24); outBuf[3] = (uint8_t)(latestLen >> 16);
    outBuf[4] = (uint8_t)(latestLen >> 8);  outBuf[5] = (uint8_t)(latestLen);
    outBuf[6] = (uint8_t)(latestFrameId >> 8); outBuf[7] = (uint8_t)(latestFrameId);
    outBuf[8] = (uint8_t)(crc >> 24);  outBuf[9]  = (uint8_t)(crc >> 16);
    outBuf[10] = (uint8_t)(crc >> 8);  outBuf[11] = (uint8_t)(crc);
    outLen = latestLen;
    outOff = 0;
    outHdrPos = 0;
    outState = 1;
    if (latestMutex) xSemaphoreGive(latestMutex);
  }

  // Push header+payload in bounded slices; never block, and always honour the
  // byte count write() actually queued (a short write must not shift outOff,
  // or the host's CRC would fail on every frame). IMPORTANT: if the USB write
  // buffer is full mid-frame we simply RETURN and resume next pass - we must
  // NOT reset outState, because the header (and possibly part of the payload)
  // is already on the wire. Aborting and re-starting the frame would inject a
  // duplicate header inside the previous payload and desync the host's parser
  // on every frame. onRecv() won't overwrite latestBuf while outState==1.
  uint32_t pushed = 0;
  while (outState == 1 && pushed < USB_SEND_SLICE_BYTES) {
    size_t freeNow = (size_t)Serial.availableForWrite();
    if (freeNow == 0) return;                          // host busy - resume next pass

    if (outHdrPos < USB_HEADER_BYTES) {                // header, byte-exact
      size_t n = USB_HEADER_BYTES - outHdrPos;
      if (n > freeNow) n = freeNow;
      size_t w = Serial.write(outBuf + outHdrPos, n);
      if (w == 0) return;                              // resume next pass
      outHdrPos += (uint8_t)w;
      pushed += (uint32_t)w;
      continue;
    }

    size_t left = outLen - outOff;
    if (left == 0) {
      outState = 0;
      latestLen = 0;   // consumed - do NOT resend the same frame forever
      usbFrames++;
      return;
    }
    size_t chunk = left;
    if (chunk > freeNow) chunk = freeNow;
    if (chunk > (USB_SEND_SLICE_BYTES - pushed)) chunk = USB_SEND_SLICE_BYTES - pushed;
    size_t w = Serial.write(latestBuf + outOff, chunk);
    if (w == 0) return;                                // resume next pass
    outOff += w;
    pushed += (uint32_t)w;
  }
}

// Drain the audio FIFO to the USB host (8 kHz unsigned 8-bit PCM, tagged with
// the audio magic so the host can separate it from JPEG frames). Never blocks
// and never interleaves with a video frame's delivery; slow host = dropped
// audio, which beats blocking the RF path.
static void pumpUsbAudio() {
  if (!usbStreaming || !Serial) return;
  if (outState != 0) return;              // never interleave with a video frame
  uint32_t avail = audioFifoAvail();
  if (!avail) return;
  if (avail > 240) avail = 240;           // one USB packet per pass
  if ((size_t)Serial.availableForWrite() < USB_HEADER_BYTES + avail) return;

  audioHdr[0] = (uint8_t)USB_AUDIO_MAGIC_HI;
  audioHdr[1] = (uint8_t)USB_AUDIO_MAGIC_LO;
  audioHdr[2] = (uint8_t)(avail >> 24); audioHdr[3] = (uint8_t)(avail >> 16);
  audioHdr[4] = (uint8_t)(avail >> 8);  audioHdr[5] = (uint8_t)(avail);
  audioHdr[6] = (uint8_t)(audioLastSeq >> 8); audioHdr[7] = (uint8_t)(audioLastSeq);
  uint32_t crc = crc32(audioFifo + audioOut, avail);
  audioHdr[8] = (uint8_t)(crc >> 24); audioHdr[9] = (uint8_t)(crc >> 16);
  audioHdr[10] = (uint8_t)(crc >> 8); audioHdr[11] = (uint8_t)(crc);
  Serial.write(audioHdr, USB_HEADER_BYTES);
  size_t n = (audioOut + avail > AUDIO_FIFO_BYTES)
             ? AUDIO_FIFO_BYTES - audioOut : avail;         // handle wrap
  Serial.write(audioFifo + audioOut, n);
  if (n < avail) Serial.write(audioFifo, avail - n);
  audioOut = (audioOut + avail) % AUDIO_FIFO_BYTES;
}

// ---------------------------------------------------------------------------
// Dashboard control relay.
// 'Q'/'R'/'T'/'P' are idempotent "set" commands, so they are re-sent a few
// times: ESP-NOW broadcasts are not ACKed and one lost datagram = one dead
// button press. 'M' is a TOGGLE and must never be repeated (an even number of
// deliveries would cancel itself out), so it is sent exactly once.
// ---------------------------------------------------------------------------
static void relayToSender(const uint8_t* cmd, size_t len, const char* label) {
  int sent = 0;
  for (int i = 0; i < 3; ++i) {
    if (esp_now_send(BCAST_MAC, cmd, len) == ESP_OK) sent++;
    delay(2);
  }
  char line[64];
  snprintf(line, sizeof(line), "[rx] %s -> S3 (%d/3 sent)\n", label, sent);
  Serial0.print(line);
  if (!usbStreaming) Serial.print(line);
}

// Read the decimal argument that follows a command char: the dashboard sends
// "t30" / "p2", and a human types the same in `pio device monitor`. Returns -1
// when no digit arrives inside the window. Once the FIRST digit is in, it only
// waits ~20 ms for the rest, so a relayed "t30" costs the streaming loop a few
// milliseconds instead of the full window.
static int readDecArg(int maxDigits) {
  int v = -1;
  uint32_t t0 = millis();
  uint32_t idleFrom = 0;
  while (millis() - t0 < 300) {
    int c = -1;
    if (Serial.available())        c = Serial.read();
    else if (Serial0.available())  c = Serial0.read();
    if (c < 0) {
      if (v >= 0) {                       // digits seen: wait briefly for more
        if (!idleFrom) idleFrom = millis();
        else if (millis() - idleFrom > 20) break;
      }
      delay(2);
      continue;
    }
    idleFrom = 0;
    if (c < '0' || c > '9') break;
    v = (v < 0 ? 0 : v) * 10 + (c - '0');
    if (--maxDigits <= 0) break;
  }
  return v;
}

// Flush one queued console line to the USB host. In console mode it goes out
// immediately; in stream mode it waits until the pump is BETWEEN frames
// (outState == 0) so console text can never land inside a JPEG delivery - the
// host's parser would resync anyway, but there is no reason to inject noise.
// If the host's write buffer is full the line stays queued and is retried next
// pass; if the ring overflows, a status line is dropped (another follows within
// LINK_REPORT_PERIOD_MS / RECORD_STATUS_PERIOD_MS).
static void flushUsbLine() {
  if (usbLineTail == usbLineHead) return;        // nothing queued
  if (usbStreaming && outState != 0) return;     // never split a frame
  if (!Serial) return;                           // no USB host attached

  char buf[USB_LINE_LEN];
  portENTER_CRITICAL(&usbLineMux);
  memcpy(buf, usbLineBuf[usbLineTail], sizeof(buf));   // peek (producers only
  portEXIT_CRITICAL(&usbLineMux);                      //  ever write at head)
  buf[sizeof(buf) - 1] = '\0';

  size_t len = strlen(buf);
  if ((size_t)Serial.availableForWrite() < len) return;  // host busy - retry
  Serial.print(buf);

  portENTER_CRITICAL(&usbLineMux);
  if (usbLineTail != usbLineHead)
    usbLineTail = (uint8_t)((usbLineTail + 1) % USB_LINE_SLOTS);
  portEXIT_CRITICAL(&usbLineMux);
}

// Live link level. The C3 receives the ESP-NOW video on the channel of its AP
// connection, so WiFi.RSSI() is the level of the link the video actually uses.
// Reported ~1 Hz, and sooner when the level moves by LINK_REPORT_MIN_DELTA dBm
// (so a fading link is visible immediately instead of up to a second late).
static void reportLink() {
  static uint32_t lastMs = 0;
  static int      lastRssi = 0;
  static bool     haveLast = false;
  uint32_t now = millis();
  bool connected = (WiFi.status() == WL_CONNECTED);
  int rssi = connected ? (int)WiFi.RSSI() : 0;
  bool moved = !haveLast || abs(rssi - lastRssi) >= LINK_REPORT_MIN_DELTA;
  if (now - lastMs < LINK_REPORT_PERIOD_MS && !(moved && now - lastMs >= 250))
    return;
  lastMs = now;
  lastRssi = rssi;
  haveLast = true;

  char line[64];
  snprintf(line, sizeof(line), "[rx] link rssi=%d ch=%u sta=%u\n",
           rssi, (unsigned)WiFi.channel(), connected ? 1u : 0u);
  Serial0.print(line);
  queueUsbLine(line);
}

// ---------------------------------------------------------------------------
// Commands (accepted on BOTH ports). 'g'/'x' switch the USB CDC between the
// readable console (pio monitor: banner, logs, m/s) and the binary frame
// stream that host/c3_usb_mjpeg.py consumes. 'm'/'s' always answer on the
// hardware UART; on USB they only echo in console mode, so text can never be
// interleaved into the framed JPEG stream.
// ---------------------------------------------------------------------------
static void handleCommand(char ch) {
  if (ch == 'g') {                       // host viewer: start the frame stream
    usbStreaming = true;
    outState = 0;                        // drop any stale partial delivery
    audioOut = audioIn;                  // flush stale audio, start fresh
    Serial0.println("[rx] USB CDC -> STREAM mode (binary frames to host)");
  } else if (ch == '0' || ch == '1' || ch == '2') {
    // Quality control from the dashboard: relay as an ESP-NOW broadcast to
    // the S3 sender ('Q' + level).
    uint8_t cmd[2] = { (uint8_t)'Q', (uint8_t)ch };
    char label[32];
    snprintf(label, sizeof(label), "quality %c", ch);
    relayToSender(cmd, sizeof(cmd), label);
  } else if (ch == 'F') {
    // Dashboard "fps" mode: the sender drops the mic and pins quality at LOW so
    // the freed airtime goes to frame rate. Idempotent SET -> relayed 3x.
    int lv = readDecArg(1);
    if (lv != 0 && lv != 1) {
      Serial0.println("[rx] usage: F0 / F1   (fps priority off / on)");
      if (!usbStreaming) Serial.println("[rx] usage: F0 / F1");
    } else {
      uint8_t cmd[2] = { (uint8_t)FPS_CTRL_TAG_F, (uint8_t)('0' + lv) };
      relayToSender(cmd, sizeof(cmd), lv ? "fps priority ON" : "fps priority OFF");
    }
  } else if (ch == 'V' || ch == 'v') {
    // Dashboard frame-rate cap: an absolute limit, 1..60 fps, on the sender.
    int fps = readDecArg(2);
    if (fps < 1 || fps > 60) {
      Serial0.println("[rx] usage: V<1..60>   (fps cap)");
      if (!usbStreaming) Serial.println("[rx] usage: V<1..60>");
    } else {
      uint8_t cmd[2] = { (uint8_t)FPS_CTRL_TAG_V, (uint8_t)fps };
      char label[32];
      snprintf(label, sizeof(label), "fps cap %d", fps);
      relayToSender(cmd, sizeof(cmd), label);
    }
  } else if (ch == 'A') {
    // Dashboard AUTO: hand quality AND rate back to the adaptive controller.
    uint8_t cmd = (uint8_t)QUAL_CTRL_TAG_A;
    relayToSender(&cmd, 1, "quality AUTO");
  } else if (ch == 'r' || ch == 'e') {
    // Dashboard: start ('r') / stop ('e') the microSD recording on the S3.
    uint8_t cmd[2] = { (uint8_t)REC_CTRL_TAG_R, (uint8_t)(ch == 'r' ? '1' : '0') };
    relayToSender(cmd, sizeof(cmd), ch == 'r' ? "rec start" : "rec stop");
  } else if (ch == 't') {
    // Dashboard: maximum recording length in seconds ("t30"; t0 = until
    // stopped). 5 digits max -> up to 99999, clamped by the sender.
    int secs = readDecArg(5);
    if (secs < 0) {
      Serial0.println("[rx] usage: t<seconds>   (t0 = record until stopped)");
      if (!usbStreaming) Serial.println("[rx] usage: t<seconds>");
    } else {
      uint8_t cmd[3] = { (uint8_t)REC_CTRL_TAG_T,
                         (uint8_t)((uint16_t)secs >> 8),
                         (uint8_t)((uint16_t)secs & 0xFF) };
      char label[40];
      snprintf(label, sizeof(label), "rec max %ds", secs);
      relayToSender(cmd, sizeof(cmd), label);
    }
  } else if (ch == 'p') {
    // Dashboard: recording/streaming priority (0 = SD first, 1 = balanced,
    // 2 = stream first).
    int prio = readDecArg(1);
    if (prio < 0 || prio > 2) {
      Serial0.println("[rx] usage: p<0..2>   (0 = SD first, 1 = balanced, 2 = stream first)");
      if (!usbStreaming) Serial.println("[rx] usage: p<0..2>");
    } else {
      uint8_t cmd[2] = { (uint8_t)REC_CTRL_TAG_P, (uint8_t)('0' + prio) };
      char label[40];
      snprintf(label, sizeof(label), "rec priority %d", prio);
      relayToSender(cmd, sizeof(cmd), label);
    }
  } else if (ch == 'M') {                // dashboard: toggle sender microphone
    // Toggle: sent ONCE (see the note at relayToSender).
    uint8_t cmd = (uint8_t)'M';
    esp_err_t e = esp_now_send(BCAST_MAC, &cmd, 1);
    Serial0.printf("[rx] mic toggle -> S3 (%s)\n",
                   e == ESP_OK ? "sent" : "SEND FAILED");
    if (!usbStreaming)
      Serial.printf("[rx] mic toggle -> S3 (%s)\n",
                    e == ESP_OK ? "sent" : "SEND FAILED");
  } else if (ch == 'x') {                // host closing / manual stop
    usbStreaming = false;
    outState = 0;
    Serial0.println("[rx] USB CDC -> CONSOLE mode (m/s available)");
    Serial.println("[rx] console mode (m = MAC, s = status)");
  } else if (ch == 'm') {
    uint8_t d[6]; WiFi.macAddress(d);
    Serial0.printf("MAC %02X:%02X:%02X:%02X:%02X:%02X IP %s\n",
                   d[0],d[1],d[2],d[3],d[4],d[5], WiFi.localIP().toString().c_str());
    if (!usbStreaming)
      Serial.printf("MAC %02X:%02X:%02X:%02X:%02X:%02X IP %s\n",
                    d[0],d[1],d[2],d[3],d[4],d[5], WiFi.localIP().toString().c_str());
  } else if (ch == 's') {
    Serial0.printf("latest frame #%u len %u  espnow rx %u  usb written %u  mode %s\n",
                   latestFrameId, (unsigned)latestLen, (unsigned)rxFrames,
                   (unsigned)usbFrames, usbStreaming ? "stream" : "console");
    if (!usbStreaming)
      Serial.printf("latest frame #%u len %u  espnow rx %u  usb written %u  mode %s\n",
                    latestFrameId, (unsigned)latestLen, (unsigned)rxFrames,
                    (unsigned)usbFrames, usbStreaming ? "stream" : "console");
  }
}

static void handleSerialInput() {
  while (Serial0.available()) handleCommand(Serial0.read());
  while (Serial.available())  handleCommand(Serial.read());
}

// ---------------------------------------------------------------------------
// Console banner. TinyUSB drops every Serial write while the USB host has not
// asserted DTR, so text printed "at boot" is lost if the monitor is opened
// later. We therefore re-greet on EVERY fresh USB attach (monitor or host
// viewer) - opening `pio device monitor` always shows something, and the host
// viewer can show the banner as on-page proof of which board it is talking to.
// ---------------------------------------------------------------------------
static bool usbConnected = false;

static void consoleHello() {
  uint8_t d[6]; WiFi.macAddress(d);
  const char* line1 = "\n[rx] ESP32-C3 dish receiver | WiFi '%s' | MAC %02X:%02X:%02X:%02X:%02X:%02X\n";
  const char* line2 = "[rx] console: m = MAC, s = status, g = start USB stream, x = stop\n"
                      "[rx] ctrl: 0/1/2 quality, r/e rec start/stop, t<secs> max length, "
                      "p<0..2> priority, M mic, F0/F1 fps mode, V<1..60> fps cap, A auto\n";
  Serial0.printf(line1, WIFI_SSID, d[0],d[1],d[2],d[3],d[4],d[5]);
  Serial0.print(line2);
  Serial.printf(line1, WIFI_SSID, d[0],d[1],d[2],d[3],d[4],d[5]);  // no-op if no host
  Serial.print(line2);
}

void setup() {
  // USB CDC carries the binary frame stream to the host (or a console until
  // the host asks for the stream with 'g').
  Serial.begin((uint32_t)USB_BAUD);
  // Hardware UART (GPIO21/20) mirrors diagnostics in every mode.
  Serial0.begin(115200);
  latestMutex = xSemaphoreCreateMutex();   // guards latestBuf (onRecv vs pump)
  delay(300);

  // Give an already-open monitor a moment to attach (assert DTR) so the boot
  // banner is not silently dropped by TinyUSB.
  for (uint32_t t0 = millis(); !Serial && millis() - t0 < 2000; ) delay(50);

  consoleHello();
  usbConnected = (bool)Serial;
  Serial0.println("[rx] joining WiFi...");
  if (Serial) Serial.println("[rx] joining WiFi...");

  // STA on the S3 AP keeps both on the same WiFi channel -> ESP-NOW works.
  WiFi.mode(WIFI_STA);
  WiFi.config(CLIENT_IPADDR, AP_IPADDR, CLIENT_NETMASK);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t lastDot = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    // Progress + late monitor attach are visible on BOTH ports.
    if (millis() - lastDot > 1000) {
      lastDot = millis();
      Serial0.print('.');
      if (Serial) Serial.print('.');
    }
    if (!usbConnected && Serial) { usbConnected = true; consoleHello(); }
  }
  Serial0.println();
  if (Serial) Serial.println();
  const char* conn = "[rx] connected to %s, ip %s\n";
  Serial0.printf(conn, WIFI_SSID, WiFi.localIP().toString().c_str());
  Serial.printf(conn, WIFI_SSID, WiFi.localIP().toString().c_str());

  Serial0.println("[rx] connected - ESP-NOW RX: disabling modem power-save (needed for reliable ESP-NOW while STA-connected)");
  if (Serial) Serial.println("[rx] connected - ESP-NOW RX: disabling modem power-save");
  WiFi.setSleep(false);   // STA power-save can silently drop ESP-NOW datagrams

  esp_now_init();
  esp_now_register_recv_cb(&onRecv);

  // Broadcast peer for the control back-channel (quality change requests to
  // the S3 sender: payload 'Q' + level). The sender's recv callback accepts
  // from any MAC, so a broadcast peer is all we need.
  esp_now_peer_info_t bcast;
  memset(&bcast, 0, sizeof(bcast));
  for (int i = 0; i < 6; ++i) bcast.peer_addr[i] = 0xFF;
  bcast.channel = (uint8_t)WIFI_CHANNEL;
  bcast.ifidx   = WIFI_IF_STA;
  bcast.encrypt = false;
  if (esp_now_add_peer(&bcast) != ESP_OK)
    Serial0.println("[rx] broadcast peer add FAILED (quality relay disabled)");
  Serial0.println("[rx] ready - waiting for ESP-NOW frames from the S3");
  if (Serial) Serial.println("[rx] ready - waiting for ESP-NOW frames from the S3");
}

void loop() {
  checkAssembleTimeout();

  // WiFi watchdog: if the S3's AP drops (S3 reboot / out of range) the STA
  // must re-associate, or the ESP-NOW receive side goes silently deaf (the
  // radio can end up scanning on the wrong channel). Reconnect aggressively.
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastAttempt = 0;
    if (millis() - lastAttempt > 2000) {
      lastAttempt = millis();
      if (!wifiDownLogged) {
        wifiDownLogged = true;
        Serial0.println("[rx] WiFi lost - reconnecting...");
        if (Serial && !usbStreaming) Serial.println("[rx] WiFi lost - reconnecting...");
      }
      WiFi.reconnect();
    }
  } else {
    downLoggedReset();   // re-arm the "WiFi lost" log for the next outage
  }

  // Track USB attach/detach. On every FRESH attach we greet (banner + MAC) so
  // a monitor opened at any time - or the host viewer - gets readable proof of
  // which board/port it is on. TinyUSB had silently dropped earlier writes.
  bool connected = (bool)Serial;
  if (usbConnected && !connected) {
    usbConnected = false;
    usbStreaming = false;   // host gone -> back to the readable console
    outState = 0;
    Serial0.println("[rx] USB host gone -> CONSOLE mode");
  } else if (connected && !usbConnected && !usbStreaming) {
    usbConnected = true;
    consoleHello();
  }

  pumpUsb();
  pumpUsbAudio();
  flushUsbLine();          // queued REC / link lines, between frames
  reportLink();            // live RSSI -> "[rx] link rssi=... " (host SIGNAL card)
  handleSerialInput();

  // Streaming but the air side is silent? Tell the user where to look. Safe
  // to print on USB: the pump only writes binary while latestLen is non-zero,
  // so with no frames in flight the port is idle (no torn frames possible).
  static uint32_t lastIdleHint = 0;
  if (usbStreaming && !latestLen && millis() - lastIdleHint > 5000) {
    lastIdleHint = millis();
    const char* msg = (rxFrames == 0)
      ? "[rx] streaming, but NO ESP-NOW frames yet - is the S3 sender up?\n"
        "[rx]   - does the S3 console print '[espnow] sending to <MAC>' at boot?\n"
        "[rx]   - is ESPNOW_PEER_MAC in src/config.h set to THIS C3's MAC (see banner above)?\n"
      : "[rx] streaming, frames were received but none is in flight right now\n";
    Serial0.print(msg);
    if (!usbStreaming)   // never write console text into the binary USB stream
      Serial.print(msg);
  }
}