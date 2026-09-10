// stream_browser.cpp - minimal, no-framework HTTP/MJPEG server for the
// phone/browser target. Uses a raw WiFiServer so we control streaming/error
// handling exactly (timeouts, client-gone detection, quality control).
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "camera_hal.h"
#include "stream_browser.h"

static WiFiServer srv;
static uint32_t   lastFrameMs = 0;
static QualityLevel current = QUALITY_DEFAULT;

// Non-blocking MJPEG stream state: while a stream is active we keep the client
// alive and push frame BYTES in small chunks from browserPoll() without ever
// blocking the loop, so /stream AND /quality are both serviced concurrently and
// one huge frame can never stall the whole server (the previous blocking
// mjpegWrite + single large UXGA frame was wedging the loop until reboot).
static WiFiClient streamClient;
static bool       streaming = false;
static uint32_t   streamFrames = 0;
static uint32_t   liveFailures = 0;    // consecutive capture/write failures

// Incremental frame-send state: once we start a frame we MUST deliver all of it
// (Content-Length), so we keep the frame + offset alive across loop iterations.
static camera_fb_t* outFb    = NULL;   // frame currently being streamed
static size_t       outOff   = 0;      // bytes of outFb already written
static char         outHdr[64];
static int          outHdrLen = 0, outHdrPos = 0;

// Max bytes we try to push per browserPoll() call (time-slicing budget).
static const uint32_t SEND_SLICE_BYTES = 2048;

static const char INDEX_HTML[] =
  "<!DOCTYPE html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>XIAO S3 Sense</title>"
  "<style>button.q{background:#333;color:#eee;border:1px solid #555;"
  "padding:6px 14px;margin:4px;cursor:pointer}</style>"
  "</head><body style='background:#000;color:#eee'>"
  "<h2>XIAO ESP32-S3 Sense</h2>"
  "<img src='/stream' style='width:100%;max-width:640px;image-rendering:auto'>"
  "<br><div>Quality:"
  "<button class='q' onclick='setQ(0)'>LOW</button>"
  "<button class='q' onclick='setQ(1)'>MED</button>"
  "<button class='q' onclick='setQ(2)'>HIGH</button>"
  "<span id='st'></span>"
  "</div><script>"
  "function setQ(n){"
  "  var s=document.getElementById('st');s.textContent='...';"
  "  fetch('/quality?level='+n,{cache:'no-store'}).then(function(r){return r.text()}).then(function(t){"
  "    s.textContent=t;setTimeout(function(){s.textContent=''},1500);}).catch(function(){"
  "    s.textContent='failed';});"
  "}"
  "</script></body></html>";

void browserBegin() {
  // IMPORTANT: the WiFi stack + lwIP tcpip thread must be started BEFORE the
  // TCP server. Without this the first WiFiServer op trips lwIP's
  // "Invalid mbox" assert. Bring up the soft AP (same as the ESP-NOW path).
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IPADDR, AP_IPADDR, AP_NETMASK);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  delay(300);

  srv = WiFiServer(SERVER_PORT);
  srv.begin();
  Serial.printf("[browser] AP \"%s\"  http://%s  (target browser MJPEG)\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("[browser] join the %s network, then open the URL above\n",
                WIFI_SSID);
}

// Apply a global quality level and keep the flag for new streams.
static void setQuality(QualityLevel l) {
  if (l < QUALITY_LOW) l = QUALITY_LOW;
  if (l > QUALITY_HIGH) l = QUALITY_HIGH;
  current = l;
  cameraApplyQuality(l);
}

// Per-frame FPS cap for the current profile.
static bool fpsGate() {
  QualityProfile p = qualityProfileFor(current);
  uint32_t period = p.max_fps > 0.0f ? (uint32_t)(1000.0f / p.max_fps) : 0;
  if (period == 0) return true;
  uint32_t now = millis();
  if (now - lastFrameMs < period) return false;
  lastFrameMs = now;
  return true;
}

// ---------------------------------------------------------------------------
// Incremental, non-blocking MJPEG streaming.
// Each browserPoll() only writes a bounded slice (SEND_SLICE_BYTES) and then
// returns, so the loop stays free to answer /quality and the camera keeps
// producing frames. A slow client simply makes the stream's frame rate drop -
// it can never hang the board.
// ---------------------------------------------------------------------------

static void endStream(const char* reason) {
  if (outFb) { cameraRelease(outFb); outFb = NULL; }
  streaming = false;
  streamClient.stop();
  Serial.printf("[browser] stream ended (%s) after %u frames\n",
                reason, streamFrames);
}

static int beginStream(WiFiClient c) {
  c.setNoDelay(true);
  c.write("HTTP/1.1 200 OK\r\n");
  c.write("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  c.write("Cache-Control: no-cache, no-store, max-age=0\r\n");
  c.write("Pragma: no-cache\r\n\r\n");

  streamClient = c;
  streaming = true;
  streamFrames = 0;
  liveFailures = 0;
  outFb = NULL; outOff = 0;
  outHdrLen = outHdrPos = 0;
  return 0;
}

static void streamTick() {
  if (!streaming) return;
  if (!streamClient.connected()) { endStream("client left"); return; }

  browserHandleSerialQ();                    // allow serial 'q' mid-stream

  // ---- If no frame is mid-send, capture the next one (bounded) ------------
  if (!outFb) {
    if (!fpsGate()) return;

    camera_fb_t* fb = cameraCapture();
    if (!fb) {
      // Camera temporarily empty; recover if it stays broken for a while.
      if (++liveFailures > 100) {
        Serial.print("[browser] capture failing - re-initing camera\n");
        cameraReinit(current);
      }
      if (liveFailures < 20) delay(2);       // brief backoff when idle
      return;
    }
    liveFailures = 0;

    // Never shove a giant frame at a client - auto-downscale instead.
    if (fb->len > BROWSER_MAX_FRAME_BYTES) {
      cameraRelease(fb);
      if (current > QUALITY_LOW) {
        Serial.printf("[browser] frame %u > %u B - downscaling\n",
                      (unsigned)fb->len, (unsigned)BROWSER_MAX_FRAME_BYTES);
        setQuality((QualityLevel)(current - 1));
      }
      return;
    }

    outFb = fb;
    outOff = 0;
    outHdrLen = snprintf(outHdr, sizeof(outHdr),
              "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
              (unsigned)fb->len);
    outHdrPos = 0;
  }

  // ---- Push a bounded slice of the current frame --------------------------
  uint32_t pushed = 0;
  while (pushed < SEND_SLICE_BYTES) {
    if (!streamClient.connected()) { endStream("client left"); return; }

    if (outHdrPos < outHdrLen) {
      size_t n = streamClient.write((uint8_t*)outHdr + outHdrPos,
                                    (size_t)(outHdrLen - outHdrPos));
      if (n == 0) return;                    // stack buffer full - try next poll
      outHdrPos += (int)n;
      pushed += (uint32_t)n;
      continue;
    }

    if (outOff < outFb->len) {
      size_t chunk = (size_t)(outFb->len - outOff);
      if (chunk > SEND_SLICE_BYTES) chunk = SEND_SLICE_BYTES;
      size_t n = streamClient.write((uint8_t*)outFb->buf + outOff, chunk);
      if (n == 0) return;                    // stack buffer full - resume later
      outOff += n;
      pushed += (uint32_t)n;
      continue;
    }

    // Frame fully delivered: closing CRLF + release the fb.
    streamClient.write((uint8_t*)"\r\n", 1);
    cameraRelease(outFb);
    outFb = NULL;
    streamFrames++;
    liveFailures = 0;
    return;                                  // done with this frame
  }
}

static void serveIndex(WiFiClient c) {
  c.write("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  c.write((const uint8_t*)INDEX_HTML, sizeof(INDEX_HTML) - 1);
  c.stop();
}

static void serveQuality(WiFiClient c, const char* query) {
  // query looks like "level=2" (we already stripped the '?').
  int lv = -1;
  // tiny manual parse
  const char* p = query;
  while (p && *p) {
    if (*p == 'l' && strncmp(p, "level=", 6) == 0) {
      lv = (int)(p[6] - '0');
      break;
    }
    p++;
  }
  if (lv < 0) lv = (int)QUALITY_DEFAULT;
  setQuality((QualityLevel)lv);
  char body[64];
  snprintf(body, sizeof(body), "quality set to %d\n", lv);
  c.write("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n");
  c.write((const uint8_t*)body, strlen(body));
  c.stop();
}

void browserHandleSerialQ() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'q') {
      // parse "q0"/"q1"/"q2"
      delay(5);
      if (Serial.available()) {
        char d = Serial.read();
        int lv = d - '0';
        if (lv >= QUALITY_LOW && lv <= QUALITY_HIGH) setQuality((QualityLevel)lv);
      }
    }
  }
}

void browserPoll() {
  browserHandleSerialQ();

  // 1) Pump the active stream (one frame if due). Returns quickly so the rest
  //    of the loop can service other connections (notably /quality) concurrently.
  if (streaming) streamTick();

  // 2) Accept and dispatch any pending connection (page, /quality, or a second
  //    /stream which we reject while one stream is already running).
  WiFiClient c = srv.available();
  if (!c) return;

  String line = c.readStringUntil('\n');
  line.trim();

  if (line.startsWith("GET /stream")) {
    if (streaming) c.stop();                  // only one streamer at a time
    else beginStream(c);
  } else if (line.startsWith("GET /quality?")) {
    int q = line.indexOf('?');
    serveQuality(c, q >= 0 ? line.substring(q + 1).c_str() : (char*)"");
  } else {
    serveIndex(c);
  }
}