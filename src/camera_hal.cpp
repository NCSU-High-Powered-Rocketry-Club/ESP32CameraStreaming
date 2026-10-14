// camera_hal.cpp - implementation of the quality/capture/adaptive engine.
#include "camera_hal.h"
#include "camera_pins.h"     // pin macros (PWDN_GPIO_NUM, SIOD_GPIO_NUM, ...)
#include "camera_debug.h"    // gated boot / camera diagnostics (see DEBUG_CAMERA)
#include <Arduino.h>

// ---- Profile table (order matters: LOW..HIGH = 0..2) -----------------------
QualityProfile qualityProfileFor(QualityLevel level) {
  static const QualityProfile table[3] = {
    { QUALITY_LOW,    "LOW",    FRAMESIZE_QVGA, 320, 240, 35, 15.0f, 16UL * 1024 },
    { QUALITY_MEDIUM, "MEDIUM", FRAMESIZE_VGA,  640, 480, 20, 10.0f, 48UL * 1024 },
    { QUALITY_HIGH,   "HIGH",   FRAMESIZE_UXGA, 1600,1200,12,  5.0f, 160UL * 1024},
  };
  int i = (level < 0) ? 0 : (level > QUALITY_HIGH ? QUALITY_HIGH : level);
  return table[i];
}

// Build a camera_config_t for a profile WITHOUT initialising. This lets us boot
// at QUALITY_DEFAULT and is a handy reference for manual setups.
camera_config_t cameraConfigFor(QualityProfile p) {
  camera_config_t c;
  c.pin_pwdn     = PWDN_GPIO_NUM;
  c.pin_reset    = RESET_GPIO_NUM;
  c.pin_xclk     = XCLK_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_d7       = Y9_GPIO_NUM;
  c.pin_d6       = Y8_GPIO_NUM;
  c.pin_d5       = Y7_GPIO_NUM;
  c.pin_d4       = Y6_GPIO_NUM;
  c.pin_d3       = Y5_GPIO_NUM;
  c.pin_d2       = Y4_GPIO_NUM;
  c.pin_d1       = Y3_GPIO_NUM;
  c.pin_d0       = Y2_GPIO_NUM;
  c.pin_vsync    = VSYNC_GPIO_NUM;
  c.pin_href     = HREF_GPIO_NUM;
  c.pin_pclk     = PCLK_GPIO_NUM;

  c.xclk_freq_hz = CAMERA_XCLK_HZ;    // sensor clock (see config.h)
  c.ledc_timer   = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;    // streaming
  c.frame_size   = p.frame_size;
  c.fb_location  = CAMERA_FB_LOCATION; // PSRAM (default) or internal DRAM
  c.jpeg_quality = p.jpeg_quality;
  c.fb_count     = 1;
  c.grab_mode    = CAMERA_GRAB_LATEST; // always most recent frame
  return c;
}

bool cameraBegin() {
  QualityProfile p = qualityProfileFor(QUALITY_DEFAULT);
  camera_config_t cfg = cameraConfigFor(p);
  esp_err_t err = esp_camera_init(&cfg);
  debugPrintInitResult("cam", err);
  if (err != ESP_OK) {
    Serial.println("[cam] init FAILED");
    return false;
  }
  cameraApplyQuality(p.level); // explicit, so HW is in a known state
  Serial.printf("[cam] %s @ %dx%d quality=%d\n",
                p.name, p.frame_width, p.frame_height, p.jpeg_quality);
  debugPrintSensorProbe();     // SCCB identity + one capture probe (DEBUG_CAMERA)
  return true;
}

void cameraApplyQuality(QualityLevel level) {
  QualityProfile p = qualityProfileFor(level);
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  s->set_framesize(s, p.frame_size);
  s->set_quality(s, p.jpeg_quality);
  adaptiveReset(); // the link characteristics change -> reset the histogram
  Serial.printf("[q] set %s (%dx%d, jpegq=%d)\n", p.name,
                p.frame_width, p.frame_height, p.jpeg_quality);
}

camera_fb_t* cameraCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {                       // driver produced nothing
    debugNoteCaptureFailure("capture");
    return nullptr;
  }
  if (fb->len == 0) {              // empty / invalid frame
    esp_camera_fb_return(fb);
    debugNoteCaptureFailure("capture");
    return nullptr;
  }
  debugCheckFrame(fb, "capture");  // warns on non-JPEG, per-frame at DEBUG_CAMERA>=2
  return fb;
}

void cameraRelease(camera_fb_t* fb) {
  if (fb) esp_camera_fb_return(fb);
}

bool cameraReinit(QualityLevel level) {
  if (level < QUALITY_LOW) level = QUALITY_LOW;
  if (level > QUALITY_HIGH) level = QUALITY_HIGH;
  QualityProfile p = qualityProfileFor(level);
  camera_config_t cfg = cameraConfigFor(p);
  esp_camera_deinit();
  delay(20);
  esp_err_t err = esp_camera_init(&cfg);
  debugPrintInitResult("cam", err);
  if (err != ESP_OK) {
    Serial.println("[cam] reinit FAILED");
    return false;
  }
  cameraApplyQuality(level);
  Serial.printf("[cam] reinit at %s\n", p.name);
  debugPrintSensorProbe();     // re-verify the sensor survived the reinit
  return true;
}

// ---------------------------------------------------------------------------
// Adaptive quality state machine (~sliding-window failure ratio).
// ---------------------------------------------------------------------------
#define AQ_N ADAPTIVE_WINDOW
static bool  aq_window[AQ_N];
static int   aq_head = 0, aq_count = 0;
static const float aq_down = ADAPTIVE_DOWN_THRESHOLD;
static const float aq_up   = ADAPTIVE_UP_THRESHOLD;

void adaptiveReset() {
  aq_head = aq_count = 0;
  // Start PESSIMISTIC: assume the worst until proven otherwise. Pre-filling the
  // window with "successes" made the up-stepper march straight to HIGH (UXGA)
  // right after every quality change - frames too big for the receiver and a
  // known crasher. Real link results fill the window within a few frames.
  for (int i = 0; i < ADAPTIVE_WINDOW; ++i) aq_window[i] = false;
}

static float adaptiveRatio() {
  if (aq_count == 0) return 1.0f;
  int fails = 0;
  for (int i = 0; i < aq_count; ++i) if (!aq_window[i]) ++fails;
  return (float)fails / (float)aq_count;
}

void adaptiveReport(bool ok) {
  if (!ADAPTIVE_QUALITY_ENABLED) return;
  aq_window[aq_head] = ok;
  aq_head = (aq_head + 1) % ADAPTIVE_WINDOW;
  if (aq_count < ADAPTIVE_WINDOW) aq_count++;
}

int adaptiveStep(QualityLevel current) {
  if (!ADAPTIVE_QUALITY_ENABLED || aq_count < 8) return -1;
  float r = adaptiveRatio();
  if (r >= aq_down && current > QUALITY_LOW) {
    adaptiveReset();
    return (int)current - 1;                     // step DOWN (lighter)
  }
  if (r <= aq_up && current < QUALITY_HIGH) {
    adaptiveReset();
    return (int)current + 1;                     // step UP (heavier)
  }
  return -1;
}

bool adaptiveActive() { return ADAPTIVE_QUALITY_ENABLED; }