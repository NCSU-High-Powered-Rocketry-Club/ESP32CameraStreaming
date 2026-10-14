// camera_debug.cpp - implementation of the gated diagnostics (see the header).
//
// Design notes:
//   * Every public function is a NO-OP when DEBUG_CAMERA is 0, so the flag can
//     be turned off for production without touching any caller.
//   * The capture probe uses esp_camera_fb_get() DIRECTLY (not cameraCapture())
//     so that probing after a capture failure cannot recurse back into
//     debugNoteCaptureFailure().
#include <Arduino.h>
#include <esp_system.h>          // esp_reset_reason(), esp_chip_info()
#include "esp_camera.h"          // sensor_t, camera_fb_t, ESP_ERR_CAMERA_*
#include "camera_hal.h"          // qualityProfileFor / cameraConfigFor / cameraReinit
#include "camera_debug.h"
#include "camera_pins.h"         // the GPIO macros (also pulls in config.h)

// ---------------------------------------------------------------------------
// Small helpers (only needed when diagnostics are compiled in)
// ---------------------------------------------------------------------------
#if DEBUG_CAMERA >= 1
static const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (normal power-up)";
    case ESP_RST_EXT:       return "EXT (reset pin)";
    case ESP_RST_SW:        return "SW (software / esp_restart)";
    case ESP_RST_PANIC:     return "PANIC (firmware crash - check the backtrace)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP wake";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (power dip - check USB cable/regulator)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// Map a sensor SCCB product ID to a human name. Values are the *_PID enum from
// the bundled esp32-camera sensor.h (see camera_pid_t).
static const char* sensorNameForPid(uint16_t pid) {
  switch (pid) {
    case OV2640_PID:  return "OV2640";
    case OV3660_PID:  return "OV3660";
    case OV5640_PID:  return "OV5640";
    case OV7670_PID:  return "OV7670";
    case OV7725_PID:  return "OV7725";
    case OV9650_PID:  return "OV9650";
    case NT99141_PID: return "NT99141";
    case GC2145_PID:  return "GC2145";
    case GC032A_PID:  return "GC032A";
    case GC0308_PID:  return "GC0308";
    case BF3005_PID:  return "BF3005";
    case BF20A6_PID:  return "BF20A6";
    case SC101IOT_PID:return "SC101IOT";
    case SC030IOT_PID:return "SC030IOT";
    case SC031GS_PID: return "SC031GS";
    default:          return "UNKNOWN/unsupported";
  }
}
#endif // DEBUG_CAMERA >= 1

// ---------------------------------------------------------------------------
// Boot information: reset reason + chip / flash / memory facts
// ---------------------------------------------------------------------------
void debugPrintBootInfo() {
#if DEBUG_CAMERA >= 1
  esp_chip_info_t chip;
  esp_chip_info(&chip);

  Serial.println("[dbg] ======== boot info ========");
  Serial.printf("[dbg] reset reason : %s\n", resetReasonName(esp_reset_reason()));
  Serial.printf("[dbg] chip         : %s rev %u, %u core(s)\n",
                ESP.getChipModel(), (unsigned)ESP.getChipRevision(),
                (unsigned)ESP.getChipCores());
  Serial.printf("[dbg] cpu freq     : %u MHz\n", (unsigned)ESP.getCpuFreqMHz());
  Serial.printf("[dbg] flash size   : %u KB\n", (unsigned)(ESP.getFlashChipSize() / 1024));
  Serial.printf("[dbg] sketch size  : %u KB\n", (unsigned)(ESP.getSketchSize() / 1024));
  Serial.printf("[dbg] heap         : free %u B, min %u B, total %u B\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                (unsigned)ESP.getHeapSize());
  Serial.printf("[dbg] PSRAM        : %s, total %u KB, free %u KB, min %u KB\n",
                psramFound() ? "found" : "NOT FOUND (camera fb_location PSRAM will fail!)",
                (unsigned)(ESP.getPsramSize() / 1024),
                (unsigned)(ESP.getFreePsram() / 1024),
                (unsigned)(ESP.getMinFreePsram() / 1024));
  uint64_t mac = ESP.getEfuseMac();
  Serial.printf("[dbg] efuse MAC    : %02X:%02X:%02X:%02X:%02X:%02X\n",
                (unsigned)((mac >> 0) & 0xFF), (unsigned)((mac >> 8) & 0xFF),
                (unsigned)((mac >> 16) & 0xFF), (unsigned)((mac >> 24) & 0xFF),
                (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  Serial.printf("[dbg] arduino core : %d.%d.%d, ESP-IDF SDK %s\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH, ESP.getSdkVersion());
  Serial.printf("[dbg] DEBUG_CAMERA : %d  (0=off, 1=boot+self-test, 2=also per-frame)\n",
                (int)DEBUG_CAMERA);
  Serial.println("[dbg] ==========================");
#endif
}

// ---------------------------------------------------------------------------
// The GPIO -> camera pin map, so a bad/loose ribbon or wrong variant is obvious
// ---------------------------------------------------------------------------
void debugPrintPinMap() {
#if DEBUG_CAMERA >= 1
  Serial.println("[dbg] ---- camera pin map (XIAO ESP32-S3 Sense) ----");
  Serial.printf("[dbg] XCLK=%d  SIOD/SDA=%d  SIOC/SCL=%d  PWDN=%d  RESET=%d\n",
                XCLK_GPIO_NUM, SIOD_GPIO_NUM, SIOC_GPIO_NUM,
                PWDN_GPIO_NUM, RESET_GPIO_NUM);
  Serial.printf("[dbg] data Y9..Y2 = %d %d %d %d %d %d %d %d\n",
                Y9_GPIO_NUM, Y8_GPIO_NUM, Y7_GPIO_NUM, Y6_GPIO_NUM,
                Y5_GPIO_NUM, Y4_GPIO_NUM, Y3_GPIO_NUM, Y2_GPIO_NUM);
  Serial.printf("[dbg] VSYNC=%d  HREF=%d  PCLK=%d\n",
                VSYNC_GPIO_NUM, HREF_GPIO_NUM, PCLK_GPIO_NUM);
  Serial.println("[dbg] ------------------------------------------------");
#endif
}

// ---------------------------------------------------------------------------
// esp_camera_init() result + what it means
// ---------------------------------------------------------------------------
void debugPrintInitResult(const char* tag, esp_err_t err) {
#if DEBUG_CAMERA >= 1
  Serial.printf("[dbg][%s] esp_camera_init -> %s (0x%x)\n",
                tag, (err == ESP_OK) ? "ESP_OK" : "FAILED", (unsigned)err);
  switch (err) {
    case ESP_OK:
      break;
    case ESP_ERR_CAMERA_NOT_DETECTED:
      Serial.println("[dbg] *** NOT_DETECTED (0x20001): the driver could not talk to "
                     "the sensor over SCCB.");
      Serial.println("[dbg] *** #1 cause is the camera module / ribbon, not firmware - "
                     "reseat or swap the camera.");
      break;
    case ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE:
      Serial.println("[dbg] *** FAILED_TO_SET_FRAME_SIZE (0x20002): the sensor answered "
                     "but rejected the resolution.");
      break;
    case ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT:
      Serial.println("[dbg] *** FAILED_TO_SET_OUT_FORMAT (0x20003): bad pixel format/quality.");
      break;
    default:
      Serial.printf("[dbg] *** init failed (0x%x). 0x20001=not detected, "
                    "0x101=invalid arg, 0x105=no mem.\n", (unsigned)err);
      break;
  }
#else
  (void)tag; (void)err;
#endif
}

// ---------------------------------------------------------------------------
// The camera self-test: SCCB identity + one live capture probe
// ---------------------------------------------------------------------------
void debugPrintSensorProbe() {
#if DEBUG_CAMERA >= 1
  Serial.println("[dbg] ---- camera self-test ----");
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("[dbg] sensor      : NULL - driver is not initialised "
                   "(init failed above; nothing on the SCCB bus)");
  } else {
    uint16_t pid = s->id.PID;
    Serial.printf("[dbg] sensor PID  : 0x%04X  (%s)\n", pid, sensorNameForPid(pid));
    Serial.printf("[dbg] sensor MID  : %02X %02X   VER 0x%02X   SCCB addr 0x%02X\n",
                  s->id.MIDH, s->id.MIDL, s->id.VER, s->slv_addr);
    camera_sensor_info_t* info = esp_camera_sensor_get_info(&s->id);
    Serial.printf("[dbg] sensor table: %s\n",
                  info ? info->name : "(no match - unknown/unsupported sensor)");
    Serial.printf("[dbg] framesize   : %d   quality %d\n",
                  (int)s->status.framesize, (int)s->status.quality);
    if (pid == 0x0000 || pid == 0xFFFF) {
      Serial.println("[dbg] *** PID reads 0x00/0xFF: NOTHING is answering on SCCB.");
      Serial.println("[dbg] *** That is a bad camera module / ribbon (or missing "
                     "XCLK), not firmware - swapping the module fixes it.");
    } else if (sensorNameForPid(pid)[0] == 'U' && s->id.VER == 0) {
      Serial.println("[dbg] *** Unknown PID with VER=0: sensor may be damaged or "
                     "the ribbon is partially seated.");
    }
  }

  // One live capture probe. Use the raw driver call so a failure cannot recurse
  // into debugNoteCaptureFailure().
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[dbg] capture     : FAILED - no frame returned "
                   "(no VSYNC/PCLK from the sensor, or DMA starved).");
  } else {
    bool jpeg = (fb->len >= 2) && (fb->buf[0] == 0xFF) && (fb->buf[1] == 0xD8);
    Serial.printf("[dbg] capture     : %ux%u  %u B  fmt=%d  SOI=%s  t=%u.%06u\n",
                  (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len,
                  (int)fb->format, jpeg ? "OK (FF D8)" : "MISSING (not JPEG!)",
                  (unsigned)fb->timestamp.tv_sec, (unsigned)fb->timestamp.tv_usec);
    esp_camera_fb_return(fb);
  }
  Serial.println("[dbg] --------------------------");
#endif
}

// ---------------------------------------------------------------------------
// Full on-demand dump (serial 'd') and the boot-time / capture-failure hooks
// ---------------------------------------------------------------------------
void debugDumpAll() {
#if DEBUG_CAMERA >= 1
  debugPrintBootInfo();
  debugPrintPinMap();
  debugPrintSensorProbe();
#else
  Serial.println("[dbg] DEBUG_CAMERA is 0 in config.h - set it to 1 (or 2) and "
                 "reflash to enable boot + camera diagnostics.");
#endif
}

static uint32_t g_captureFailures = 0;

bool debugCheckFrame(const camera_fb_t* fb, const char* tag) {
  if (!fb) return false;
  bool jpeg = (fb->len >= 2) && (fb->buf[0] == 0xFF) && (fb->buf[1] == 0xD8);
#if DEBUG_CAMERA >= 2
  Serial.printf("[dbg][%s] frame %ux%u %u B fmt=%d %s\n", tag,
                (unsigned)fb->width, (unsigned)fb->height, (unsigned)fb->len,
                (int)fb->format, jpeg ? "SOI ok" : "BAD (no FF D8)");
#elif DEBUG_CAMERA >= 1
  if (!jpeg) {
    Serial.printf("[dbg][%s] WARNING: frame is not JPEG (no FF D8): len=%u fmt=%d\n",
                  tag, (unsigned)fb->len, (int)fb->format);
  }
#else
  (void)tag;
#endif
  return jpeg;
}

void debugNoteCaptureFailure(const char* tag) {
  g_captureFailures++;
#if DEBUG_CAMERA >= 1
  // Rate-limit so a dead camera does not flood the console...
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs >= 2000) {
    lastMs = now;
    Serial.printf("[dbg][%s] capture returned no frame (%u total). Possible causes: "
                  "bad camera module/ribbon, unseated connector, or brownout.\n",
                  tag, (unsigned)g_captureFailures);
    // ...but on the FIRST failure, dump the sensor identity while it is broken.
    if (g_captureFailures == 1) debugPrintSensorProbe();
  }
#else
  (void)tag;
#endif
}

uint32_t debugCaptureFailures() { return g_captureFailures; }

// ---------------------------------------------------------------------------
// Camera bus sweep (serial 't'): try every plausible timing/memory combo and
// report which (if any) delivers a JPEG. Definitive dead-module test.
// ---------------------------------------------------------------------------
void debugCameraBusSweep() {
#if DEBUG_CAMERA >= 1
  static const int xclks[] = { 20000000, 16000000, 10000000 };  // 24MHz dropped (near OV3660 max)
  static const camera_fb_location_t locs[] = { CAMERA_FB_IN_PSRAM, CAMERA_FB_IN_DRAM };
  const int nX = (int)(sizeof(xclks) / sizeof(xclks[0]));
  const int nL = (int)(sizeof(locs) / sizeof(locs[0]));

  Serial.println("[dbg] ---- camera bus sweep (search for ANY mode that yields a frame) ----");
  Serial.println("[dbg] (note: a module whose SCCB ID reads fine but never frames has a dead DATA path)");

  int works = 0, modes = 0;
  for (int xi = 0; xi < nX; ++xi) {
    for (int li = 0; li < nL; ++li) {
      modes++;
      QualityProfile p = qualityProfileFor(QUALITY_LOW);
      camera_config_t cfg = cameraConfigFor(p);
      cfg.xclk_freq_hz = xclks[xi];   // override the normal CAMERA_XCLK_HZ
      cfg.fb_location  = locs[li];    // PSRAM vs internal DRAM
      cfg.fb_count     = 1;
      const char* loc  = (locs[li] == CAMERA_FB_IN_PSRAM) ? "PSRAM" : "DRAM ";

      esp_camera_deinit();
      delay(50);
      esp_err_t err = esp_camera_init(&cfg);
      if (err != ESP_OK) {
        Serial.printf("[dbg] xclk=%2dMHz fb=%s -> init FAILED (0x%x)\n",
                      xclks[xi] / 1000000, loc, (unsigned)err);
        continue;
      }

      delay(200);                     // let the sensor settle after init
      bool got = false;
      size_t bytes = 0;
      // A real QVGA JPEG is several KB; anything smaller is a truncated / stale
      // DMA buffer, NOT a usable frame. Requiring a plausible size avoids a
      // false "WORKS!" from a leftover buffer that merely starts with FF D8.
      const size_t MIN_FRAME_BYTES = 512;
      for (int a = 0; a < 3 && !got; ++a) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
          if (fb->len >= MIN_FRAME_BYTES && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
            got = true;
          }
          if (fb->len > bytes) bytes = fb->len;  // remember the largest seen
          esp_camera_fb_return(fb);
        }
        if (!got) delay(80);
      }

      if (got) {
        works++;
        Serial.printf("[dbg] xclk=%2dMHz fb=%s -> init OK, capture OK (%u B, FF D8)  <== WORKS\n",
                      xclks[xi] / 1000000, loc, (unsigned)bytes);
      } else {
        Serial.printf("[dbg] xclk=%2dMHz fb=%s -> init OK, capture FAILED "
                      "(best %u B - no usable frame)\n",
                      xclks[xi] / 1000000, loc, (unsigned)bytes);
      }
    }
  }

  Serial.printf("[dbg] RESULT: %d/%d mode(s) produced a JPEG.\n", works, modes);
  if (works == 0) {
    Serial.println("[dbg] *** SCCB identity is fine but NO mode frames: the camera");
    Serial.println("[dbg] *** DATA path is dead (module/ribbon/contacts) - reseat +");
    Serial.println("[dbg] *** clean the flex, then replace the module if it persists.");
  } else {
    Serial.println("[dbg] A mode above works - set CAMERA_XCLK_HZ in config.h to that");
    Serial.println("[dbg] value (16 MHz also enables EDMA on the S3) to use this module.");
  }

  cameraReinit(QUALITY_DEFAULT);      // leave the camera usable afterwards
  Serial.println("[dbg] ---------------------------------------------------------------");
#else
  Serial.println("[dbg] DEBUG_CAMERA is 0 in config.h - set it to 1 to use the bus sweep.");
#endif
}

