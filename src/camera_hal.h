// camera_hal.h - thin, dependency-free wrapper around the esp_camera driver that
// is bundled in the arduino-esp32 core. Provides: camera bring-up, frame capture,
// runtime quality switching and a small adaptive-quality state machine.
#pragma once

#include "esp_camera.h"      // bundled with the arduino-esp32 SDK (no lib_deps)
#include "config.h"

// --- Quality profile -------------------------------------------------------
typedef enum {
  QUALITY_LOW = 0,
  QUALITY_MEDIUM = 1,
  QUALITY_HIGH = 2
} QualityLevel;

typedef struct {
  QualityLevel level;
  const char* name;
  framesize_t frame_size;   // resolution; see Framesize_* in esp_camera
  int frame_width, frame_height;
  int jpeg_quality;         // 0..63, LOWER = higher fidelity
  float max_fps;            // target stream frame rate
  uint32_t max_frame_bytes; // ~typical compressed frame size for this preset
} QualityProfile;

// Fill the camera config struct for the given level WITHOUT initialising the
// driver. Useful to boot at a chosen level.
QualityProfile qualityProfileFor(QualityLevel level);

// Boot the camera driver with QUALITY_DEFAULT. Returns true on success.
bool cameraBegin();

// Switch the running camera to a new quality level (resolution + JPEG quality).
void cameraApplyQuality(QualityLevel level);

// Capture one JPEG frame. Returns the frame buffer (esp_camera owns it) or NULL
// on failure. You MUST call cameraRelease() with the returned buffer.
camera_fb_t* cameraCapture();

// Give a captured frame back to the driver.
void cameraRelease(camera_fb_t* fb);

// Re-initialise the camera driver at the given quality. Used to recover if the
// camera stops producing frames (e.g. after a long burst of large captures).
// Returns true on success.
bool cameraReinit(QualityLevel level);

// --- Adaptive quality (error tolerance) ------------------------------------
// Feed it per-frame / per-segment success/failure deltas while streaming and it
// will tell you (via adaptiveStep()) when the controller should change quality.
void   adaptiveReset();
void   adaptiveReport(bool ok);
// Returns the NEW suggested level, or -1 to keep the current one.
int    adaptiveStep(QualityLevel current);
bool   adaptiveActive();