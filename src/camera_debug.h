// camera_debug.h - compile-time gated bring-up / camera diagnostics.
//
// EVERYTHING here is a no-op unless DEBUG_CAMERA >= 1 (see config.h). The point
// is to make a bad camera module obvious on the serial monitor instead of a
// silent black stream: a dead module does not answer on the SCCB bus, so
// esp_camera_init() fails with ESP_ERR_CAMERA_NOT_DETECTED (0x20001) or the
// sensor PID reads 0x00/0xFF. That is hardware, not firmware.
//
// Usage:
//   setup():  debugPrintBootInfo();  debugPrintPinMap();   // before camera init
//   camera bring-up (camera_hal.cpp) calls the init-result + sensor-probe hooks.
//   serial 'd':  debugDumpAll();
#pragma once

#include <Arduino.h>
#include "esp_camera.h"     // camera_fb_t, esp_err_t, sensor_t, ESP_ERR_CAMERA_*
#include "config.h"

// ---- one-shot boot / self-test dumps --------------------------------------
// Board + SDK + memory facts and the reset reason (a BROWNOUT/PANIC here can
// masquerade as "camera broken"). Call once, early in setup().
void debugPrintBootInfo();

// The GPIO -> camera pin map + XCLK, so an unseated ribbon or the wrong board
// revision/variant is obvious at a glance.
void debugPrintPinMap();

// Report + interpret an esp_camera_init()/re-init result code.
void debugPrintInitResult(const char* tag, esp_err_t err);

// THE camera self-test: reads the sensor's SCCB identity (PID/VER/MID + part
// name) and runs one capture probe. Distinguishes "bad module / bad ribbon"
// (no or bogus PID, no JPEG) from "bad firmware".
void debugPrintSensorProbe();

// Full dump: boot info + pin map + sensor probe. Backs the 'd' serial command.
void debugDumpAll();

// --- camera bus sweep (serial 't') ------------------------------------------
// Re-initialises the sensor across several XCLK frequencies and frame-buffer
// locations, trying a capture in each, and reports EVERY mode that actually
// produced a JPEG. This is the definitive test for a module whose SCCB ID reads
// fine but which never delivers pixels: if 0 modes yield a frame, the module's
// DATA path (sensor/ribbon/contacts) is dead and it must be reseated, cleaned
// or replaced. It ends by re-initialising the camera at QUALITY_DEFAULT.
void debugCameraBusSweep();

// ---- live-streaming helpers ------------------------------------------------
// Sanity-check one captured frame (must start with the JPEG SOI marker
// FF D8). Returns true if it looks like JPEG. Logs a WARNING for a bad frame
// at DEBUG_CAMERA >= 1 and a one-line summary per frame at DEBUG_CAMERA >= 2.
bool debugCheckFrame(const camera_fb_t* fb, const char* tag);

// Rate-limited "camera returned no frame" line. The FIRST failure also dumps
// the sensor probe, so the log captures the sensor identity while it is broken.
void debugNoteCaptureFailure(const char* tag);

// Number of failed captures seen since boot (0 when diagnostics are off).
uint32_t debugCaptureFailures();
