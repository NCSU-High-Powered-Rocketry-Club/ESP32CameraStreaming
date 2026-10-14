// main.cpp - entry point for the ESP32-S3 Sense camera streamer.
//
// The receiver is chosen by the STREAM_TARGET flag in src/config.h:
//   STREAM_TARGET_BROWSER   -> HTTP MJPEG server (phone / browser)
//   STREAM_TARGET_ESP32C3   -> segmented ESP-NOW to the ESP32-C3 dish receiver
//
// Serial commands (both targets):
//   q0 / q1 / q2  -> switch quality LOW / MEDIUM / HIGH
//   s             -> print status
//   d             -> dump boot + camera diagnostics (see DEBUG_CAMERA in config.h)
//   t             -> camera bus sweep (try every XCLK/fb mode for a frame)
#include <Arduino.h>
#include "config.h"
#include "camera_hal.h"
#include "camera_debug.h"    // gated boot / camera diagnostics (see DEBUG_CAMERA)

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  #include "stream_browser.h"
#else
  #include "stream_espnow.h"
  #if ENABLE_SD_RECORDING
    #include "recorder.h"     // microSD AVI recorder (ESP-NOW target only)
  #endif
#endif

static void printBanner() {
  Serial.println("\n----------------------------------------------");
  Serial.println(" XIAO ESP32-S3 Sense - camera streamer");
  Serial.println("----------------------------------------------");
  Serial.print (" Target : ");
#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  Serial.println("BROWSER (phone/PC over HTTP MJPEG)");
#else
  Serial.println("ESP32-C3 (dish receiver over ESP-NOW)");
#endif
  Serial.println(" quality: q0/q1/q2 (LOW/MED/HIGH) | s = status");
#if (STREAM_TARGET == STREAM_TARGET_ESP32C3) && ENABLE_SD_RECORDING
  Serial.println(" SD rec : r = start, e = stop, D<secs> = max length, P<0..2> = priority");
#endif
#if DEBUG_CAMERA >= 1
  Serial.println(" debug  : d = dump diagnostics, t = camera bus sweep");
#endif
  Serial.println("----------------------------------------------\n");
}

void statusReport() {
  QualityProfile p = qualityProfileFor(QUALITY_DEFAULT);
  (void)p;
  Serial.printf("[main] quality default index = %d\n", QUALITY_DEFAULT);
  Serial.printf("[main] free heap = %u B\n", (unsigned)ESP.getFreeHeap());
#if (STREAM_TARGET == STREAM_TARGET_ESP32C3) && ENABLE_SD_RECORDING
  {
    char rec[96];
    recorderStatusLine(rec, sizeof(rec));
    Serial.printf("[main] REC %s\n", rec);
  }
#endif
#if DEBUG_CAMERA >= 1
  Serial.printf("[main] capture failures = %u\n", (unsigned)debugCaptureFailures());
#endif
}

void setup() {
  Serial.begin(115200);
  delay(300);
  printBanner();

#if DEBUG_CAMERA >= 1
  // Print board facts + the pin map BEFORE touching the camera, so an init
  // failure is read in context (a BROWNOUT reset here also mimics a dead cam).
  debugPrintBootInfo();
  debugPrintPinMap();
#endif

  if (!cameraBegin()) {
    Serial.println("[main] FATAL: camera did not initialise. Check the sensor.");
#if DEBUG_CAMERA >= 1
    Serial.println("[main] a bad module/ribbon reports ESP_ERR_CAMERA_NOT_DETECTED "
                   "(0x20001) - see the [dbg] lines above; try 'd' to re-dump.");
#endif
  }

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  browserBegin();
#else
  espnowBegin();
#endif

  statusReport();
}

void loop() {
  // Process a serial status/debug query cheaply.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') statusReport();
    if (c == 'd') debugDumpAll();     // full boot + camera diagnostic dump
    if (c == 't') debugCameraBusSweep();  // search for any mode that frames
#if (STREAM_TARGET == STREAM_TARGET_ESP32C3)
    // Quality (q0/q1/q2) and microSD recording control (r / e / D<secs> /
    // P<0..2>) on the ESP-NOW target. main.cpp is the only Serial reader, so
    // the stream module gets every char from here.
    espnowHandleSerialKey(c);
#endif
  }

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  browserPoll();          // serves /stream, /quality, /
#else
  espnowPoll();           // streams frames to the C3
#endif
}