// main.cpp - entry point for the ESP32-S3 Sense camera streamer.
//
// The receiver is chosen by the STREAM_TARGET flag in src/config.h:
//   STREAM_TARGET_BROWSER   -> HTTP MJPEG server (phone / browser)
//   STREAM_TARGET_ESP32C3   -> segmented ESP-NOW to the ESP32-C3 dish receiver
//
// Serial commands (both targets):
//   q0 / q1 / q2  -> switch quality LOW / MEDIUM / HIGH
//   s             -> print status
#include <Arduino.h>
#include "config.h"
#include "camera_hal.h"

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  #include "stream_browser.h"
#else
  #include "stream_espnow.h"
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
  Serial.println("----------------------------------------------\n");
}

void statusReport() {
  QualityProfile p = qualityProfileFor(QUALITY_DEFAULT);
  (void)p;
  Serial.printf("[main] quality default index = %d\n", QUALITY_DEFAULT);
  Serial.printf("[main] free heap = %u B\n", (unsigned)ESP.getFreeHeap());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  printBanner();

  if (!cameraBegin()) {
    Serial.println("[main] FATAL: camera did not initialise. Check the sensor.");
  }

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  browserBegin();
#else
  espnowBegin();
#endif

  statusReport();
}

void loop() {
  // Process a serial status query cheaply.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') statusReport();
  }

#if (STREAM_TARGET == STREAM_TARGET_BROWSER)
  browserPoll();          // serves /stream, /quality, /
#else
  espnowPoll();           // streams frames to the C3
#endif
}