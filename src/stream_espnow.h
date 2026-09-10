// stream_espnow.h - ESP-NOW target (ESP32-C3 "dish" receiver).
// Frames are JPEG, fragmented into <=240-byte datagrams, each carrying a small
// sequence header so the receiver can reassemble and safely discard stragglers.
#pragma once

// Bring up WiFi-AP + ESP-NOW peer and start streaming to the C3 (call once in setup).
void espnowBegin();

// Pump loop. When a frame needs sending this blocks until the frame is flushed,
// so treat this like the browser target's stream loop.
void espnowPoll();

// Runtime quality control from serial (shared with browser target).
void espnowHandleSerialQ();