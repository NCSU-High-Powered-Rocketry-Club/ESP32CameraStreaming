// stream_browser.h - HTTP MJPEG target (phone / laptop web browser).
// Serves:
//   GET /                      -> HTML page embedding the live stream
//   GET /stream                -> multipart/x-mixed-replace MJPEG stream
//   GET /quality?level=0|1|2   -> runtime quality change
#pragma once

// Bring up the tiny HTTP server (call once in setup()).
void browserBegin();

// Pump the server. This is NON-BLOCKING: it time-slices the active MJPEG stream
// so the loop stays free to also answer quality changes (/quality) and open the
// page while a client is streaming. Call it from loop().
void browserPoll();

// Convenience for the serial quality command (q0/q1/q2). Called automatically
// by browserPoll() too, so this is only needed if you want it elsewhere.
void browserHandleSerialQ();