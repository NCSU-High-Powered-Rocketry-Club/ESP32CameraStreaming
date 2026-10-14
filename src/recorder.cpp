// recorder.cpp - microSD MJPEG (.avi) recorder, merged from the standalone
// "camera" AVI-recorder sketch into the ESP-NOW sender.
//
// A real AVI (Motion-JPEG) container is written so the clip plays in VLC,
// ffmpeg, phones, ... - see the layout comment above AviWriter. The ESP-NOW
// sender captures ONE frame per loop pass and this module appends that same
// JPEG to the card, so recording costs no extra capture time. Which consumer
// wins when the frame slot is too short for both is the RECORD_PRIORITY_*
// policy in stream_espnow.cpp.
#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include "config.h"
#include "recorder.h"

// ===========================================================================
//  JPEG trimming - used by BOTH consumers (the SD clip and the air stream), so
//  it lives outside the ENABLE_SD_RECORDING gate.
// ===========================================================================
// Length of one JPEG: up to and including the first EOI (FF D9). The OV3660 can
// hand back several concatenated JPEGs and/or pad past the end marker, so every
// frame is trimmed to keep it a single, self-contained image.
size_t recorderJpegLength(const uint8_t* buf, size_t len) {
  for (size_t i = 2; i + 1 < len; i++) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xD9) {
      return i + 2;                      // up to and including the EOI
    }
  }
  return len;
}

#if !ENABLE_SD_RECORDING
// ===========================================================================
//  Recorder disabled - STREAM_TARGET_BROWSER (phone/PC) build or the flag
//  turned off in config.h. config.h forces ENABLE_SD_RECORDING to 0 for every
//  target but the ESP-NOW one, so the browser path gains no SD code, no mount
//  and no recorder commands; these stubs just keep the module linkable.
// ===========================================================================
bool recorderBegin() { return false; }
bool recorderSdReady() { return false; }
bool recorderActive() { return false; }
bool recorderStart(uint16_t, uint16_t, uint16_t) { return false; }
bool recorderWriteFrame(const uint8_t*, size_t) { return false; }
void recorderNoteSkippedFrame() {}
bool recorderStop() { return false; }
void recorderTick() {}
void recorderSetMaxSeconds(uint32_t) {}
uint32_t recorderMaxSeconds() { return 0; }
void recorderSetPriority(uint8_t) {}
uint8_t recorderPriority() { return RECORD_PRIORITY_DEFAULT; }
const char* recorderPriorityName() { return "disabled"; }
uint16_t recorderFileIndex() { return 0; }
const char* recorderFileName() { return ""; }
uint32_t recorderFrameCount() { return 0; }
uint32_t recorderSecondsLeft() { return 0; }
void recorderStatusLine(char* out, size_t n) { snprintf(out, n, "disabled"); }

#else   // ---------------- the real microSD recorder -------------------------

// ===========================================================================
//  Minimal AVI (Motion-JPEG) writer
// ---------------------------------------------------------------------------
//  Layout of the 224 byte header written up front:
//    'RIFF' <size> 'AVI '
//      'LIST' <size> 'hdrl'
//        'avih' 56 <MainAVIHeader>
//        'LIST' <size> 'strl'
//          'strh' 56 <AVIStreamHeader>
//          'strf' 40 <BITMAPINFOHEADER>
//      'LIST' <size> 'movi'
//        '00dc' <len> <jpeg> [pad]   (repeated)
//    'idx1' <size> <index entries>   (written at the end)
// ===========================================================================
namespace {

struct IdxEntry {
  uint32_t offset;
  uint32_t size;
};

// Absolute file offsets of the fields that are patched once recording stops.
static const uint32_t AVI_HEADER_SIZE     = 224;
static const uint32_t OFF_RIFF_SIZE       = 4;
static const uint32_t OFF_MOVI_SIZE       = 216;
static const uint32_t OFF_AVIH_MICROSEC   = 32;
static const uint32_t OFF_AVIH_MAXBPS     = 36;
static const uint32_t OFF_AVIH_FRAMES     = 48;
static const uint32_t OFF_AVIH_SUGGESTED  = 60;
static const uint32_t OFF_STRH_LENGTH     = 140;
static const uint32_t OFF_STRH_SUGGESTED  = 144;
static const uint32_t MOVI_FOURCC_POS     = 220;

inline void putU16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
inline void putU32(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
inline void putFourCC(uint8_t* p, const char* c) { p[0] = c[0]; p[1] = c[1]; p[2] = c[2]; p[3] = c[3]; }

class AviWriter {
public:
  AviWriter() : _width(0), _height(0), _fps(1), _frameCount(0), _maxFrame(0),
                _moviBytes(0), _idx(nullptr), _idxCount(0), _idxCap(0), _totalSize(0) {}
  ~AviWriter() { if (_idx) free(_idx); }

  bool begin(const char* path, uint16_t width, uint16_t height, uint16_t fps) {
    _width = width;
    _height = height;
    _fps = fps ? fps : 1;
    _frameCount = 0;
    _maxFrame = 0;
    _moviBytes = 0;
    _idxCount = 0;

    _file = SD.open(path, FILE_WRITE);
    if (!_file) {
      return false;
    }
    uint8_t hdr[AVI_HEADER_SIZE];
    buildHeader(hdr);
    return _file.write(hdr, AVI_HEADER_SIZE) == AVI_HEADER_SIZE;
  }

  bool addFrame(const uint8_t* data, size_t len) {
    if (len == 0) {
      return true;                       // skip empty frames
    }
    uint32_t chunkPos = (uint32_t)_file.position();
    uint32_t offset = chunkPos - MOVI_FOURCC_POS;

    uint8_t head[8];
    putFourCC(head, "00dc");
    putU32(head + 4, (uint32_t)len);
    if (_file.write(head, 8) != 8) {
      return false;
    }
    if (_file.write(data, len) != len) {
      return false;
    }
    if (len & 1) {                       // chunks are word aligned
      uint8_t pad = 0;
      if (_file.write(&pad, 1) != 1) {
        return false;
      }
    }
    _moviBytes += 8 + (uint32_t)len + (uint32_t)(len & 1);
    if (len > _maxFrame) {
      _maxFrame = (uint32_t)len;
    }
    if (!pushIndex(offset, (uint32_t)len)) {
      return false;
    }
    _frameCount++;
    return true;
  }

  bool end() {
    // --- idx1 index ------------------------------------------------------
    uint32_t idxSize = _frameCount * 16;
    if (_file.write((const uint8_t*)"idx1", 4) != 4) {
      return false;
    }
    uint8_t tmp[4];
    putU32(tmp, idxSize);
    if (_file.write(tmp, 4) != 4) {
      return false;
    }
    for (uint32_t i = 0; i < _idxCount; i++) {
      uint8_t e[16];
      putFourCC(e, "00dc");
      putU32(e + 4, 0x10);              // AVIIF_KEYFRAME
      putU32(e + 8, _idx[i].offset);
      putU32(e + 12, _idx[i].size);
      if (_file.write(e, 16) != 16) {
        return false;
      }
    }

    uint32_t fileSize = (uint32_t)_file.position();
    _totalSize = fileSize;
    uint32_t microsec = 1000000UL / _fps;

    // --- patch the header with the real numbers --------------------------
    patchU32(OFF_RIFF_SIZE,      fileSize - 8);
    patchU32(OFF_MOVI_SIZE,      _moviBytes + 4);          // 'movi' + chunks
    patchU32(OFF_AVIH_MICROSEC,  microsec);
    patchU32(OFF_AVIH_MAXBPS,    _maxFrame * _fps);
    patchU32(OFF_AVIH_FRAMES,    _frameCount);
    patchU32(OFF_AVIH_SUGGESTED, _maxFrame);
    patchU32(OFF_STRH_LENGTH,    _frameCount);
    patchU32(OFF_STRH_SUGGESTED, _maxFrame);

    _file.flush();
    _file.close();
    return true;
  }

  // Size of the finished file (valid after end()).
  uint32_t totalSize() const { return _totalSize; }

private:
  void buildHeader(uint8_t* h) {
    memset(h, 0, AVI_HEADER_SIZE);

    putFourCC(h + 0,  "RIFF");
    putU32  (h + 4,   0);              // patched in end()
    putFourCC(h + 8,  "AVI ");
    putFourCC(h + 12, "LIST");
    putU32  (h + 16,  192);            // size of 'hdrl'
    putFourCC(h + 20, "hdrl");

    // avih (MainAVIHeader, 56 bytes) ---------------------------------------
    putFourCC(h + 24, "avih");
    putU32  (h + 28,  56);
    putU32  (h + 32,  1000000UL / _fps);   // dwMicroSecPerFrame
    putU32  (h + 36,  0);                  // dwMaxBytesPerSec (patched)
    putU32  (h + 40,  0);                  // dwPaddingGranularity
    putU32  (h + 44,  0x10);               // dwFlags: AVIF_HASINDEX
    putU32  (h + 48,  0);                  // dwTotalFrames (patched)
    putU32  (h + 52,  0);                  // dwInitialFrames
    putU32  (h + 56,  1);                  // dwStreams
    putU32  (h + 60,  0);                  // dwSuggestedBufferSize (patched)
    putU32  (h + 64,  _width);             // dwWidth
    putU32  (h + 68,  _height);            // dwHeight
    // h + 72 .. h + 87 : dwReserved[4] = 0

    // strl / strh / strf --------------------------------------------------
    putFourCC(h + 88,  "LIST");
    putU32  (h + 92,   116);               // size of 'strl'
    putFourCC(h + 96,  "strl");
    putFourCC(h + 100, "strh");
    putU32  (h + 104,  56);
    putFourCC(h + 108, "vids");            // fccType
    putFourCC(h + 112, "MJPG");            // fccHandler
    putU32  (h + 116,  0);                 // dwFlags
    putU16  (h + 120,  0);                 // wPriority
    putU16  (h + 122,  0);                 // wLanguage
    putU32  (h + 124,  0);                 // dwInitialFrames
    putU32  (h + 128,  1);                 // dwScale
    putU32  (h + 132,  _fps);              // dwRate
    putU32  (h + 136,  0);                 // dwStart
    putU32  (h + 140,  0);                 // dwLength (patched)
    putU32  (h + 144,  0);                 // dwSuggestedBufferSize (patched)
    putU32  (h + 148,  0xFFFFFFFF);        // dwQuality
    putU32  (h + 152,  0);                 // dwSampleSize
    putU16  (h + 156,  0);                 // rcFrame.left
    putU16  (h + 158,  0);                 // rcFrame.top
    putU16  (h + 160,  _width);            // rcFrame.right
    putU16  (h + 162,  _height);           // rcFrame.bottom

    putFourCC(h + 164, "strf");
    putU32  (h + 168,  40);
    putU32  (h + 172,  40);                // biSize (BITMAPINFOHEADER)
    putU32  (h + 176,  _width);            // biWidth
    putU32  (h + 180,  _height);           // biHeight
    putU16  (h + 184,  1);                 // biPlanes
    putU16  (h + 186,  24);                // biBitCount
    putFourCC(h + 188, "MJPG");            // biCompression
    putU32  (h + 192,  (uint32_t)_width * _height * 3);   // biSizeImage
    putU32  (h + 196,  0);                 // biXPelsPerMeter
    putU32  (h + 200,  0);                 // biYPelsPerMeter
    putU32  (h + 204,  0);                 // biClrUsed
    putU32  (h + 208,  0);                 // biClrImportant

    putFourCC(h + 212, "LIST");
    putU32  (h + 216,  0);                 // movi size (patched)
    putFourCC(h + 220, "movi");
  }

  void patchU32(uint32_t pos, uint32_t v) {
    uint8_t b[4];
    putU32(b, v);
    _file.seek(pos);
    _file.write(b, 4);
  }

  bool pushIndex(uint32_t offset, uint32_t size) {
    if (_idxCount == _idxCap) {
      uint32_t newCap = _idxCap ? _idxCap * 2 : 256;
      IdxEntry* p = (IdxEntry*)realloc(_idx, newCap * sizeof(IdxEntry));
      if (!p) {
        return false;
      }
      _idx = p;
      _idxCap = newCap;
    }
    _idx[_idxCount].offset = offset;
    _idx[_idxCount].size = size;
    _idxCount++;
    return true;
  }

  File      _file;
  uint16_t  _width;
  uint16_t  _height;
  uint16_t  _fps;
  uint32_t  _frameCount;
  uint32_t  _maxFrame;
  uint32_t  _moviBytes;
  IdxEntry* _idx;
  uint32_t  _idxCount;
  uint32_t  _idxCap;
  uint32_t  _totalSize;
};

}  // namespace

// ===========================================================================
//  Module state
// ===========================================================================
static bool      s_sdReady  = false;
static bool      s_active   = false;
static AviWriter s_avi;
static uint32_t  s_fileIndex = 0;
static char      s_fileName[40] = "";
static uint32_t  s_startMs   = 0;
static uint32_t  s_maxSecs   = RECORD_MAX_SECONDS;
static uint8_t   s_priority  = RECORD_PRIORITY_DEFAULT;
static uint16_t  s_aviFps    = RECORD_AVI_FPS;

// Per-clip statistics (mirrors the standalone sketch's RecStats, minus the
// capture timing which the streaming loop measures itself).
static uint32_t s_frames   = 0;
static uint32_t s_skipped  = 0;      // frames the priority policy kept away
static uint32_t s_failed   = 0;      // SD write failures
static uint64_t s_bytes    = 0;
static uint32_t s_minSize  = 0xFFFFFFFF;
static uint32_t s_maxSize  = 0;
static uint64_t s_writeUs  = 0;

// ---------------------------------------------------------------------------
//  microSD mount
// ---------------------------------------------------------------------------
bool recorderBegin() {
#if !ENABLE_SD_RECORDING
  return false;
#else
  if (s_sdReady) return true;

  Serial.printf("[rec] mounting microSD (CS=GPIO%d)...\n", SD_CS_PIN);

  // arduino-esp32 uses SCK=GPIO7, MISO=GPIO8, MOSI=GPIO9 by default; the card
  // slot on the Sense board only adds the CS line on GPIO21.
  if (!SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQUENCY)) {
    Serial.println("[rec]   mount failed at high speed - retrying at 4 MHz");
    SD.end();
    if (!SD.begin(SD_CS_PIN, SPI, 4000000)) {
      Serial.println("[rec]   SD mount FAILED - check the card (FAT32) and the "
                     "J3 pad; recording is disabled");
      return false;
    }
  }

  uint8_t type = SD.cardType();
  if (type == CARD_NONE) {
    Serial.println("[rec]   no card detected - recording is disabled");
    SD.end();
    return false;
  }
  const char* typeName = (type == CARD_MMC)   ? "MMC"
                       : (type == CARD_SD)    ? "SDSC"
                       : (type == CARD_SDHC)  ? "SDHC"
                                              : "UNKNOWN";
  Serial.printf("[rec]   mounted: %s, %llu MB total / %llu MB used\n",
                typeName, SD.cardSize() / (1024ULL * 1024ULL),
                SD.usedBytes() / (1024ULL * 1024ULL));
  s_sdReady = true;
  Serial.printf("[rec]   clip length %s, priority %s\n",
                s_maxSecs ? "limited by RECORD_MAX_SECONDS" : "until stopped",
                recorderPriorityName());
  return true;
#endif
}

bool recorderSdReady() { return s_sdReady; }
bool recorderActive()  { return s_active; }

// ---------------------------------------------------------------------------
//  Clip lifecycle
// ---------------------------------------------------------------------------
// /video00001.avi, /video00002.avi, ... - scanning for the first free index
// means clips from an earlier power cycle are never overwritten.
static uint32_t nextFreeIndex() {
  char name[40];
  for (uint32_t i = 1; i <= 99999; ++i) {
    snprintf(name, sizeof(name), "%s%05lu%s",
             RECORD_FILE_PREFIX, (unsigned long)i, RECORD_FILE_EXTENSION);
    if (!SD.exists(name)) return i;
  }
  return 0;
}

bool recorderStart(uint16_t width, uint16_t height, uint16_t fps) {
  if (s_active) {
    Serial.println("[rec] already recording - stop the clip first");
    return false;
  }
  if (!s_sdReady) {
    Serial.println("[rec] start refused: no SD card mounted");
    return false;
  }
  if (width == 0 || height == 0) {
    Serial.println("[rec] start refused: camera reported 0x0");
    return false;
  }

  uint32_t idx = nextFreeIndex();
  if (!idx) {
    Serial.printf("[rec] start refused: no free %sNNNNN%s name\n",
                  RECORD_FILE_PREFIX, RECORD_FILE_EXTENSION);
    return false;
  }
  snprintf(s_fileName, sizeof(s_fileName), "%s%05lu%s",
           RECORD_FILE_PREFIX, (unsigned long)idx, RECORD_FILE_EXTENSION);

  s_aviFps = fps ? fps : RECORD_AVI_FPS;
  if (!s_avi.begin(s_fileName, width, height, s_aviFps)) {
    Serial.printf("[rec] cannot create %s (card full / write protected?)\n", s_fileName);
    s_fileName[0] = '\0';
    return false;
  }

  s_active    = true;
  s_fileIndex = idx;
  s_startMs   = millis();
  s_frames = s_skipped = s_failed = 0;
  s_bytes = s_writeUs = 0;
  s_minSize = 0xFFFFFFFF;
  s_maxSize = 0;

  Serial.printf("[rec] === recording %s : %ux%u @ %ufps, %s ===\n",
                s_fileName, (unsigned)width, (unsigned)height, s_aviFps,
                s_maxSecs ? "duration-limited" : "until stopped");
  if (s_maxSecs) {
    Serial.printf("[rec]     auto-stop after %lu s (change it from the dashboard)\n",
                  (unsigned long)s_maxSecs);
  }
  return true;
}

bool recorderWriteFrame(const uint8_t* jpeg, size_t len) {
  if (!s_active || !jpeg) return false;

  size_t jlen = recorderJpegLength(jpeg, len);
  uint32_t t0 = micros();
  bool ok = s_avi.addFrame(jpeg, jlen);
  s_writeUs += (micros() - t0);

  if (!ok) {
    s_failed++;
    return false;
  }
  s_frames++;
  s_bytes += jlen;
  if (jlen < s_minSize) s_minSize = (uint32_t)jlen;
  if (jlen > s_maxSize) s_maxSize = (uint32_t)jlen;
  return true;
}

void recorderNoteSkippedFrame() { if (s_active) s_skipped++; }

void recorderTick() {
  if (!s_active) return;
  if (s_maxSecs && (millis() - s_startMs) >= s_maxSecs * 1000UL) {
    Serial.printf("[rec] clip length reached (%lu s) - stopping\n",
                  (unsigned long)s_maxSecs);
    recorderStop();
  }
}

bool recorderStop() {
  if (!s_active) {
    Serial.println("[rec] stop ignored: not recording");
    return false;
  }
  s_active = false;

  bool ok = s_avi.end();
  uint32_t elapsed = millis() - s_startMs;
  uint32_t totalSize = s_avi.totalSize();
  uint32_t frames = s_frames;

  if (!ok) {
    Serial.printf("[rec] !! finalising %s failed (card removed?)\n", s_fileName);
  }
  if (frames == 0) {
    Serial.printf("[rec] !! no frames were recorded into %s\n", s_fileName);
    s_fileName[0] = '\0';
    s_fileIndex = 0;
    return false;
  }

  const float avgKB   = s_bytes / 1024.0f / frames;
  const float minKB   = (s_minSize == 0xFFFFFFFF ? 0 : s_minSize) / 1024.0f;
  const float maxKB   = s_maxSize / 1024.0f;
  const float writeMs = s_writeUs / 1000.0f / frames;
  const float fps     = frames * 1000.0f / (elapsed ? elapsed : 1);
  const float kbps    = elapsed ? (totalSize / 1024.0f) * 1000.0f / elapsed : 0.0f;

  Serial.printf("[rec] === saved %s : %lu frames, %lu KB, %lu ms (%.1f fps) ===\n",
                s_fileName, (unsigned long)frames,
                (unsigned long)(totalSize / 1024), (unsigned long)elapsed, fps);
  Serial.printf("[rec]   frame size : avg %.1f KB, min %.1f KB, max %.1f KB\n",
                avgKB, minKB, maxKB);
  Serial.printf("[rec]   SD cost    : %.1f ms/frame, %.0f KB/s written\n",
                writeMs, kbps);
  if (s_skipped || s_failed) {
    Serial.printf("[rec]   note       : %lu frames kept off the card by the "
                  "priority policy, %lu SD write failures\n",
                  (unsigned long)s_skipped, (unsigned long)s_failed);
  }

  s_fileName[0] = '\0';
  s_fileIndex = 0;
  return true;
}

// ---------------------------------------------------------------------------
//  Settings / status
// ---------------------------------------------------------------------------
void recorderSetMaxSeconds(uint32_t sec) {
  if (sec > 3600) sec = 3600;            // sanity: this is a clip, not a DVR
  s_maxSecs = sec;
  if (sec) {
    Serial.printf("[rec] max clip length = %lu s\n", (unsigned long)sec);
  } else {
    Serial.println("[rec] max clip length = unlimited (record until stopped)");
  }
}

uint32_t recorderMaxSeconds() { return s_maxSecs; }

void recorderSetPriority(uint8_t priority) {
  if (priority > RECORD_PRIORITY_STREAM) priority = RECORD_PRIORITY_DEFAULT;
  s_priority = priority;
  Serial.printf("[rec] priority %u (%s)\n", (unsigned)priority, recorderPriorityName());
}

uint8_t recorderPriority() { return s_priority; }

const char* recorderPriorityName() {
  switch (s_priority) {
    case RECORD_PRIORITY_SD:     return "SD first";
    case RECORD_PRIORITY_STREAM: return "stream first";
    default:                     return "balanced";
  }
}

uint16_t recorderFileIndex() { return (uint16_t)s_fileIndex; }
const char* recorderFileName() { return s_fileName; }
uint32_t recorderFrameCount() { return s_frames; }

uint32_t recorderSecondsLeft() {
  if (!s_active || !s_maxSecs) return 0;
  uint32_t elapsed = (millis() - s_startMs) / 1000UL;
  return elapsed >= s_maxSecs ? 0 : s_maxSecs - elapsed;
}

void recorderStatusLine(char* out, size_t n) {
  // left = the countdown the dashboard shows: "-" when idle, "unlimited" for a
  // clip that runs until STOP, else the seconds remaining.
  char leftTxt[16];
  if (!s_active)        snprintf(leftTxt, sizeof(leftTxt), "-");
  else if (!s_maxSecs)  snprintf(leftTxt, sizeof(leftTxt), "unlimited");
  else                  snprintf(leftTxt, sizeof(leftTxt), "%lus",
                                 (unsigned long)recorderSecondsLeft());

  snprintf(out, n, "state=%s idx=%u frames=%lu left=%s prio=%u sd=%u",
           s_active ? "on" : "off", (unsigned)s_fileIndex,
           (unsigned long)(s_active ? s_frames : 0), leftTxt,
           (unsigned)s_priority, s_sdReady ? 1u : 0u);
}

#endif  // ENABLE_SD_RECORDING
