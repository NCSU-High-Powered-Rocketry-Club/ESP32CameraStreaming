// camera_pins.h - GPIO -> camera pin map for the Seeed XIAO ESP32-S3 Sense.
//
// Values verified against Seeed's XIAO_ESP32S3_Sense_camera wiki and the
// CameraWebServer camera_pins.h shipped in the arduino-esp32 SDK for this board.
// The block is keyed off CAMERA_MODEL_XIAO_ESP32S3 so it can be dropped into the
// classic esp32 CameraWebServer example unchanged if ever needed.
#pragma once

#if defined(CAMERA_MODEL_XIAO_ESP32S3)

#define PWDN_GPIO_NUM     -1          // camera has no power-down line (software reset used)
#define RESET_GPIO_NUM    -1          // software reset is performed by the driver
#define XCLK_GPIO_NUM     10          // camera clock
#define SIOD_GPIO_NUM     40          // SCCB (I2C) data  - CAM_SDA
#define SIOC_GPIO_NUM     39          // SCCB (I2C) clock - CAM_SCL

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      21          // on-board flash LED (only used if you enable it)

#endif // CAMERA_MODEL_XIAO_ESP32S3