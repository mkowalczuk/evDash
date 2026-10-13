#pragma once

#ifdef BOARD_M5STACK_CORES3
#include "HWCDC.h"
#else
#include <HardwareSerial.h>
#endif // BOARD_M5STACK_CORES3
#include <FS.h>
#include <SD.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

typedef void (*LogSerialMirrorCallback)(const uint8_t *data, size_t size, void *context);

// DEBUG LEVEL (bitmask)
#define DEBUG_NONE 0          // 0 - no debug messages (quiet)
#define DEBUG_COMM (1 << 0)   // 1 - comm (BLE/CAN)
#define DEBUG_NET (1 << 1)    // 2 - network (wifi / cloud services: MQTT, API, ABRP transport)
#define DEBUG_GSM DEBUG_NET   // backwards compatibility alias
#define DEBUG_SDCARD (1 << 2) // 4 - sdcard
#define DEBUG_GPS (1 << 3)    // 8 - gps
#define DEBUG_ABRP (1 << 4)   // 16 - abrp telemetry
#define DEBUG_MODEM (1 << 5)  // 32 - cellular modem AT traffic (SIM7670G)
#define DEBUG_ALL 0xFF        // 255 - all debug messages

#ifdef BOARD_M5STACK_CORES3
class LogSerial : public HWCDC
#else
class LogSerial : public HardwareSerial
#endif // BOARD_M5STACK_CORES3
{
protected:
  uint8_t debugLevel;
  uint8_t lastChar = 0;
  uint8_t timestampMode = 0;                 // 0 off, 1 ms since boot, 2 wall clock
  time_t (*wallClockFn)() = nullptr;          // returns epoch secs, or 0 if unavailable
  bool logToSdcard = false;
  LogSerialMirrorCallback mirrorCallback = nullptr;
  void *mirrorContext = nullptr;
  File sdLogFile;
  char currentSdLogPath[64] = {0};
  // SD console log ring buffer
  uint8_t *sdBuffer = nullptr;
  size_t sdBufferSize = 0;
  volatile size_t sdBufferHead = 0;
  volatile size_t sdBufferTail = 0;
  volatile size_t sdBufferDropped = 0;
  uint32_t lastSdWriteMs = 0;
  uint32_t lastSdFlushMs = 0;
  portMUX_TYPE sdBufMux = portMUX_INITIALIZER_UNLOCKED;
  void pushSdBuffer(const uint8_t *buf, size_t len);
  void writeDirect(const uint8_t *buf, size_t len);
  void emitTimestampPrefix();
  fs::FS *sdFs = &SD; // filesystem for console-log file (SPI SD by default; SD_MMC on Waveshare)
public:
#ifndef BOARD_M5STACK_CORES3
  LogSerial();
#endif // BOARD_M5STACK_CORES3

  //
  void setDebugLevel(uint8_t aDebugLevel);
  uint8_t getDebugLevel() const { return debugLevel; }
  // Per-line timestamp prefix: 0 off, 1 ms since boot, 2 wall clock (falls back
  // to ms when no wall-clock source is set or time is not yet known).
  void setTimestampMode(uint8_t mode) { timestampMode = mode; }
  uint8_t getTimestampMode() const { return timestampMode; }
  void setWallClockSource(time_t (*fn)()) { wallClockFn = fn; }
  // Backing filesystem for the console-log file. Defaults to SPI SD; a board
  // with a different SD interface (e.g. Waveshare SD_MMC) sets this before
  // startSdLogging so the log is written to the correct card.
  void setSdFilesystem(fs::FS &fs) { sdFs = &fs; }
  bool isDebug(uint8_t aDebugLevel) const { return (debugLevel & aDebugLevel) != 0; }
  void setLogToSdcard(bool state);
  bool startSdLogging(const char *path);
  void stopSdLogging();
  bool isSdLogging() const { return logToSdcard; }
  const char *getSdLogPath() const { return currentSdLogPath; }
  size_t flushSdBuffer(size_t maxBytes = 1024, bool force = false);
  size_t getSdBufferUsed() const;
  size_t getSdBufferTotal() const { return sdBufferSize; }
  size_t getSdBufferDropped() const { return sdBufferDropped; }
  void setMirrorCallback(LogSerialMirrorCallback callback, void *context);
#ifdef BOARD_M5STACK_CORES3
  using HWCDC::write;
#else
  using HardwareSerial::write;
#endif // BOARD_M5STACK_CORES3
  size_t write(uint8_t data) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  void flush() override;

  // info
  template <class T, typename... Args>
  void info(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    println(msg);
  }
  template <class T, typename... Args>
  void infoNolf(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    print(msg);
  }
  // warning
  template <class T, typename... Args>
  void warn(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    print("WARN ");
    println(msg);
  }
  template <class T, typename... Args>
  void warnNolf(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    print("WARN ");
    print(msg);
  }

  // error
  template <class T, typename... Args>
  void err(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    print("ERR ");
    println(msg);
  }
  template <class T, typename... Args>
  void errNolf(uint8_t aDebugLevel, T msg)
  {
    if (aDebugLevel != DEBUG_NONE && (debugLevel & aDebugLevel) == 0)
      return;
    print("ERR ");
    print(msg);
  }
};
