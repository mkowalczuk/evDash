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

// DEBUG LEVEL
#define DEBUG_NONE 0
#define DEBUG_COMM 1   // filter comm
#define DEBUG_GSM 2    // filter gsm messages
#define DEBUG_SDCARD 3 // filter sdcard
#define DEBUG_GPS 4    // filter gps

#ifdef BOARD_M5STACK_CORES3
class LogSerial : public HWCDC
#else
class LogSerial : public HardwareSerial
#endif // BOARD_M5STACK_CORES3
{
protected:
  uint8_t debugLevel;
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
public:
#ifndef BOARD_M5STACK_CORES3
  LogSerial();
#endif // BOARD_M5STACK_CORES3

  //
  void setDebugLevel(uint8_t aDebugLevel);
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
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    println(msg);
  }
  template <class T, typename... Args>
  void infoNolf(uint8_t aDebugLevel, T msg)
  {
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    print(msg);
  }
  // warning
  template <class T, typename... Args>
  void warn(uint8_t aDebugLevel, T msg)
  {
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    print("WARN ");
    println(msg);
  }
  template <class T, typename... Args>
  void warnNolf(uint8_t aDebugLevel, T msg)
  {
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    print("WARN ");
    print(msg);
  }

  // error
  template <class T, typename... Args>
  void err(uint8_t aDebugLevel, T msg)
  {
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    print("ERR ");
    println(msg);
  }
  template <class T, typename... Args>
  void errNolf(uint8_t aDebugLevel, T msg)
  {
    if (debugLevel != DEBUG_NONE && aDebugLevel != DEBUG_NONE && aDebugLevel != debugLevel)
      return;
    print("ERR ");
    print(msg);
  }
};
