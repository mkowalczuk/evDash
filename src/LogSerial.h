#pragma once

#ifdef BOARD_M5STACK_CORES3
#include "HWCDC.h"
#else
#include <HardwareSerial.h>
#endif // BOARD_M5STACK_CORES3
#include <FS.h>
#include <SD.h>

typedef void (*LogSerialMirrorCallback)(const uint8_t *data, size_t size, void *context);

// DEBUG LEVEL (bitmask)
#define DEBUG_NONE 0          // 0 - no debug messages (quiet)
#define DEBUG_COMM (1 << 0)   // 1 - comm (BLE/CAN)
#define DEBUG_NET (1 << 1)    // 2 - network (wifi / gsm / cloud services)
#define DEBUG_GSM DEBUG_NET   // backwards compatibility alias
#define DEBUG_SDCARD (1 << 2) // 4 - sdcard
#define DEBUG_GPS (1 << 3)    // 8 - gps
#define DEBUG_ABRP (1 << 4)   // 16 - abrp telemetry
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
  bool logToSdcard = false;
  LogSerialMirrorCallback mirrorCallback = nullptr;
  void *mirrorContext = nullptr;
  File sdLogFile;
  char currentSdLogPath[64] = {0};

  void writeDirect(const uint8_t *buf, size_t len);

public:
#ifndef BOARD_M5STACK_CORES3
  LogSerial();
#endif // BOARD_M5STACK_CORES3

  //
  void setDebugLevel(uint8_t aDebugLevel);
  uint8_t getDebugLevel() const { return debugLevel; }
  bool isDebug(uint8_t aDebugLevel) const { return (debugLevel & aDebugLevel) != 0; }
  void setLogToSdcard(bool state);
  bool startSdLogging(const char *path);
  void stopSdLogging();
  bool isSdLogging() const { return logToSdcard; }
  const char *getSdLogPath() const { return currentSdLogPath; }
  void setMirrorCallback(LogSerialMirrorCallback callback, void *context);
#ifdef BOARD_M5STACK_CORES3
  using HWCDC::write;
#else
  using HardwareSerial::write;
#endif // BOARD_M5STACK_CORES3
  size_t write(uint8_t data) override;
  size_t write(const uint8_t *buffer, size_t size) override;

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
