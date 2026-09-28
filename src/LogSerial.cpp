/**
 * LogSerial class provides logging functionality.
 *
 * Allows setting debug level and logging to SD card.
 */
#include "LogSerial.h"

/**
 * Constructor
 */
#ifndef BOARD_M5STACK_CORES3
LogSerial::LogSerial() : HardwareSerial(0)
{
  // HardwareSerial::begin(115200);  // used syslog->begin(115200); (in evDash.ino)
}
#endif // BOARD_M5STACK_CORES3

/**
 * Set debug level
 */
void LogSerial::setDebugLevel(uint8_t aDebugLevel)
{
  debugLevel = aDebugLevel;
}

/**
 * Start dumping console log to an SD card file
 */
bool LogSerial::startSdLogging(const char *path)
{
  stopSdLogging();
  if (path == nullptr || path[0] == '\0')
  {
    return false;
  }
  sdLogFile = SD.open(path, FILE_APPEND);
  if (!sdLogFile)
  {
    sdLogFile = SD.open(path, FILE_WRITE);
  }
  if (!sdLogFile)
  {
    return false;
  }
  strncpy(currentSdLogPath, path, sizeof(currentSdLogPath) - 1);
  currentSdLogPath[sizeof(currentSdLogPath) - 1] = '\0';
  logToSdcard = true;
  return true;
}

/**
 * Stop dumping console log to SD card and close file
 */
void LogSerial::stopSdLogging()
{
  if (logToSdcard && sdLogFile)
  {
    sdLogFile.flush();
    sdLogFile.close();
  }
  logToSdcard = false;
  currentSdLogPath[0] = '\0';
}

/**
 * Write log to sdcard (legacy compatibility)
 */
void LogSerial::setLogToSdcard(bool state)
{
  if (!state)
  {
    stopSdLogging();
  }
}

/**
 * Mirror console bytes to an optional consumer.
 */
void LogSerial::setMirrorCallback(LogSerialMirrorCallback callback, void *context)
{
  mirrorCallback = callback;
  mirrorContext = context;
}

/**
 * Write one byte to console and mirror/log it when enabled.
 */
size_t LogSerial::write(uint8_t data)
{
#ifdef BOARD_M5STACK_CORES3
  size_t written = HWCDC::write(data);
#else
  size_t written = HardwareSerial::write(data);
#endif // BOARD_M5STACK_CORES3
  if (mirrorCallback != nullptr)
  {
    mirrorCallback(&data, 1, mirrorContext);
  }
  if (logToSdcard && sdLogFile)
  {
    size_t sdWritten = sdLogFile.write(data);
    if (sdWritten == 0)
    {
      stopSdLogging();
    }
    else if (data == '\n')
    {
      sdLogFile.flush();
    }
  }
  return written;
}

/**
 * Write bytes to console and mirror/log them when enabled.
 */
size_t LogSerial::write(const uint8_t *buffer, size_t size)
{
#ifdef BOARD_M5STACK_CORES3
  size_t written = HWCDC::write(buffer, size);
#else
  size_t written = HardwareSerial::write(buffer, size);
#endif // BOARD_M5STACK_CORES3
  if (mirrorCallback != nullptr && buffer != nullptr && size > 0)
  {
    mirrorCallback(buffer, size, mirrorContext);
  }
  if (logToSdcard && sdLogFile && buffer != nullptr && size > 0)
  {
    size_t sdWritten = sdLogFile.write(buffer, size);
    if (sdWritten == 0)
    {
      stopSdLogging();
    }
    else if (memchr(buffer, '\n', size) != nullptr)
    {
      sdLogFile.flush();
    }
  }
  return written;
}
