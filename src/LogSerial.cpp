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
 * Write raw chunk to hardware console, mirror callback, and SD log.
 */
void LogSerial::writeDirect(const uint8_t *buf, size_t len)
{
  if (buf == nullptr || len == 0)
  {
    return;
  }
#ifdef BOARD_M5STACK_CORES3
  HWCDC::write(buf, len);
#else
  HardwareSerial::write(buf, len);
#endif // BOARD_M5STACK_CORES3
  if (mirrorCallback != nullptr)
  {
    mirrorCallback(buf, len, mirrorContext);
  }
  if (logToSdcard && sdLogFile)
  {
    size_t sdWritten = sdLogFile.write(buf, len);
    if (sdWritten == 0)
    {
      stopSdLogging();
    }
    else if (memchr(buf, '\n', len) != nullptr)
    {
      sdLogFile.flush();
    }
  }
}

/**
 * Write one byte to console, normalizing lone LF to CRLF for consistent console/SD logs.
 */
size_t LogSerial::write(uint8_t data)
{
  if (data == '\n' && lastChar != '\r')
  {
    const uint8_t cr = '\r';
    writeDirect(&cr, 1);
  }
  writeDirect(&data, 1);
  lastChar = data;
  return 1;
}

/**
 * Write bytes to console, normalizing any lone LF to CRLF for consistent console/SD logs.
 */
size_t LogSerial::write(const uint8_t *buffer, size_t size)
{
  if (buffer == nullptr || size == 0)
  {
    return 0;
  }

  size_t start = 0;
  for (size_t i = 0; i < size; i++)
  {
    if (buffer[i] == '\n')
    {
      const bool needsCr = (i == 0) ? (lastChar != '\r') : (buffer[i - 1] != '\r');
      if (needsCr)
      {
        if (i > start)
        {
          writeDirect(buffer + start, i - start);
        }
        const uint8_t cr = '\r';
        writeDirect(&cr, 1);
        start = i;
      }
    }
  }

  if (start < size)
  {
    writeDirect(buffer + start, size - start);
  }

  lastChar = buffer[size - 1];
  return size;
}
