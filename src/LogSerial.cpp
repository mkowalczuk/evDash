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

  // Allocate ring buffer (prefer PSRAM for a generous 64KB buffer, fallback to 16KB/8KB internal SRAM)
  if (sdBuffer == nullptr)
  {
    sdBufferSize = 64 * 1024;
    if (psramFound())
    {
      sdBuffer = (uint8_t *)heap_caps_malloc(sdBufferSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (sdBuffer == nullptr)
    {
      sdBufferSize = 16 * 1024;
      sdBuffer = (uint8_t *)heap_caps_malloc(sdBufferSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (sdBuffer == nullptr)
    {
      sdBufferSize = 8 * 1024;
      sdBuffer = (uint8_t *)malloc(sdBufferSize);
    }
    if (sdBuffer == nullptr)
    {
      sdBufferSize = 0;
      sdLogFile.close();
      return false;
    }
  }

  portENTER_CRITICAL(&sdBufMux);
  sdBufferHead = 0;
  sdBufferTail = 0;
  sdBufferDropped = 0;
  portEXIT_CRITICAL(&sdBufMux);

  lastSdWriteMs = millis();
  lastSdFlushMs = millis();

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
  if (logToSdcard)
  {
    flushSdBuffer(0, true);
    logToSdcard = false;
    currentSdLogPath[0] = '\0';
    if (sdLogFile)
    {
      sdLogFile.flush();
      sdLogFile.close();
    }
  }
  else
  {
    currentSdLogPath[0] = '\0';
  }

  portENTER_CRITICAL(&sdBufMux);
  if (sdBuffer != nullptr)
  {
    free(sdBuffer);
    sdBuffer = nullptr;
    sdBufferSize = 0;
    sdBufferHead = 0;
    sdBufferTail = 0;
    sdBufferDropped = 0;
  }
  portEXIT_CRITICAL(&sdBufMux);
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
 * Append incoming bytes to in-memory SD ring buffer.
 * Discards oldest bytes if buffer is full, never blocking the caller.
 */
void LogSerial::pushSdBuffer(const uint8_t *buf, size_t len)
{
  if (sdBuffer == nullptr || sdBufferSize == 0 || buf == nullptr || len == 0)
  {
    return;
  }

  portENTER_CRITICAL(&sdBufMux);

  const size_t count = (sdBufferHead >= sdBufferTail)
                           ? (sdBufferHead - sdBufferTail)
                           : (sdBufferSize - sdBufferTail + sdBufferHead);
  const size_t freeSpace = sdBufferSize - 1 - count;

  // If buffer doesn't have enough space for incoming data, discard oldest bytes
  if (len > freeSpace)
  {
    size_t overflow = len - freeSpace;
    sdBufferTail = (sdBufferTail + overflow) % sdBufferSize;
    sdBufferDropped += overflow;
  }

  // Copy into ring buffer (handling wrap-around)
  const size_t firstChunk = sdBufferSize - sdBufferHead;
  if (len <= firstChunk)
  {
    memcpy(sdBuffer + sdBufferHead, buf, len);
    sdBufferHead = (sdBufferHead + len) % sdBufferSize;
  }
  else
  {
    memcpy(sdBuffer + sdBufferHead, buf, firstChunk);
    memcpy(sdBuffer, buf + firstChunk, len - firstChunk);
    sdBufferHead = len - firstChunk;
  }

  portEXIT_CRITICAL(&sdBufMux);
}

/**
 * Flush buffered console log data from RAM ring buffer to SD card in non-blocking batches.
 */
size_t LogSerial::flushSdBuffer(size_t maxBytes, bool force)
{
  if (!logToSdcard || !sdLogFile || sdBuffer == nullptr || sdBufferSize == 0)
  {
    return 0;
  }

  size_t totalWritten = 0;

  do
  {
    size_t count = 0;
    portENTER_CRITICAL(&sdBufMux);
    count = (sdBufferHead >= sdBufferTail)
                ? (sdBufferHead - sdBufferTail)
                : (sdBufferSize - sdBufferTail + sdBufferHead);
    portEXIT_CRITICAL(&sdBufMux);

    if (count == 0)
    {
      if (force && sdLogFile)
      {
        sdLogFile.flush();
      }
      break;
    }

    const uint32_t nowMs = millis();

    // If not forced: only write if accumulated >= 1024 bytes, interval elapsed >= 500ms, or buffer > 75% full
    if (!force)
    {
      const bool timeToFlush = (nowMs - lastSdWriteMs) >= 500;
      const bool sizeToFlush = count >= 1024;
      const bool nearlyFull = count >= (sdBufferSize * 3 / 4);
      if (!timeToFlush && !sizeToFlush && !nearlyFull)
      {
        break;
      }
    }

    // Cap chunk size to avoid long SPI holds (default max 1024 bytes per chunk)
    size_t toWrite = count;
    if (maxBytes > 0 && toWrite > maxBytes)
    {
      toWrite = maxBytes;
    }

    uint8_t writeChunk[1024];
    if (toWrite > sizeof(writeChunk))
    {
      toWrite = sizeof(writeChunk);
    }

    size_t droppedCount = 0;
    portENTER_CRITICAL(&sdBufMux);
    if (sdBufferDropped > 0)
    {
      droppedCount = sdBufferDropped;
      sdBufferDropped = 0;
    }

    const size_t firstChunk = sdBufferSize - sdBufferTail;
    if (toWrite <= firstChunk)
    {
      memcpy(writeChunk, sdBuffer + sdBufferTail, toWrite);
      sdBufferTail = (sdBufferTail + toWrite) % sdBufferSize;
    }
    else
    {
      memcpy(writeChunk, sdBuffer + sdBufferTail, firstChunk);
      memcpy(writeChunk + firstChunk, sdBuffer, toWrite - firstChunk);
      sdBufferTail = toWrite - firstChunk;
    }
    portEXIT_CRITICAL(&sdBufMux);

    if (droppedCount > 0)
    {
      char dropNotice[48];
      int nlen = snprintf(dropNotice, sizeof(dropNotice), "\n[LOG: dropped %u bytes]\n", (unsigned int)droppedCount);
      if (nlen > 0)
      {
        sdLogFile.write((const uint8_t *)dropNotice, (size_t)nlen);
      }
    }

    size_t written = sdLogFile.write(writeChunk, toWrite);
    if (written == 0)
    {
      // Write error; stop logging to prevent repeated errors
      logToSdcard = false;
      currentSdLogPath[0] = '\0';
      if (sdLogFile)
      {
        sdLogFile.close();
      }
      return totalWritten;
    }

    totalWritten += written;
    lastSdWriteMs = nowMs;

    bool bufferEmpty = false;
    portENTER_CRITICAL(&sdBufMux);
    bufferEmpty = (sdBufferHead == sdBufferTail);
    portEXIT_CRITICAL(&sdBufMux);

    // Periodic physical commit to SD card FAT (every 3 seconds), or when forced, or when drained
    if (force || bufferEmpty || (nowMs - lastSdFlushMs) >= 3000)
    {
      sdLogFile.flush();
      lastSdFlushMs = nowMs;
    }

  } while (force);

  return totalWritten;
}

/**
 * Flush both serial hardware buffer and SD ring buffer.
 */
void LogSerial::flush()
{
#ifdef BOARD_M5STACK_CORES3
  HWCDC::flush();
#else
  HardwareSerial::flush();
#endif // BOARD_M5STACK_CORES3
  if (logToSdcard)
  {
    flushSdBuffer(0, true);
  }
}

/**
 * Get current number of bytes waiting in SD log ring buffer.
 */
size_t LogSerial::getSdBufferUsed() const
{
  if (sdBuffer == nullptr || sdBufferSize == 0)
  {
    return 0;
  }
  return (sdBufferHead >= sdBufferTail)
             ? (sdBufferHead - sdBufferTail)
             : (sdBufferSize - sdBufferTail + sdBufferHead);
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
  if (logToSdcard)
  {
    pushSdBuffer(&data, 1);
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
  if (logToSdcard && buffer != nullptr && size > 0)
  {
    pushSdBuffer(buffer, size);
  }
  return written;
}
