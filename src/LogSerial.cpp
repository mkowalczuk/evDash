/**
 * LogSerial class provides logging functionality.
 *
 * Allows setting debug level and logging to SD card.
 */
#include "LogSerial.h"
#include <time.h>

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
  sdLogFile = sdFs->open(path, FILE_APPEND);
  if (!sdLogFile)
  {
    sdLogFile = sdFs->open(path, FILE_WRITE);
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
 * Rename the active SD console log file (e.g. once wall-clock time is known so a
 * log opened as "1970-..." gets a correct boot-time name). Flushes and closes
 * the current file, renames it on the backing filesystem, then reopens the new
 * path in append mode so subsequent output continues in the same file. The ring
 * buffer is preserved. No-op if not logging, the path is unchanged, or a file
 * already exists at the destination. Returns true on success.
 */
bool LogSerial::renameSdLog(const char *newPath)
{
  if (!logToSdcard || newPath == nullptr || newPath[0] == '\0')
  {
    return false;
  }
  if (strcmp(newPath, currentSdLogPath) == 0)
  {
    return false; // nothing to do
  }
  if (sdFs->exists(newPath))
  {
    return false; // don't clobber an existing file
  }

  // Flush everything buffered into the current file and close it so the rename
  // operates on a quiescent file.
  flushSdBuffer(0, true);
  if (sdLogFile)
  {
    sdLogFile.flush();
    sdLogFile.close();
  }

  if (!sdFs->rename(currentSdLogPath, newPath))
  {
    // Rename failed; reopen the original so logging continues uninterrupted.
    sdLogFile = sdFs->open(currentSdLogPath, FILE_APPEND);
    return false;
  }

  sdLogFile = sdFs->open(newPath, FILE_APPEND);
  if (!sdLogFile)
  {
    // Reopen failed entirely; stop SD logging to avoid writing to a closed file.
    logToSdcard = false;
    currentSdLogPath[0] = '\0';
    return false;
  }

  strncpy(currentSdLogPath, newPath, sizeof(currentSdLogPath) - 1);
  currentSdLogPath[sizeof(currentSdLogPath) - 1] = '\0';
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
 * Write raw chunk to hardware console, mirror callback, and in-memory SD ring buffer.
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
  if (logToSdcard)
  {
    pushSdBuffer(buf, len);
  }
}

/**
 * Emit the per-line timestamp prefix directly to the sink (never via write(),
 * to avoid recursing through newline-normalization and prefix logic). Format:
 *   mode 1: "[<ms>] "     (monotonic ms since boot)
 *   mode 2: "[HH:MM:SS] " (wall clock; falls back to ms when time unknown)
 */
void LogSerial::emitTimestampPrefix()
{
  if (timestampMode == 0)
  {
    return;
  }
  char buf[20];
  int len = 0;
  if (timestampMode == 2 && wallClockFn != nullptr)
  {
    time_t now = wallClockFn();
    if (now > 0)
    {
      struct tm tmv;
      localtime_r(&now, &tmv);
      len = snprintf(buf, sizeof(buf), "[%02d:%02d:%02d] ", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    }
  }
  if (len == 0)
  {
    // mode 1, or mode 2 with no usable wall clock yet: monotonic ms.
    len = snprintf(buf, sizeof(buf), "[%lu] ", static_cast<unsigned long>(millis()));
  }
  if (len > 0)
  {
    writeDirect(reinterpret_cast<const uint8_t *>(buf), static_cast<size_t>(len));
  }
}

/**
 * Write one byte to console, normalizing lone LF to CRLF for consistent console/SD logs.
 */
size_t LogSerial::write(uint8_t data)
{
  // Start-of-line timestamp: lastChar is '\n' after a completed line, or 0 at
  // boot. Do not prefix the newline byte itself.
  if (timestampMode != 0 && data != '\n' && data != '\r' &&
      (lastChar == '\n' || lastChar == 0))
  {
    emitTimestampPrefix();
  }
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

  // When timestamping is on, defer to the per-byte path so start-of-line
  // detection and prefixing happen uniformly. This is only taken while a debug
  // timestamp mode is active, so the normal (mode 0) bulk path is unchanged.
  if (timestampMode != 0)
  {
    for (size_t i = 0; i < size; i++)
    {
      write(buffer[i]);
    }
    return size;
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
