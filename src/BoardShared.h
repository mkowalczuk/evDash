#pragma once

// Shared internal helpers, constants and file-scope state for Board320_240 and
// BoardCore.
//
// These were originally file-local to Board320_240.cpp. The mainLoop() body has
// moved to BoardCore and it depends on them, so they now live here and are
// included by both translation units. Each still gets its own copy because
// everything is either `static` or inside an anonymous namespace.
//
// Not a public interface - only Board320_240.cpp and BoardCore.cpp include this.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <esp_sntp.h>
#include <math.h>
#include "config.h"
#include "LiveData.h"
#include "EvDashMobileRelay.h"
#include "traccar.h"

extern EvDashMobileRelay *mobileRelay;

namespace
{
  volatile bool s_ntpSyncCompleted = false;

  void sntpTimeSyncNotificationCallback(struct timeval *tv)
  {
    s_ntpSyncCompleted = true;
  }

  constexpr uint32_t kNetRetryIntervalSec = 30;
  constexpr uint32_t kNtpPriorityWindowMs = 60000;
  constexpr uint32_t kNtpRetryIntervalMs = 5000;
  constexpr uint32_t kNetFailureStaleResetSec = 300;
  constexpr uint32_t kNetFailureFallbackSec = 180;
  constexpr uint16_t kNetFailureFallbackCount = 3;
  constexpr uint32_t kWifiTransferIndicatorWindowMs = 2000;
  constexpr size_t kAbrpPayloadBufferSize = 768;
  constexpr size_t kAbrpFormBufferSize = 1536;
  constexpr uint16_t kAbrpHttpsConnectTimeoutMs = 2500;
  constexpr uint16_t kAbrpHttpsIoTimeoutMs = 2500;
  constexpr uint32_t kTraccarIntervalMs = 5000;
  constexpr float kGpsMaxSpeedKmh = 250.0f;
  constexpr float kGpsJitterMeters = 200.0f;
  constexpr float kGpsMaxJumpMetersShort = 2000.0f;
  constexpr uint32_t kGpsShortJumpWindowSec = 5;
  constexpr uint32_t kGpsReacquireAfterSecDefault = 900;
  constexpr uint32_t kGpsReacquireAfterSecV21 = 120;
  constexpr uint32_t kGpsFixFreshnessSec = 15;
  constexpr float kGpsHeadingMinDistanceMeters = 3.0f;
  constexpr float kContributeGpsCoordPrecision = 1000000.0f;
  constexpr uint32_t kMotionWakeResetSec = 900;
  constexpr uint32_t kChargingQueueHoldSec = 180;
  constexpr time_t kLongParkingClearSec = 2 * 60 * 60;
  constexpr uint16_t kSentryIdleSliceMs = 50;
  constexpr uint8_t kGpsWakeConfirmSamples = 2;
  constexpr uint8_t kGyroWakeConfirmSamples = 3;
  constexpr size_t kContributeJsonDocCapacity = 12288;
  constexpr uint8_t kContributeRawFrameUploadMax = 32;
  constexpr bool kContributeIncludeRawLatency = false;
  constexpr bool kContributeRetryOnceOnFail = false;
  constexpr bool kContributeRawTlsFallbackOnTlsMem = true;
  constexpr bool kContributeEnableTcpProbe = false;
  constexpr bool kContributeHttpFallbackOnTlsMem = false;
  constexpr uint16_t kContributeHttpsConnectTimeoutMs = 8000;
  constexpr uint16_t kContributeHttpsIoTimeoutMs = 8000;
  constexpr uint16_t kContributeHttpConnectTimeoutMs = 2000;
  constexpr uint16_t kContributeHttpReadTimeoutMs = 3500;
  constexpr size_t kContributeResponseBufferCap = 2048;
  constexpr uint16_t kSdLogUploadConnectTimeoutMs = 4000;
  constexpr uint16_t kSdLogUploadIoTimeoutMs = 4500;
  constexpr uint16_t kSdLogUploadManualConnectTimeoutMs = 6000;
  constexpr uint16_t kSdLogUploadManualIoTimeoutMs = 12000;
  constexpr size_t kSdLogUploadChunkSize = 4096;
  constexpr size_t kSdLogUploadManualChunkSize = 8192;
  constexpr size_t kSdLogUploadBufferSize = kSdLogUploadManualChunkSize;
  constexpr const char *kSdLogUploadBaseUrl = "https://api.evdash.eu/v1/upload";
  uint8_t gSdLogUploadBuffer[kSdLogUploadBufferSize] = {0};
  constexpr uint32_t kFirmwareVersionCheckCooldownMs = 30000;
  constexpr uint16_t kFirmwareVersionHttpTimeoutMs = 4500;
  constexpr uint32_t kPairStatusPollIntervalMs = 8000;
  constexpr uint16_t kPairHttpTimeoutMs = 4500;
  constexpr size_t kSdV2MaxFileBytes = 256U * 1024U;

  // ABRP upload runs in the main loop, so avoid large temporary stack buffers here.
  // These static buffers are only used from the single-threaded board loop path.
  static char gAbrpPayloadBuffer[kAbrpPayloadBufferSize];
  static char gAbrpEncodedPayloadBuffer[kAbrpFormBufferSize];
  static char gAbrpFormBuffer[kAbrpFormBufferSize];

  bool publishMqttFloat(PubSubClient &client, const char *baseTopic, const char *suffix, float value, uint8_t precision = 2, bool retain = false)
  {
    char topic[80];
    char tmpVal[24];
    int topicLen = snprintf(topic, sizeof(topic), "%s%s", baseTopic, suffix);
    if (topicLen < 0 || topicLen >= static_cast<int>(sizeof(topic)))
    {
      return false;
    }
    dtostrf(value, 1, precision, tmpVal);
    return client.publish(topic, tmpVal, retain);
  }

  bool publishMqttString(PubSubClient &client, const char *baseTopic, const char *suffix, const char *value, bool retain = false)
  {
    char topic[80];
    int topicLen = snprintf(topic, sizeof(topic), "%s%s", baseTopic, suffix);
    if (topicLen < 0 || topicLen >= static_cast<int>(sizeof(topic)))
    {
      return false;
    }
    return client.publish(topic, value, retain);
  }

  bool publishMqttInt(PubSubClient &client, const char *baseTopic, const char *suffix, int32_t value, bool retain = false)
  {
    char topic[80];
    char tmpVal[16];
    int topicLen = snprintf(topic, sizeof(topic), "%s%s", baseTopic, suffix);
    if (topicLen < 0 || topicLen >= static_cast<int>(sizeof(topic)))
    {
      return false;
    }
    snprintf(tmpVal, sizeof(tmpVal), "%ld", static_cast<long>(value));
    return client.publish(topic, tmpVal, retain);
  }


  struct HeapCapsJsonAllocator
  {
    void *allocate(size_t size)
    {
      void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (ptr == nullptr)
      {
        ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      }
      return ptr;
    }

    void deallocate(void *ptr)
    {
      free(ptr);
    }

    void *reallocate(void *ptr, size_t newSize)
    {
      void *newPtr = heap_caps_realloc(ptr, newSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (newPtr == nullptr)
      {
        newPtr = heap_caps_realloc(ptr, newSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      }
      return newPtr;
    }
  };

  using HeapCapsJsonDocument = BasicJsonDocument<HeapCapsJsonAllocator>;

  bool isTlsMemoryIssue(int lastTlsErrCode, const String &lastTlsErrText)
  {
    String text = lastTlsErrText;
    text.toLowerCase();
    return lastTlsErrCode == MBEDTLS_ERR_X509_ALLOC_FAILED ||
           lastTlsErrCode == -16 ||
           text.indexOf("alloc") != -1;
  }

  char *allocContributePayloadBuffer(size_t payloadLen, bool &psramBuffer)
  {
    psramBuffer = false;
    char *buffer = (char *)heap_caps_malloc(payloadLen + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buffer != nullptr)
    {
      psramBuffer = true;
      return buffer;
    }
    return (char *)heap_caps_malloc(payloadLen + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }

  bool isMobileRelayClientConnected()
  {
    return mobileRelay != nullptr && mobileRelay->clientConnected();
  }

  size_t encodeQuotes(char *dest, size_t destSize, const char *source)
  {
    size_t written = 0;
    while (*source != '\0')
    {
      if (*source == '"')
      {
        if (written + 3 >= destSize)
        {
          break;
        }
        dest[written++] = '%';
        dest[written++] = '2';
        dest[written++] = '2';
      }
      else
      {
        if (written + 1 >= destSize)
        {
          break;
        }
        dest[written++] = *source;
      }
      source++;
    }
    dest[written] = '\0';
    return written;
  }

  bool isGpsCoordSane(float lat, float lon)
  {
    if (!isfinite(lat) || !isfinite(lon))
    {
      return false;
    }
    if (fabsf(lat) < 0.0001f || fabsf(lon) < 0.0001f)
    {
      return false;
    }
    if (lat < -90.0f || lat > 90.0f)
    {
      return false;
    }
    if (lon < -180.0f || lon > 180.0f)
    {
      return false;
    }
    return true;
  }

  float gpsDistanceMeters(float lat1, float lon1, float lat2, float lon2)
  {
    constexpr float kEarthRadiusMeters = 6371000.0f;
    constexpr float kDegToRad = 0.017453292519943295f;
    float dLat = (lat2 - lat1) * kDegToRad;
    float dLon = (lon2 - lon1) * kDegToRad;
    float lat1Rad = lat1 * kDegToRad;
    float lat2Rad = lat2 * kDegToRad;
    float sinLat = sinf(dLat * 0.5f);
    float sinLon = sinf(dLon * 0.5f);
    float a = (sinLat * sinLat) + (cosf(lat1Rad) * cosf(lat2Rad) * sinLon * sinLon);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return kEarthRadiusMeters * c;
  }

  bool isGpsFixUsable(const LiveData *liveData)
  {
    if (liveData == nullptr)
    {
      return false;
    }
    if (liveData->params.gpsLat == -1.0f || liveData->params.gpsLon == -1.0f)
    {
      return false;
    }
    if (!isGpsCoordSane(liveData->params.gpsLat, liveData->params.gpsLon))
    {
      return false;
    }
    if (liveData->params.gpsValid)
    {
      return true;
    }

    if (liveData->params.gpsLastFixTime > 0 &&
        liveData->params.currentTime > 0 &&
        liveData->params.currentTime >= liveData->params.gpsLastFixTime &&
        static_cast<uint32_t>(liveData->params.currentTime - liveData->params.gpsLastFixTime) <= kGpsFixFreshnessSec)
    {
      return true;
    }

    if (liveData->params.gpsLastFixMs != 0)
    {
      const uint32_t nowMs = millis();
      if ((nowMs - liveData->params.gpsLastFixMs) <= (kGpsFixFreshnessSec * 1000U))
      {
        return true;
      }
    }

    return false;
  }

  float roundToPrecision(float value, float multiplier)
  {
    if (!isfinite(value))
    {
      return value;
    }
    return roundf(value * multiplier) / multiplier;
  }

  String formatJsonNumber(float value, uint8_t digits)
  {
    if (!isfinite(value))
    {
      return "null";
    }
    return String(value, static_cast<unsigned int>(digits));
  }

  template <typename TJson>
  void setJsonNumber(TJson &json, const char *key, float value, uint8_t digits)
  {
    if (!isfinite(value))
    {
      json[key] = nullptr;
      return;
    }
    json[key] = serialized(formatJsonNumber(value, digits));
  }

  float normalizeHeadingDeg(float headingDeg)
  {
    if (!isfinite(headingDeg))
    {
      return -1.0f;
    }
    // Some GPS modules provide tenths without decimal point (e.g. 1715 => 171.5 deg).
    if (headingDeg > 360.0f && headingDeg <= 3600.0f)
    {
      headingDeg /= 10.0f;
    }
    while (headingDeg < 0.0f)
    {
      headingDeg += 360.0f;
    }
    while (headingDeg >= 360.0f)
    {
      headingDeg -= 360.0f;
    }
    return headingDeg;
  }

  float gpsHeadingFromCoords(float lat1, float lon1, float lat2, float lon2)
  {
    constexpr float kDegToRad = 0.017453292519943295f;
    constexpr float kRadToDeg = 57.29577951308232f;
    float lat1Rad = lat1 * kDegToRad;
    float lat2Rad = lat2 * kDegToRad;
    float dLonRad = (lon2 - lon1) * kDegToRad;

    float y = sinf(dLonRad) * cosf(lat2Rad);
    float x = (cosf(lat1Rad) * sinf(lat2Rad)) - (sinf(lat1Rad) * cosf(lat2Rad) * cosf(dLonRad));

    if (!isfinite(x) || !isfinite(y) || (fabsf(x) < 0.000001f && fabsf(y) < 0.000001f))
    {
      return -1.0f;
    }

    return normalizeHeadingDeg(atan2f(y, x) * kRadToDeg);
  }

  String formatTimestampYyMmDdHhIiSs(time_t timestamp)
  {
    if (timestamp <= 0)
    {
      return "";
    }
    struct tm tmValue;
    if (localtime_r(&timestamp, &tmValue) == nullptr)
    {
      return "";
    }
    char out[16] = {0};
    snprintf(out, sizeof(out), "%02d%02d%02d%02d%02d%02d",
             (tmValue.tm_year + 1900) % 100,
             tmValue.tm_mon + 1,
             tmValue.tm_mday,
             tmValue.tm_hour,
             tmValue.tm_min,
             tmValue.tm_sec);
    return String(out);
  }

  String normalizeDeviceIdForApi(const String &deviceId)
  {
    String normalized = "";
    normalized.reserve(deviceId.length());
    for (size_t i = 0; i < deviceId.length(); i++)
    {
      const char ch = deviceId.charAt(i);
      if (ch >= '0' && ch <= '9')
      {
        normalized += ch;
      }
      else if (ch >= 'A' && ch <= 'F')
      {
        normalized += ch;
      }
      else if (ch >= 'a' && ch <= 'f')
      {
        normalized += static_cast<char>(ch - ('a' - 'A'));
      }
    }
    if (normalized.length() == 32)
    {
      return normalized;
    }
    return deviceId;
  }

  const char *getCompiledDeviceTypeForApi()
  {
#ifdef BOARD_M5STACK_CORES3
    return "coreS3";
#elif defined(BOARD_M5STACK_CORE2)
    return "core2";
#else
    return "unknown";
#endif
  }

  String toAbsoluteSdPath(const String &fileName)
  {
    if (fileName.length() == 0)
    {
      return "";
    }
    if (fileName.charAt(0) == '/')
    {
      return fileName;
    }
    return "/" + fileName;
  }

  bool isPendingSdV2LogFile(const String &filePath)
  {
    return filePath.endsWith("_v2.json") && !filePath.endsWith("_v2_uploaded.json");
  }

  bool isUploadedSdV2LogFile(const String &filePath)
  {
    return filePath.endsWith("_v2_uploaded.json");
  }

  String toUploadedSdV2Path(const String &pendingFilePath)
  {
    if (!isPendingSdV2LogFile(pendingFilePath))
    {
      return "";
    }
    return pendingFilePath.substring(0, pendingFilePath.length() - 8) + "_v2_uploaded.json";
  }

  bool parseSdLogYyMmDdHhMm(const String &filePath, time_t &outTs)
  {
    outTs = 0;
    int slashPos = filePath.lastIndexOf('/');
    String baseName = (slashPos >= 0) ? filePath.substring(slashPos + 1) : filePath;
    if (baseName.length() < 10)
    {
      return false;
    }
    for (uint8_t i = 0; i < 10; i++)
    {
      const char ch = baseName.charAt(i);
      if (ch < '0' || ch > '9')
      {
        return false;
      }
    }

    const int year = 2000 + baseName.substring(0, 2).toInt();
    const int month = baseName.substring(2, 4).toInt();
    const int day = baseName.substring(4, 6).toInt();
    const int hour = baseName.substring(6, 8).toInt();
    const int minute = baseName.substring(8, 10).toInt();
    if (month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59)
    {
      return false;
    }

    struct tm tmValue = {};
    tmValue.tm_year = year - 1900;
    tmValue.tm_mon = month - 1;
    tmValue.tm_mday = day;
    tmValue.tm_hour = hour;
    tmValue.tm_min = minute;
    tmValue.tm_sec = 0;

    const time_t parsed = mktime(&tmValue);
    if (parsed <= 0)
    {
      return false;
    }
    outTs = parsed;
    return true;
  }

  bool isAllDigits(const String &value)
  {
    if (value.length() == 0)
    {
      return false;
    }
    for (uint16_t i = 0; i < value.length(); i++)
    {
      const char ch = value.charAt(i);
      if (ch < '0' || ch > '9')
      {
        return false;
      }
    }
    return true;
  }

  String nextSdV2RolloverPath(const String &currentPath)
  {
    if (currentPath.length() == 0)
    {
      return "";
    }

    String baseName = currentPath;
    if (baseName.charAt(0) == '/')
    {
      baseName = baseName.substring(1);
    }
    if (!baseName.endsWith("_v2.json"))
    {
      return "";
    }

    String stem = baseName.substring(0, baseName.length() - 8); // remove "_v2.json"
    uint16_t nextSeq = 1;
    const int sepPos = stem.lastIndexOf('_');
    if (sepPos > 0 && sepPos < static_cast<int>(stem.length() - 1))
    {
      const String suffix = stem.substring(sepPos + 1);
      if (isAllDigits(suffix))
      {
        nextSeq = static_cast<uint16_t>(suffix.toInt() + 1);
        stem = stem.substring(0, sepPos);
      }
    }

    if (stem.length() == 0)
    {
      stem = "log";
    }

    for (uint16_t seq = nextSeq; seq < 9999; seq++)
    {
      const String candidate = "/" + stem + "_" + String(seq) + "_v2.json";
      if (!SD.exists(candidate.c_str()))
      {
        return candidate;
      }
    }
    return "";
  }

  bool rotateSdV2FileIfNeeded(char *fileNameBuffer, size_t fileNameBufferSize, size_t pendingAppendBytes)
  {
    if (fileNameBuffer == nullptr || fileNameBufferSize == 0 || pendingAppendBytes == 0)
    {
      return false;
    }

    const String currentPath = toAbsoluteSdPath(String(fileNameBuffer));
    if (!isPendingSdV2LogFile(currentPath))
    {
      return false;
    }

    size_t currentSize = 0;
    File currentFile = SD.open(currentPath.c_str(), FILE_READ);
    if (currentFile && !currentFile.isDirectory())
    {
      currentSize = static_cast<size_t>(currentFile.size());
    }
    if (currentFile)
    {
      currentFile.close();
    }

    if ((currentSize + pendingAppendBytes) <= kSdV2MaxFileBytes)
    {
      return false;
    }

    const String nextPath = nextSdV2RolloverPath(currentPath);
    if (nextPath.length() == 0 || nextPath.length() >= fileNameBufferSize)
    {
      return false;
    }

    nextPath.toCharArray(fileNameBuffer, fileNameBufferSize);
    return true;
  }

  bool hasContributeRawFrames(const LiveData *liveData)
  {
    if (liveData == nullptr)
    {
      return false;
    }
    for (uint8_t i = 0; i < liveData->contributeRawFrameCount; i++)
    {
      const LiveData::ContributeRawFrame &raw = liveData->contributeRawFrames[i];
      if (raw.key[0] != '\0' && raw.value[0] != '\0')
      {
        return true;
      }
    }
    return false;
  }

  bool isContributeV2SnapshotEffectivelyEmpty(const LiveData *liveData)
  {
    if (liveData == nullptr)
    {
      return true;
    }
    if (liveData->params.getValidResponse)
    {
      return false;
    }
    if (isGpsFixUsable(liveData))
    {
      return false;
    }
    if (liveData->params.ignitionOn || liveData->params.chargingOn ||
        liveData->params.chargerACconnected || liveData->params.chargerDCconnected)
    {
      return false;
    }
    if (hasContributeRawFrames(liveData))
    {
      return false;
    }
    if (liveData->params.socPerc >= 0 || liveData->params.sohPerc >= 0 ||
        liveData->params.batPowerKw > -999.0f || liveData->params.batPowerKwh100 >= 0 ||
        liveData->params.batVoltage >= 0 || liveData->params.batPowerAmp > -999.0f ||
        liveData->params.auxVoltage >= 0 || liveData->params.auxCurrentAmp > -999.0f ||
        liveData->params.batMinC > -100.0f || liveData->params.batMaxC > -100.0f ||
        liveData->params.indoorTemperature > -100.0f || liveData->params.outdoorTemperature > -100.0f ||
        liveData->params.speedKmh >= 0 || liveData->params.odoKm >= 0 ||
        liveData->params.batCellMinV >= 0 || liveData->params.batCellMaxV >= 0 ||
        liveData->params.batCellMinVNo != 255 ||
        liveData->params.cumulativeEnergyChargedKWh >= 0 ||
        liveData->params.cumulativeEnergyDischargedKWh >= 0)
    {
      return false;
    }
    return true;
  }
} // namespace
