/**
 * Initializes the board hardware and peripherals.
 * Sets up pins, time, display, etc.
 *
This code initializes and configures the hardware and peripherals for a display board.

@src\Board320_240.cpp:1-5124 initializes a 320x240 pixel display board.

It first includes necessary libraries and header files. It gets the current date and time from the real time clock module and sets it using the time library functions.

It initializes the display driver and sets the rotation, brightness and color depth based on configuration settings. It also initializes a sprite graphics buffer to hold images to display.

The initBoard() function configures the input buttons, gets the current time, and increments a boot counter.

afterSetup() checks if the board is waking from sleep mode, initializes the voltmeter module if enabled, and calls additional setup functions from a board interface class.

It prints debug messages, checks sleep mode settings, and decides whether to go back to sleep or wake up the board.

The main purpose is to initialize all the hardware like the screen, buttons, voltmeter, etc. and configure settings like brightness and rotation. It gets the current time and date and stores it.
It also handles waking from sleep and going back to sleep.

The key inputs are the real time clock data, configuration settings, and sleep mode/wakeup logic.

The outputs are an initialized display, sprite buffer, time, debug messages, and sleep management.

The main logic flow is:

1. Include libraries
2. Get time and date
3. Initialize display and sprites
4. Configure input buttons
5. Increment boot count
6. Check sleep mode and wake/sleep logic
7. Initialize other devices like voltmeter
8. Call additional setup functions

So in summary, it initializes the core display and hardware functionality, retrieves time and settings, handles wake/sleep logic, and prepares the board for operation.

 */
#include <FS.h>
#include <analogWrite.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h> //To be able to use https with ABRP api server
#include <Update.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <mbedtls/x509.h>
#include "config.h"
#include "BoardInterface.h"
#include "Board320_240.h"
#include <time.h>
#include <ArduinoJson.h>
#include "CarModelUtils.h"
#include "EvDashMobileRelay.h"
#include "traccar.h"
#include <esp_sntp.h>

#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
#include <PubSubClient.h>
#endif // BOARD_M5STACK_CORE2 || BOARD_M5STACK_CORES3

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
  constexpr uint16_t kAbrpHttpsConnectTimeoutMs = 1000;
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

#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
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
#endif

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

#ifdef BOARD_M5STACK_CORES3
// SD card
#define TFCARD_CS_PIN 4
#endif // BOARD_M5STACK_CORES3

/**
   Read the battery-backed RTC.
   Returns 0 when the clock has never been set (or holds a pre-2021 date), which
   tells the caller to fall back to SNTP.
*/
time_t Board320_240::rtcReadTime()
{
#ifdef BOARD_M5STACK_CORE2
  RTC_TimeTypeDef RTCtime;
  RTC_DateTypeDef RTCdate;
  M5.Rtc.GetTime(&RTCtime);
  M5.Rtc.GetDate(&RTCdate);
  if (RTCdate.Year > 2020)
  {
    struct tm tm_tmp;
    tm_tmp.tm_year = RTCdate.Year - 1900;
    tm_tmp.tm_mon = RTCdate.Month - 1;
    tm_tmp.tm_mday = RTCdate.Date;
    tm_tmp.tm_hour = RTCtime.Hours;
    tm_tmp.tm_min = RTCtime.Minutes;
    tm_tmp.tm_sec = RTCtime.Seconds;

    return mktime(&tm_tmp);
  }
#endif // BOARD_M5STACK_CORE2
#ifdef BOARD_M5STACK_CORES3
  auto dt = CoreS3.Rtc.getDateTime();
  if (dt.date.year > 2020)
  {
    struct tm tm_tmp;
    tm_tmp.tm_year = dt.date.year - 1900;
    tm_tmp.tm_mon = dt.date.month - 1;
    tm_tmp.tm_mday = dt.date.date;
    tm_tmp.tm_hour = dt.time.hours;
    tm_tmp.tm_min = dt.time.minutes;
    tm_tmp.tm_sec = dt.time.seconds;

    return mktime(&tm_tmp);
  }
#endif // BOARD_M5STACK_CORES3
  return 0;
}

/**
   Persist a time correction to the battery-backed RTC.
*/
void Board320_240::rtcWriteTime(time_t newTime)
{
#ifdef BOARD_M5STACK_CORE2
  // Core2 stores local wall-clock time. mktime() interpreted the caller's
  // components as local, so recovering them via localtime() round-trips them.
  struct tm *tmm = localtime(&newTime);
  RTC_TimeTypeDef RTCtime = {tmm->tm_hour, tmm->tm_min, tmm->tm_sec};
  RTC_DateTypeDef RTCdate = {tmm->tm_year + 1900, tmm->tm_mon + 1, tmm->tm_mday};

  M5.Rtc.SetTime(&RTCtime);
  M5.Rtc.SetDate(&RTCdate);
#endif // BOARD_M5STACK_CORE2
#ifdef BOARD_M5STACK_CORES3
  // CoreS3's library takes a UTC tm, matching the pre-extraction call site.
  CoreS3.Rtc.setDateTime(gmtime(&newTime));
#endif // BOARD_M5STACK_CORES3
}

/**
   Init board
*/
void Board320_240::initBoard()
{
  liveData->params.booting = true;

  // Seed the system clock. Boards with a battery-backed RTC supply the initial
  // time here; the rest return 0 and SNTP establishes it later.
  struct timeval tv;
  tv.tv_sec = rtcReadTime();
  tv.tv_usec = 0;

  settimeofday(&tv, NULL);
  sntp_set_time_sync_notification_cb(sntpTimeSyncNotificationCallback);
  struct tm tm;
  if (getLocalTime(&tm, 0))
  {
    liveData->params.currentTime = mktime(&tm);
  }
  else
  {
    liveData->params.currentTime = tv.tv_sec;
  }
  liveData->params.chargingStartTime = liveData->params.currentTime;
}

/**
   After setup device
*/
void Board320_240::afterSetup()
{
  // Check if board was sleeping
  bool afterSetup = false;

  syslog->print("SleepMode: ");
  syslog->println(liveData->settings.sleepModeLevel);

  // Init Voltmeter
  if (liveData->settings.voltmeterEnabled == 1)
  {
    syslog->println("Initializing INA3221 voltmeter:");
    ina3221.begin();
    syslog->print("ch1:");
    syslog->print(ina3221.getBusVoltage_V(1));
    syslog->println("V");
    syslog->print("ch1:");
    syslog->print(ina3221.getCurrent_mA(1));
    syslog->println("mA");
  }

  // Init display
  syslog->println("Init TFT display");
  tft.begin();
  tft.setRotation(liveData->settings.displayRotation);

  setBrightness();
  showBootProgress("Display initialization...", "Preparing resources", TFT_RED);

  bool psramUsed = false; // 320x240 16bpp sprites requires psram
#if defined(ESP32) && defined(CONFIG_SPIRAM_SUPPORT)
  if (psramFound())
    psramUsed = true;
#endif
  printHeapMemory();
  syslog->println((psramUsed) ? "Sprite 16" : "Sprite 8");
  spriteColorDepth = (psramUsed) ? 16 : 8;
  spr.setColorDepth(spriteColorDepth);
  spr.createSprite(320, 240);
  menuBackbufferActive = false;
  liveData->params.spriteInit = true;
  printHeapMemory();
  showBootProgress("Display initialization...", "Framebuffer ready", TFT_PURPLE);

  // Show test data on right button during boot device
  liveData->params.displayScreen = liveData->settings.defaultScreen;
  showBootProgress("Checking test mode...", "Hold right button for demo", TFT_YELLOW);
  if (isButtonPressed(pinButtonRight))
  {
    syslog->printf("Right button pressed");
    loadTestData();
    printHeapMemory();
    showBootProgress("Demo mode enabled", "Using static test data", TFT_YELLOW);
  }

  // Wifi
  // Starting Wifi after BLE prevents reboot loop
  if (!liveData->params.wifiApMode &&
      liveData->settings.wifiEnabled == 1)
  {
    showBootProgress("WiFi initialization...", "Connecting to configured AP", TFT_BLUE);
    wifiSetup();
    printHeapMemory();
  }
  else
  {
    showBootProgress("WiFi initialization...", "Skipped (disabled)", TFT_BLUE);
  }

  // Init GPS
  if (liveData->settings.gpsModuleType != GPS_MODULE_TYPE_NONE && liveData->settings.gpsHwSerialPort <= 2)
  {
    showBootProgress("GPS initialization...", "Starting GPS serial", TFT_SKYBLUE);
    initGPS();
    printHeapMemory();
  }
  else
  {
    showBootProgress("GPS initialization...", "Skipped (not configured)", TFT_SKYBLUE);
  }

  // SD card
  if (liveData->settings.sdcardEnabled == 1)
  {
    showBootProgress("SD card initialization...", "Mounting storage", TFT_ORANGE);
    if (sdcardMount() && liveData->settings.sdcardAutstartLog == 1)
    {
      syslog->println("Toggle recording on SD card");
      sdcardToggleRecording();
      printHeapMemory();
      showBootProgress("SD card initialization...", "Autolog enabled", TFT_ORANGE);
    }
  }
  else
  {
    showBootProgress("SD card initialization...", "Skipped (disabled)", TFT_ORANGE);
  }

  // Init comm device
  showBootProgress("Adapter initialization...", "Starting OBD2/CAN comm", TFT_SILVER);
  BoardInterface::afterSetup();
  printHeapMemory();

  syslog->println("COMM in main loop (threading removed)");
  syslog->println("NET send in main loop (queue/task removed)");

  showBootProgress("Finalizing startup...", "Syncing clocks", TFT_GREEN);
  showTime();
  showBootProgress("Startup completed", "Loading main screen", TFT_GREEN);

  liveData->params.wakeUpTime = liveData->params.currentTime;
  liveData->params.lastCanbusResponseTime = liveData->params.currentTime;
  liveData->params.booting = false;
}

/**
 * Draw currently selected screen into `spr` without pushing it to TFT.
 */
bool Board320_240::drawActiveScreenToSprite()
{

  // Headlights reminders
  if (!testDataMode && liveData->settings.headlightsReminder == 1 && liveData->params.forwardDriveMode &&
      !liveData->params.headLights && !liveData->params.autoLights)
  {
    spr.fillSprite(TFT_RED);
    sprSetFont(fontOrbitronLight32);
    spr.setTextColor(TFT_WHITE);
    spr.setTextDatum(MC_DATUM);
    sprDrawString("! LIGHTS OFF !", 160, 120);
    return true;
  }

  spr.fillSprite(TFT_BLACK);

  liveData->params.displayScreenAutoMode = SCREEN_AUTO;

  // Display selected screen
  switch (liveData->params.displayScreen)
  {
  // 1. Auto mode = >5kpm Screen 3 - speed, other wise basic Screen2 - Main screen, if charging then Screen 5 Graph
  case SCREEN_AUTO:
    if (liveData->params.batCellMinV > 1.5 && liveData->params.batCellMinV < 3.0)
    {
      if (liveData->params.displayScreenAutoMode != SCREEN_CELLS)
        liveData->params.displayScreenAutoMode = SCREEN_CELLS;
      drawSceneBatteryCells();
    }
    else if (liveData->params.speedKmh > 15 || (liveData->params.speedKmhGPS > 15 && liveData->params.gpsSat >= 4))
    {
      if (liveData->params.displayScreenAutoMode != SCREEN_SPEED)
        liveData->params.displayScreenAutoMode = SCREEN_SPEED;
      drawSceneSpeed();
    }
    else if (liveData->params.chargingOn)
    {
      if (liveData->params.displayScreenAutoMode != SCREEN_CHARGING)
        liveData->params.displayScreenAutoMode = SCREEN_CHARGING;
      drawSceneChargingGraph();
    }
    else
    {
      if (liveData->params.displayScreenAutoMode != SCREEN_DASH)
        liveData->params.displayScreenAutoMode = SCREEN_DASH;
      drawSceneMain();
    }
    break;
  // 2. Main screen
  case SCREEN_DASH:
    drawSceneMain();
    break;
  // 3. Big speed + kwh/100km
  case SCREEN_SPEED:
    drawSceneSpeed();
    break;
  // 4. Battery cells
  case SCREEN_CELLS:
    drawSceneBatteryCells();
    break;
  // 5. Charging graph
  case SCREEN_CHARGING:
    drawSceneChargingGraph();
    break;
  // 6. SOC10% table (CEC-CED)
  case SCREEN_SOC10:
    drawSceneSoc10Table();
    break;
  // 7. Debug screen
  case SCREEN_DEBUG:
    drawSceneDebug();
    break;
  // 8. HUD
  case SCREEN_HUD:
    drawSceneHud();
    break;
  }

  // Skip following lines for HUD display mode
  if (liveData->params.displayScreen == SCREEN_HUD)
  {
    return false;
  }

  // GPS state
  if ((gpsHwUart != NULL || liveData->params.gpsSat > 0) && (liveData->params.displayScreen == SCREEN_SPEED || liveData->params.displayScreenAutoMode == SCREEN_SPEED))
  {
    const int gpsIndicatorX = 168;
    spr.fillCircle(gpsIndicatorX, 10, 8, (liveData->params.gpsValid) ? TFT_GREEN : TFT_RED);
    spr.fillCircle(gpsIndicatorX, 10, 6, TFT_BLACK);
    spr.setTextSize(1);
    spr.setTextColor((liveData->params.gpsValid) ? TFT_GREEN : TFT_WHITE);
    spr.setTextDatum(TL_DATUM);
    sprintf(tmpStr1, "%d", liveData->params.gpsSat);
    sprSetFont(fontFont2);
    sprDrawString(tmpStr1, gpsIndicatorX + 14, 2);
  }

  // SDCARD recording
  // liveData->params.sdcardRecording
  if (liveData->settings.sdcardEnabled == 1 && (liveData->params.queueLoopCounter & 1) == 1)
  {
    const int sdIndicatorX = (liveData->params.displayScreen == SCREEN_SPEED || liveData->params.displayScreenAutoMode == SCREEN_SPEED) ? 168 : 310;
    const bool sdV2BackgroundUploadActive = (sdV2UploadFilePath.length() > 0 &&
                                             sdV2UploadFileName.length() > 0 &&
                                             liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI &&
                                             liveData->settings.wifiEnabled == 1 &&
                                             !liveData->params.wifiApMode &&
                                             WiFi.status() == WL_CONNECTED &&
                                             liveData->params.netAvailable);
    const int sdOuterR = 4;
    const int sdInnerR = sdV2BackgroundUploadActive ? 4 : 3;
    spr.fillCircle(sdIndicatorX, 10, sdOuterR, TFT_BLACK);
    spr.fillCircle(sdIndicatorX, 10, sdInnerR,
                   (liveData->params.sdcardInit) ? (liveData->params.sdcardRecording) ? (strlen(liveData->params.sdcardFilename) != 0) ? TFT_GREEN /* assigned filename (opsec from bms or gsm/gps timestamp */ : TFT_BLUE /* recording started but waiting for data */ : TFT_ORANGE /* sdcard init ready but recording not started*/ : TFT_YELLOW /* failed to initialize sdcard */
    );
  }

  const auto remoteApiConfiguredForWifi = [this]() -> bool
  {
    if (liveData->settings.remoteUploadIntervalSec == 0)
    {
      return false;
    }
    const char *url = liveData->settings.remoteApiUrl;
    if (url == nullptr || url[0] == '\0')
    {
      return false;
    }
    if (strcmp(url, "not_set") == 0)
    {
      return false;
    }
    return strstr(url, "http") != nullptr;
  }();

  const auto abrpConfiguredForWifi = [this]() -> bool
  {
    if (liveData->settings.remoteUploadAbrpIntervalSec == 0)
    {
      return false;
    }
    const char *token = liveData->settings.abrpApiToken;
    if (token == nullptr || token[0] == '\0')
    {
      return false;
    }
    if (strcmp(token, "empty") == 0 || strcmp(token, "not_set") == 0)
    {
      return false;
    }
    return true;
  }();

  const bool contributeOnlyConfigured =
      (liveData->settings.contributeData == 1) &&
      !remoteApiConfiguredForWifi &&
      !abrpConfiguredForWifi;

  const uint8_t wifiOkWindowSec = contributeOnlyConfigured ? 75 : 15;

  // Ignition ON, Gyro motion
  if (liveData->params.displayScreen == SCREEN_SPEED || liveData->params.displayScreenAutoMode == SCREEN_SPEED)
  {
    if (liveData->params.gyroSensorMotion)
    {
      spr.fillRect(128, 3, 4, 18, TFT_ORANGE);
    }
    if (liveData->params.ignitionOn)
    {
      spr.fillRect(130, 4, 2, 14, TFT_GREEN);
    }
  }

  // WiFi Status
  if (liveData->settings.wifiEnabled == 1 && liveData->settings.remoteUploadModuleType == 1)
  {
    if (liveData->params.displayScreen == SCREEN_SPEED || liveData->params.displayScreenAutoMode == SCREEN_SPEED)
    {
      const uint16_t wifiColor =
          (WiFi.status() == WL_CONNECTED)
              ? ((liveData->params.lastSuccessNetSendTime + wifiOkWindowSec >= liveData->params.currentTime) ? TFT_GREEN /* last request was 200 OK */ : TFT_YELLOW /* wifi connected but not send */)
              : TFT_RED; /* wifi not connected */
      const bool wifiTransferActive =
          (wifiTransferLastActivityMs != 0 &&
           (millis() - wifiTransferLastActivityMs) <= kWifiTransferIndicatorWindowMs);

      const int wifiCx = 146;
      const int wifiCy = 11;
      if (wifiTransferActive)
      {
        // Data transfer icon: upper arrow right, lower arrow left.
        spr.drawLine(wifiCx - 9, wifiCy - 3, wifiCx + 6, wifiCy - 3, wifiColor);
        spr.drawLine(wifiCx + 6, wifiCy - 3, wifiCx + 2, wifiCy - 6, wifiColor);
        spr.drawLine(wifiCx + 6, wifiCy - 3, wifiCx + 2, wifiCy, wifiColor);
        spr.drawLine(wifiCx + 9, wifiCy + 4, wifiCx - 6, wifiCy + 4, wifiColor);
        spr.drawLine(wifiCx - 6, wifiCy + 4, wifiCx - 2, wifiCy + 1, wifiColor);
        spr.drawLine(wifiCx - 6, wifiCy + 4, wifiCx - 2, wifiCy + 7, wifiColor);
      }
      else
      {
        // WiFi icon (upper arcs + center dot), tuned to be ~20% larger.
        const int wifiOuterR = 10;
        const int wifiMidR = 6;
        const int wifiInnerR = 4;
        spr.drawCircle(wifiCx, wifiCy, wifiOuterR, wifiColor);
        spr.drawCircle(wifiCx, wifiCy, wifiMidR, wifiColor);
        spr.drawCircle(wifiCx, wifiCy, wifiInnerR, wifiColor);
        spr.fillRect(wifiCx - (wifiOuterR + 1), wifiCy, (wifiOuterR + 1) * 2 + 1, wifiOuterR + 2, TFT_BLACK); // keep only upper arcs
        spr.fillCircle(wifiCx, wifiCy + 3, 2, wifiColor);
      }

      if (liveData->params.isWifiBackupLive)
      {
        // Backup link badge.
        spr.fillCircle(wifiCx + 11, 2, 2, wifiColor);
      }
    }
    else if (liveData->params.displayScreen != SCREEN_BLANK)
    {
      spr.fillRect(308, 0, 5, 5,
                   (WiFi.status() == WL_CONNECTED) ? (liveData->params.lastSuccessNetSendTime + wifiOkWindowSec >= liveData->params.currentTime) ? TFT_GREEN /* last request was 200 OK */ : TFT_YELLOW /* wifi connected but not send */ : TFT_RED /* wifi not connected */
      );
    }
  }

  // Door status
  if (liveData->params.displayScreen == SCREEN_SPEED || liveData->params.displayScreenAutoMode == SCREEN_SPEED)
  {
    const bool showCarIcon = (liveData->params.trunkDoorOpen || liveData->params.leftFrontDoorOpen || liveData->params.rightFrontDoorOpen ||
                              liveData->params.leftRearDoorOpen || liveData->params.rightRearDoorOpen || liveData->params.hoodDoorOpen);
    if (showCarIcon)
    {
      const int16_t carX = 40;
      const int16_t carY = 40;
      const int16_t carW = 50;
      const int16_t carH = 90;
      const int16_t bodyX = carX + 3;
      const int16_t bodyY = carY + 6;
      const int16_t bodyW = carW - 6;
      const int16_t bodyH = carH - 12;
      const uint16_t bodyColor = 0x4208;
      const uint16_t cabinColor = 0x3186;
      const uint16_t glassColor = 0x7BEF;
      const uint16_t lineColor = 0x5AEB;

      // Top-view silhouette
      spr.fillRoundRect(bodyX, bodyY, bodyW, bodyH, 10, bodyColor);
      spr.drawRoundRect(bodyX, bodyY, bodyW, bodyH, 10, lineColor);
      spr.fillRoundRect(bodyX + 8, bodyY + 14, bodyW - 16, bodyH - 28, 8, cabinColor);
      spr.drawRoundRect(bodyX + 8, bodyY + 14, bodyW - 16, bodyH - 28, 8, lineColor);
      spr.fillRoundRect(bodyX + 10, bodyY + 18, bodyW - 20, 12, 4, glassColor);
      spr.fillRoundRect(bodyX + 10, bodyY + bodyH - 30, bodyW - 20, 12, 4, glassColor);
      spr.drawFastVLine(bodyX + (bodyW / 2), bodyY + 20, bodyH - 40, lineColor);
      spr.fillRoundRect(bodyX - 3, bodyY + 22, 3, 10, 2, lineColor);
      spr.fillRoundRect(bodyX + bodyW, bodyY + 22, 3, 10, 2, lineColor);
      spr.drawFastHLine(bodyX + 2, bodyY + 24, 6, lineColor);
      spr.drawFastHLine(bodyX + 2, bodyY + bodyH - 24, 6, lineColor);
      spr.drawFastHLine(bodyX + bodyW - 8, bodyY + 24, 6, lineColor);
      spr.drawFastHLine(bodyX + bodyW - 8, bodyY + bodyH - 24, 6, lineColor);

      // Light indicators
      if (liveData->params.brakeLights)
      {
        spr.fillRoundRect(bodyX + 5, bodyY + bodyH, 10, 6, 2, TFT_RED);
        spr.fillRoundRect(bodyX + bodyW - 15, bodyY + bodyH, 10, 6, 2, TFT_RED);
      }
      if (liveData->params.headLights || liveData->params.dayLights)
      {
        const uint16_t headColor = liveData->params.headLights ? TFT_YELLOW : TFT_GOLD;
        spr.fillRoundRect(bodyX + 5, bodyY - 6, 10, 6, 2, headColor);
        spr.fillRoundRect(bodyX + bodyW - 15, bodyY - 6, 10, 6, 2, headColor);
      }

      // Charging door
      if (liveData->params.chargerACconnected || liveData->params.chargerDCconnected)
      {
        uint16_t doorColor = TFT_GREEN;
        if (liveData->params.chargerACconnected && liveData->params.chargerDCconnected)
          doorColor = TFT_MAGENTA;
        else if (liveData->params.chargerDCconnected)
          doorColor = TFT_BLUE;
        const int16_t doorX = bodyX + bodyW + 2;
        const int16_t doorY = bodyY + bodyH / 2 - 6;
        spr.fillRoundRect(doorX, doorY, 10, 12, 2, doorColor);
        spr.drawRoundRect(doorX, doorY, 10, 12, 2, lineColor);
        if (liveData->params.chargingOn)
        {
          spr.drawFastVLine(doorX + 5, doorY + 2, 8, TFT_WHITE);
          spr.drawFastHLine(doorX + 3, doorY + 6, 4, TFT_WHITE);
        }
      }

      if (liveData->params.trunkDoorOpen)
        spr.fillRoundRect(bodyX + 6, bodyY - 10, bodyW - 12, 10, 3, TFT_GOLD);
      if (liveData->params.leftFrontDoorOpen)
        spr.fillRoundRect(bodyX - 14, bodyY + 20, 12, 18, 3, TFT_GOLD);
      if (liveData->params.rightFrontDoorOpen)
        spr.fillRoundRect(bodyX - 14, bodyY + bodyH - 38, 12, 18, 3, TFT_GOLD);
      if (liveData->params.leftRearDoorOpen)
        spr.fillRoundRect(bodyX + bodyW + 2, bodyY + 20, 12, 18, 3, TFT_GOLD);
      if (liveData->params.rightRearDoorOpen)
        spr.fillRoundRect(bodyX + bodyW + 2, bodyY + bodyH - 38, 12, 18, 3, TFT_GOLD);
      if (liveData->params.hoodDoorOpen)
        spr.fillRoundRect(bodyX + 6, bodyY + bodyH, bodyW - 12, 10, 3, TFT_GOLD);
    }
  }
  else
  {
    if (liveData->params.trunkDoorOpen)
      spr.fillRect(20, 0, 320 - 40, 4, TFT_GOLD);
    if (liveData->params.leftFrontDoorOpen)
      spr.fillRect(0, 20, 4, 98, TFT_GOLD);
    if (liveData->params.rightFrontDoorOpen)
      spr.fillRect(0, 122, 4, 98, TFT_GOLD);
    if (liveData->params.leftRearDoorOpen)
      spr.fillRect(320 - 4, 20, 4, 98, TFT_GOLD);
    if (liveData->params.rightRearDoorOpen)
      spr.fillRect(320 - 4, 122, 4, 98, TFT_GOLD);
    if (liveData->params.hoodDoorOpen)
      spr.fillRect(20, 240 - 4, 320 - 40, 4, TFT_GOLD);
  }

  bool statusBoxUsed = false;
  constexpr int16_t statusBoxRadius = 8;

  // OBD adapter not connected
  if (liveData->settings.commType == COMM_TYPE_OBD2_BLE4 &&
      !liveData->commConnected && liveData->obd2ready)
  {
    // Print message
    spr.fillRect(0, 185, 220, 50, TFT_BLACK);
    spr.drawRect(0, 185, 220, 50, TFT_WHITE);
    spr.setTextSize(1);
    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(TFT_WHITE);
    sprintf(tmpStr1, "OBDII not connected #%d", commInterface->getConnectAttempts());
    sprSetFont(fontFont2);
    sprDrawString(tmpStr1, 10, 190);
    sprDrawString(commInterface->getConnectStatus().c_str(), 10, 210);
    statusBoxUsed = true;
  }
  else
    // CAN not connected
    if (canStatusMessageVisible())
    {
      // Print message
      spr.fillRoundRect(0, 185, 320, 50, statusBoxRadius, TFT_BLACK);
      spr.drawRoundRect(0, 185, 320, 50, statusBoxRadius, TFT_WHITE);
      spr.setTextSize(1);
      spr.setTextDatum(TL_DATUM);
      spr.setTextColor(TFT_WHITE);
      sprSetFont(fontRobotoThin24);
      sprintf(tmpStr1, "CAN #%d %s%d", commInterface->getConnectAttempts(), (liveData->params.stopCommandQueue ? "QS" : "QR"), liveData->params.queueLoopCounter, " ", liveData->currentAtshRequest);
      sprSetFont(fontFont2);
      sprDrawString(tmpStr1, 10, 190);
      sprDrawString(commInterface->getConnectStatus().c_str(), 10, 210);
      statusBoxUsed = true;
    }

  if (!statusBoxUsed && liveData->settings.wifiEnabled == 1 &&
      WiFi.status() == WL_CONNECTED && netStatusMessageVisible())
  {
    spr.fillRoundRect(0, 185, 320, 50, statusBoxRadius, TFT_BLACK);
    spr.drawRoundRect(0, 185, 320, 50, statusBoxRadius, TFT_WHITE);
    spr.setTextSize(1);
    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(TFT_WHITE);
    sprSetFont(fontFont2);
    sprDrawString("WiFi OK", 10, 190);
    sprDrawString("Net temporarily unavailable", 10, 210);
    statusBoxUsed = true;
  }

  return true;
}

void Board320_240::redrawScreen()
{
  lastRedrawTime = liveData->params.currentTime;
  liveData->redrawScreenRequested = false;

  if (liveData->menuVisible || currentBrightness == 0 || !liveData->params.spriteInit)
  {
    return;
  }
  if (redrawScreenIsRunning)
  {
    return;
  }
  redrawScreenIsRunning = true;
  messageDialogVisible = false;

  if (drawActiveScreenToSprite())
  {
    spr.pushSprite(0, 0);
  }

  redrawScreenIsRunning = false;
}

void Board320_240::showScreenSwipePreview(int16_t deltaX)
{
  static int16_t lastPreviewDeltaX = 0;
  static uint32_t lastPreviewMs = 0;
  const uint8_t firstCarouselScreen = SCREEN_DASH;
  const uint8_t lastCarouselScreen = SCREEN_DEBUG;
  if (liveData->menuVisible || currentBrightness == 0 || !liveData->params.spriteInit)
  {
    return;
  }
  if (redrawScreenIsRunning)
  {
    return;
  }
  if (liveData->params.displayScreen == SCREEN_HUD)
  {
    return;
  }

  if (deltaX > 319)
    deltaX = 319;
  else if (deltaX < -319)
    deltaX = -319;

  if (deltaX == 0)
  {
    lastPreviewDeltaX = 0;
    return;
  }

  const uint32_t nowMs = millis();
  const bool deltaChangedEnough = (abs(deltaX - lastPreviewDeltaX) >= 2);
  const bool frameElapsed = (nowMs - lastPreviewMs) >= 16;
  if (!deltaChangedEnough && !frameElapsed)
  {
    return;
  }
  lastPreviewDeltaX = deltaX;
  lastPreviewMs = nowMs;

  const uint8_t originalScreen = liveData->params.displayScreen;
  if (originalScreen < firstCarouselScreen || originalScreen > lastCarouselScreen)
  {
    return;
  }
  if ((deltaX > 0 && originalScreen == firstCarouselScreen) ||
      (deltaX < 0 && originalScreen == lastCarouselScreen))
  {
    return;
  }

  const uint8_t originalAutoMode = liveData->params.displayScreenAutoMode;
  const uint8_t adjacentScreen = (deltaX > 0) ? (originalScreen - 1) : (originalScreen + 1);

  redrawScreenIsRunning = true;

  tft.setRotation(liveData->settings.displayRotation);

  liveData->params.displayScreen = originalScreen;
  liveData->params.displayScreenAutoMode = originalAutoMode;
  if (drawActiveScreenToSprite())
  {
    spr.pushSprite(deltaX, 0);
  }

  liveData->params.displayScreen = adjacentScreen;
  liveData->params.displayScreenAutoMode = originalAutoMode;
  if (drawActiveScreenToSprite())
  {
    const int16_t adjacentX = deltaX + ((deltaX > 0) ? -320 : 320);
    spr.pushSprite(adjacentX, 0);
  }

  liveData->params.displayScreen = originalScreen;
  liveData->params.displayScreenAutoMode = originalAutoMode;
  redrawScreenIsRunning = false;
}

/**
 * Parse test data
 */
void Board320_240::loadTestData()
{
  syslog->println("Loading test data");

  testDataMode = true; // skip lights off message
  carInterface->loadTestData();
  redrawScreen();
}

/**
 * Print heap memory to serial console
 */
void Board320_240::printHeapMemory()
{
  syslog->printf("Total/free heap: %i/%i-%i, total/free PSRAM %i/%i bytes\n", ESP.getHeapSize(), ESP.getFreeHeap(), heap_caps_get_free_size(MALLOC_CAP_8BIT), ESP.getPsramSize(), ESP.getFreePsram());
}

/**
 * Update the IMU motion flag used to wake Sentry.
 * Motion = angular rate (gyro) OR linear acceleration off ~1 g (accelerometer),
 * so a smooth straight pull-away wakes the device too, not only a turn/bump.
 * The flag is a latch: set here on motion, consumed (cleared) once per mainLoop
 * pass by the wake logic. In Sentry boardLoop samples the IMU every 50 ms while
 * mainLoop runs ~1x/s, so without the latch only the last IMU sample of each
 * 1 s window was visible and short motion events were almost always missed.
 */
void Board320_240::updateGyroSensorMotion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ)
{
  // IMU not ready yet (all axes zero): keep the previous state.
  if (gyroX == 0.0 && gyroY == 0.0 && gyroZ == 0.0 && accX == 0.0 && accY == 0.0 && accZ == 0.0)
    return;

  // Snapshot for the debug screen (page 3), so motion detection can be verified live.
  liveData->params.imuGyroX = gyroX;
  liveData->params.imuGyroY = gyroY;
  liveData->params.imuGyroZ = gyroZ;
  liveData->params.imuAccX = accX;
  liveData->params.imuAccY = accY;
  liveData->params.imuAccZ = accZ;

  // Angular rate: a turn, bump or being picked up.
  bool motion = (abs(gyroX) > 12.0 || abs(gyroY) > 12.0 || abs(gyroZ) > 12.0);

  // Linear acceleration: |accel| is ~1 g at rest regardless of mounting angle;
  // driving accel, braking and road bumps push it off 1 g. The < 4 g guard
  // ignores a unit/scale glitch, so a bad read falls back to gyro-only instead
  // of pinning the device permanently awake. 0.09 g keeps ~2x margin above
  // typical accelerometer calibration bias (a few % of 1 g).
  const float accMag = sqrt(accX * accX + accY * accY + accZ * accZ);
  if (accMag < 4.0 && fabs(accMag - 1.0) > 0.09)
    motion = true;

  if (motion)
  {
    liveData->params.gyroSensorMotion = true;
    liveData->params.imuMotionCount++;
  }
}

/**
 * commLoop function - This function runs the main communication loop.
 * It calls the commInterface's mainLoop() method to read data from
 * BLE and CAN interfaces. This allows the communication code to run
 * continuously in the background.
 */
void Board320_240::commLoop()
{
  if (commInterface == nullptr || liveData->params.stopCommandQueue || commInterface->isSuspended())
  {
    return;
  }

  // Start timing
  int64_t startTime3 = esp_timer_get_time();

  // Read data from BLE/CAN
  commInterface->mainLoop();

  int64_t endTime3 = esp_timer_get_time();

  // Calculate duration
  int64_t duration3 = endTime3 - startTime3;

  // Print the duration using syslog
  // Use String constructor to convert int64_t to String
  // syslog->println("Time taken by function: commLoop() " + String(duration3) + " microseconds");
}

/**
 * Board loop function.
 * This function runs in the main loop to handle touch events
 * and other board specific tasks.
 */
void Board320_240::boardLoop()
{
}

void Board320_240::recordContributeSample()
{
  const time_t nowTime = liveData->params.currentTime;
  if (nowTime <= 0)
  {
    return;
  }
  if (lastContributeSampleTime != 0 && (nowTime - lastContributeSampleTime) < kContributeSampleIntervalSec)
  {
    return;
  }
  lastContributeSampleTime = nowTime;

  ContributeMotionSample motionSample{};
  motionSample.time = nowTime;
  if (isGpsFixUsable(liveData))
  {
    motionSample.hasGpsFix = true;
    motionSample.lat = roundToPrecision(liveData->params.gpsLat, kContributeGpsCoordPrecision);
    motionSample.lon = roundToPrecision(liveData->params.gpsLon, kContributeGpsCoordPrecision);
  }
  float currentSpeedKmh = liveData->params.speedKmh;
  if (currentSpeedKmh < 0 && liveData->params.speedKmhGPS >= 0)
  {
    currentSpeedKmh = liveData->params.speedKmhGPS;
  }
  motionSample.speedKmh = roundToPrecision(currentSpeedKmh, 10.0f);
  if (liveData->params.gpsHeadingDeg >= 0)
  {
    motionSample.headingDeg = roundToPrecision(liveData->params.gpsHeadingDeg, 10.0f);
  }
  motionSample.cellMinV = roundToPrecision(liveData->params.batCellMinV, 1000.0f);
  motionSample.cellMaxV = roundToPrecision(liveData->params.batCellMaxV, 1000.0f);
  motionSample.cellMinNo = liveData->params.batCellMinVNo;

  contributeMotionSamples[contributeMotionSampleNext] = motionSample;
  contributeMotionSampleNext = (contributeMotionSampleNext + 1) % kContributeSampleSlots;
  if (contributeMotionSampleCount < kContributeSampleSlots)
  {
    contributeMotionSampleCount++;
  }

  if (liveData->params.chargingOn)
  {
    ContributeChargingSample chargingSample{};
    chargingSample.time = nowTime;
    chargingSample.soc = roundToPrecision(liveData->params.socPerc, 10.0f);
    chargingSample.batV = roundToPrecision(liveData->params.batVoltage, 10.0f);
    chargingSample.batA = roundToPrecision(liveData->params.batPowerAmp, 10.0f);
    chargingSample.powKw = roundToPrecision(liveData->params.batPowerKw, 1000.0f);

    contributeChargingSamples[contributeChargingSampleNext] = chargingSample;
    contributeChargingSampleNext = (contributeChargingSampleNext + 1) % kContributeSampleSlots;
    if (contributeChargingSampleCount < kContributeSampleSlots)
    {
      contributeChargingSampleCount++;
    }
  }
}

Board320_240::ContributeChargingEvent Board320_240::captureContributeChargingEventSnapshot(time_t eventTime) const
{
  ContributeChargingEvent event{};
  event.valid = true;
  event.time = eventTime;
  event.soc = roundToPrecision(liveData->params.socPerc, 10.0f);
  event.batV = roundToPrecision(liveData->params.batVoltage, 10.0f);
  event.batA = roundToPrecision(liveData->params.batPowerAmp, 10.0f);
  event.cellMinV = roundToPrecision(liveData->params.batCellMinV, 1000.0f);
  event.cellMaxV = roundToPrecision(liveData->params.batCellMaxV, 1000.0f);
  event.cellMinNo = liveData->params.batCellMinVNo;
  event.batMinC = roundToPrecision(liveData->params.batMinC, 10.0f);
  event.batMaxC = roundToPrecision(liveData->params.batMaxC, 10.0f);
  event.cecKWh = roundToPrecision(liveData->params.cumulativeEnergyChargedKWh, 1000.0f);
  event.cedKWh = roundToPrecision(liveData->params.cumulativeEnergyDischargedKWh, 1000.0f);
  return event;
}

void Board320_240::handleContributeChargingTransitions()
{
  const time_t nowTime = liveData->params.currentTime;
  ContributeChargingEvent snapshot = captureContributeChargingEventSnapshot(nowTime);

  if (liveData->params.chargingOn)
  {
    contributeLastDuringCharge = snapshot;
  }
  else
  {
    contributeLastBeforeCharge = snapshot;
  }

  if (liveData->params.chargingOn && !lastChargingOn)
  {
    ContributeChargingEvent startEvent = contributeLastBeforeCharge.valid ? contributeLastBeforeCharge : snapshot;
    startEvent.valid = true;
    startEvent.time = nowTime;
    contributeChargingStartEvent = startEvent;
  }
  if (!liveData->params.chargingOn && lastChargingOn)
  {
    ContributeChargingEvent endEvent = contributeLastDuringCharge.valid ? contributeLastDuringCharge : snapshot;
    endEvent.valid = true;
    endEvent.time = nowTime;
    contributeChargingEndEvent = endEvent;
  }
}

bool Board320_240::buildContributePayloadV2(String &outJson, bool useReadableTsForSd)
{
  (void)useReadableTsForSd;
  HeapCapsJsonDocument jsonData(kContributeJsonDocCapacity);
  const time_t nowTime = liveData->params.currentTime;
  const String contributeKey = ensureContributeKey();
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());
  const String readableTs = formatTimestampYyMmDdHhIiSs(nowTime);

  jsonData["ver"] = 2;
  if (readableTs.length() == 12)
  {
    jsonData["ts"] = readableTs;
  }
  else
  {
    jsonData["ts"] = "000000000000";
  }
  jsonData["key"] = contributeKey;
  jsonData["deviceId"] = hardwareDeviceId;
  jsonData["dev"] = getCompiledDeviceTypeForApi();
  jsonData["apiKey"] = liveData->settings.remoteApiKey;
  jsonData["register"] = 1;
  jsonData["carType"] = getCarModelAbrpStr(liveData->settings.carType);
  jsonData["carVin"] = liveData->params.carVin;
  jsonData["sd"] = (liveData->params.sdcardInit && liveData->params.sdcardRecording) ? 1 : 0;

  const char *gpsMode = "none";
  switch (liveData->settings.gpsModuleType)
  {
  case GPS_MODULE_TYPE_NEO_M8N:
    gpsMode = "m8n";
    break;
  case GPS_MODULE_TYPE_M5_GNSS:
    gpsMode = "m9n";
    break;
  case GPS_MODULE_TYPE_GPS_V21_GNSS:
    gpsMode = "v21";
    break;
  default:
    gpsMode = "none";
    break;
  }
  jsonData["gps"] = gpsMode;

  const char *commMode = "unknown";
  switch (liveData->settings.commType)
  {
  case COMM_TYPE_CAN_COMMU:
    commMode = "can";
    break;
  case COMM_TYPE_OBD2_BLE4:
    commMode = "ble4";
    break;
  default:
    commMode = "unknown";
    break;
  }
  jsonData["comm"] = commMode;

  jsonData["stoppedCan"] = liveData->params.stopCommandQueue ? 1 : 0;
  jsonData["ign"] = liveData->params.ignitionOn ? 1 : 0;
  jsonData["chg"] = liveData->params.chargingOn ? 1 : 0;
  jsonData["chgAc"] = liveData->params.chargerACconnected ? 1 : 0;
  jsonData["chgDc"] = liveData->params.chargerDCconnected ? 1 : 0;
  setJsonNumber(jsonData, "soc", liveData->params.socPerc, 1);
  if (liveData->params.socPercBms != -1)
  {
    setJsonNumber(jsonData, "socBms", liveData->params.socPercBms, 1);
  }
  setJsonNumber(jsonData, "soh", liveData->params.sohPerc, 1);
  setJsonNumber(jsonData, "powKw", liveData->params.batPowerKw, 3);
  setJsonNumber(jsonData, "powKwh100", liveData->params.batPowerKwh100, 3);
  setJsonNumber(jsonData, "batV", liveData->params.batVoltage, 1);
  setJsonNumber(jsonData, "batA", liveData->params.batPowerAmp, 1);
  setJsonNumber(jsonData, "auxV", liveData->params.auxVoltage, 1);
  setJsonNumber(jsonData, "auxA", liveData->params.auxCurrentAmp, 1);
  setJsonNumber(jsonData, "batMinC", liveData->params.batMinC, 1);
  setJsonNumber(jsonData, "batMaxC", liveData->params.batMaxC, 1);
  setJsonNumber(jsonData, "inC", liveData->params.indoorTemperature, 1);
  setJsonNumber(jsonData, "outC", liveData->params.outdoorTemperature, 1);
  setJsonNumber(jsonData, "spd", liveData->params.speedKmh, 1);
  if (liveData->params.speedKmhGPS >= 0)
  {
    setJsonNumber(jsonData, "gpsSpd", liveData->params.speedKmhGPS, 1);
  }
  if (liveData->params.gpsHeadingDeg >= 0)
  {
    setJsonNumber(jsonData, "hdg", liveData->params.gpsHeadingDeg, 1);
  }
  setJsonNumber(jsonData, "odoKm", liveData->params.odoKm, 1);
  setJsonNumber(jsonData, "cMinV", liveData->params.batCellMinV, 3);
  setJsonNumber(jsonData, "cMaxV", liveData->params.batCellMaxV, 3);
  jsonData["cMinNo"] = liveData->params.batCellMinVNo;
  setJsonNumber(jsonData, "cecKWh", liveData->params.cumulativeEnergyChargedKWh, 3);
  setJsonNumber(jsonData, "cedKWh", liveData->params.cumulativeEnergyDischargedKWh, 3);
  if (isGpsFixUsable(liveData))
  {
    setJsonNumber(jsonData, "lat", liveData->params.gpsLat, 6);
    setJsonNumber(jsonData, "lon", liveData->params.gpsLon, 6);
  }
  JsonArray motion;
  const uint8_t motionStartIndex = (contributeMotionSampleCount == kContributeSampleSlots) ? contributeMotionSampleNext : 0;
  for (uint8_t i = 0; i < contributeMotionSampleCount; i++)
  {
    uint8_t sampleIndex = (motionStartIndex + i) % kContributeSampleSlots;
    const ContributeMotionSample &sample = contributeMotionSamples[sampleIndex];
    if (sample.time == 0)
    {
      continue;
    }
    const time_t sampleAge = nowTime - sample.time;
    if (sampleAge < 0 || sampleAge > kContributeSampleWindowSec)
    {
      continue;
    }
    if (!sample.hasGpsFix)
    {
      continue;
    }
    if (motion.isNull())
    {
      motion = jsonData.createNestedArray("motion");
    }
    JsonObject row = motion.createNestedObject();
    row["t"] = static_cast<int32_t>(sample.time - nowTime);
    setJsonNumber(row, "lat", sample.lat, 6);
    setJsonNumber(row, "lon", sample.lon, 6);
    setJsonNumber(row, "spd", sample.speedKmh, 1);
    setJsonNumber(row, "hdg", sample.headingDeg, 1);
    setJsonNumber(row, "cMinV", sample.cellMinV, 3);
    setJsonNumber(row, "cMaxV", sample.cellMaxV, 3);
    row["cMinNo"] = sample.cellMinNo;
  }

  if (liveData->params.chargingOn)
  {
    JsonArray charging = jsonData.createNestedArray("charging");
    const uint8_t chargingStartIndex = (contributeChargingSampleCount == kContributeSampleSlots) ? contributeChargingSampleNext : 0;
    for (uint8_t i = 0; i < contributeChargingSampleCount; i++)
    {
      uint8_t sampleIndex = (chargingStartIndex + i) % kContributeSampleSlots;
      const ContributeChargingSample &sample = contributeChargingSamples[sampleIndex];
      if (sample.time == 0)
      {
        continue;
      }
      const time_t sampleAge = nowTime - sample.time;
      if (sampleAge < 0 || sampleAge > kContributeSampleWindowSec)
      {
        continue;
      }
      JsonObject row = charging.createNestedObject();
      row["t"] = static_cast<int32_t>(sample.time - nowTime);
      setJsonNumber(row, "soc", sample.soc, 1);
      setJsonNumber(row, "batV", sample.batV, 1);
      setJsonNumber(row, "batA", sample.batA, 1);
      setJsonNumber(row, "powKw", sample.powKw, 3);
    }
  }

  if (contributeChargingStartEvent.valid)
  {
    const time_t eventAge = nowTime - contributeChargingStartEvent.time;
    if (eventAge >= 0 && eventAge <= kContributeSampleWindowSec)
    {
      JsonObject chargingStart = jsonData.createNestedObject("chargingStart");
      chargingStart["time"] = contributeChargingStartEvent.time;
      setJsonNumber(chargingStart, "soc", contributeChargingStartEvent.soc, 1);
      setJsonNumber(chargingStart, "batV", contributeChargingStartEvent.batV, 1);
      setJsonNumber(chargingStart, "batA", contributeChargingStartEvent.batA, 1);
      setJsonNumber(chargingStart, "cMinV", contributeChargingStartEvent.cellMinV, 3);
      setJsonNumber(chargingStart, "cMaxV", contributeChargingStartEvent.cellMaxV, 3);
      chargingStart["cMinNo"] = contributeChargingStartEvent.cellMinNo;
      setJsonNumber(chargingStart, "batMinC", contributeChargingStartEvent.batMinC, 1);
      setJsonNumber(chargingStart, "batMaxC", contributeChargingStartEvent.batMaxC, 1);
      setJsonNumber(chargingStart, "cecKWh", contributeChargingStartEvent.cecKWh, 3);
      setJsonNumber(chargingStart, "cedKWh", contributeChargingStartEvent.cedKWh, 3);
    }
  }

  if (contributeChargingEndEvent.valid)
  {
    const time_t eventAge = nowTime - contributeChargingEndEvent.time;
    if (eventAge >= 0 && eventAge <= kContributeSampleWindowSec)
    {
      JsonObject chargingEnd = jsonData.createNestedObject("chargingEnd");
      chargingEnd["time"] = contributeChargingEndEvent.time;
      setJsonNumber(chargingEnd, "soc", contributeChargingEndEvent.soc, 1);
      setJsonNumber(chargingEnd, "batV", contributeChargingEndEvent.batV, 1);
      setJsonNumber(chargingEnd, "batA", contributeChargingEndEvent.batA, 1);
      setJsonNumber(chargingEnd, "cMinV", contributeChargingEndEvent.cellMinV, 3);
      setJsonNumber(chargingEnd, "cMaxV", contributeChargingEndEvent.cellMaxV, 3);
      chargingEnd["cMinNo"] = contributeChargingEndEvent.cellMinNo;
      setJsonNumber(chargingEnd, "batMinC", contributeChargingEndEvent.batMinC, 1);
      setJsonNumber(chargingEnd, "batMaxC", contributeChargingEndEvent.batMaxC, 1);
      setJsonNumber(chargingEnd, "cecKWh", contributeChargingEndEvent.cecKWh, 3);
      setJsonNumber(chargingEnd, "cedKWh", contributeChargingEndEvent.cedKWh, 3);
    }
  }

  uint8_t rawAdded = 0;
  uint8_t rawDropped = 0;
  for (uint8_t i = 0; i < liveData->contributeRawFrameCount; i++)
  {
    const LiveData::ContributeRawFrame &raw = liveData->contributeRawFrames[i];
    if (raw.key[0] == '\0' || raw.value[0] == '\0')
    {
      continue;
    }

    if (rawAdded >= kContributeRawFrameUploadMax)
    {
      rawDropped++;
      continue;
    }

    jsonData[String(raw.key)] = raw.value;
    if (kContributeIncludeRawLatency)
    {
      jsonData[String(raw.key) + "_ms"] = String(raw.latencyMs);
    }
    rawAdded++;
  }

  if (rawDropped > 0)
  {
    jsonData["rawDrop"] = rawDropped;
  }

  if (jsonData.overflowed())
  {
    syslog->println("Contribute JSON overflow (stability mode): payload fields trimmed");
  }

  outJson = "";
  size_t payloadLen = serializeJson(jsonData, outJson);
  return payloadLen > 0;
}

void Board320_240::syncContributeRelativeTimes(time_t offset)
{
  if (offset == 0)
  {
    return;
  }

  for (uint8_t i = 0; i < kContributeSampleSlots; i++)
  {
    if (contributeMotionSamples[i].time != 0)
    {
      contributeMotionSamples[i].time += offset;
    }
    if (contributeChargingSamples[i].time != 0)
    {
      contributeChargingSamples[i].time += offset;
    }
  }

  auto syncEventTime = [offset](ContributeChargingEvent &event)
  {
    if (event.valid && event.time != 0)
    {
      event.time += offset;
    }
  };
  syncEventTime(contributeLastBeforeCharge);
  syncEventTime(contributeLastDuringCharge);
  syncEventTime(contributeChargingStartEvent);
  syncEventTime(contributeChargingEndEvent);

  if (lastContributeSampleTime != 0)
  {
    lastContributeSampleTime += offset;
  }
  if (lastContributeSdRecordTime != 0)
  {
    lastContributeSdRecordTime += offset;
  }
}

/**
 * Main loop - primary thread
 */
void Board320_240::mainLoop()
{
  // Calculate FPS
  const uint32_t loopDurationMs = (millis() - mainLoopStart);
  displayFps = (loopDurationMs == 0 ? 0 : (1000.0f / loopDurationMs));
  mainLoopStart = millis();

  // Serial console commands
  processSerialConsole();

  // board loop
  boardLoop();

  ///////////////////////////////////////////////////////////////////////
  // Handle buttons
  // MIDDLE - menu select
  if (!isButtonPressed(pinButtonMiddle))
  {
    btnMiddlePressed = false;
  }
  else
  {
    if (!btnMiddlePressed)
    {
      btnMiddlePressed = true;
      liveData->params.lastButtonPushedTime = liveData->params.currentTime;
      tft.setRotation(liveData->settings.displayRotation);
      if (liveData->menuVisible)
      {
        menuItemClick();
      }
      else
      {
        showMenu();
      }
    }
  }
  // LEFT - screen rotate, menu
  if (!isButtonPressed(pinButtonLeft))
  {
    btnLeftPressed = false;
  }
  else
  {
    if (!btnLeftPressed)
    {
      btnLeftPressed = true;
      liveData->params.lastButtonPushedTime = liveData->params.currentTime;
      tft.setRotation(liveData->settings.displayRotation);
      // Menu handling
      if (liveData->menuVisible)
      {
        menuMove(false);
      }
      else
      {
        liveData->params.displayScreen++;
        if (liveData->params.displayScreen > displayScreenCount - 1)
          liveData->params.displayScreen = 0; // rotate screens
        // Turn off display on screen 0
        setBrightness();
        redrawScreen();
      }
    }
  }
  // RIGHT - menu, debug screen rotation
  if (!isButtonPressed(pinButtonRight))
  {
    btnRightPressed = false;
  }
  else
  {
    if (!btnRightPressed)
    {
      btnRightPressed = true;
      liveData->params.lastButtonPushedTime = liveData->params.currentTime;
      tft.setRotation(liveData->settings.displayRotation);
      // Menu handling
      if (liveData->menuVisible)
      {
        menuMove(true);
      }
      else
      {
        // doAction
        if (liveData->params.displayScreen == SCREEN_SPEED)
        {
          liveData->params.displayScreen = SCREEN_HUD;
          tft.fillScreen(TFT_BLACK);
          redrawScreen();
        }
        else if (liveData->params.displayScreen == SCREEN_HUD)
        {
          liveData->params.displayScreen = SCREEN_SPEED;
          redrawScreen();
        }

        setBrightness();
      }
    }
  }
  // Both left&right button (hide menu)
  if (isButtonPressed(pinButtonLeft) && isButtonPressed(pinButtonRight))
  {
    hideMenu();
  }

  if (liveData->menuVisible &&
      liveData->params.currentTime - liveData->params.lastButtonPushedTime >= kMenuAutoHideTimeoutSec)
  {
    hideMenu();
  }

  const bool allowGpsProcessing = !(liveData->params.stopCommandQueue && liveData->settings.voltmeterEnabled == 1);
  if (allowGpsProcessing)
  {
    // GPS process
    // Start timing
    int64_t startTime4 = esp_timer_get_time();

    if (gpsHwUart != NULL)
    {
      unsigned long start = millis();
      if (gpsHwUart->available())
      {
        do
        {
          int ch = gpsHwUart->read();
          if (ch != -1)
            syslog->infoNolf(DEBUG_GPS, char(ch));
          gps.encode(ch);
        } while (gpsHwUart->available());
        syncGPS();
      }
    }
    else
    {
      // MEB CAR GPS
      if (liveData->params.gpsValid && liveData->params.gpsLat != -1.0 && liveData->params.gpsLon != -1.0)
        calcAutomaticBrightnessLatLon();
    }
    if (liveData->params.setGpsTimeFromCar != 0)
    {
      struct tm *tmm = gmtime(&liveData->params.setGpsTimeFromCar);
      tmm->tm_isdst = 0;
      setGpsTime(tmm->tm_year + 1900, tmm->tm_mon + 1, tmm->tm_mday, tmm->tm_hour, tmm->tm_min, tmm->tm_sec);
      liveData->params.setGpsTimeFromCar = 0;
    }

    int64_t endTime4 = esp_timer_get_time();
    // Calculate duration
    int64_t duration4 = endTime4 - startTime4;

    // Print the duration using syslog
    // Use String constructor to convert int64_t to String
    // syslog->println("Time taken by function: GPS loop " + String(duration4) + " microseconds");
  }

  // currentTime
  struct tm now = cachedNow;
  const uint32_t nowMs = millis();
  if (lastTimeUpdateMs == 0 || (nowMs - lastTimeUpdateMs) >= 1000)
  {
    if (getLocalTime(&now, 0))
    {
      cachedNow = now;
      cachedNowEpoch = mktime(&cachedNow);
    }
    else if (cachedNowEpoch != 0 && lastTimeUpdateMs != 0)
    {
      const uint32_t deltaSec = (nowMs - lastTimeUpdateMs) / 1000U;
      if (deltaSec > 0)
      {
        cachedNowEpoch += deltaSec;
        localtime_r(&cachedNowEpoch, &cachedNow);
      }
    }
    if (cachedNowEpoch != 0)
    {
      liveData->params.currentTime = cachedNowEpoch;
    }
    else
    {
      // Fallback to uptime seconds when RTC/NTP/GPS time isn't available yet.
      liveData->params.currentTime = nowMs / 1000U;
    }
    lastTimeUpdateMs = nowMs;
  }

  // Periodic automatic brightness recalculation (handles sunrise/sunset without new GPS fix)
  if (liveData->settings.lcdBrightness == 0 &&
      liveData->params.gpsLat != -1.0 &&
      liveData->params.gpsLon != -1.0 &&
      liveData->params.currentTime != 0)
  {
    static time_t lastAutoBrightnessCalc = 0;
    const time_t nowTime = liveData->params.currentTime;
    if (lastAutoBrightnessCalc == 0 || (nowTime - lastAutoBrightnessCalc) >= 60)
    {
      lastAutoBrightnessCalc = nowTime;
      calcAutomaticBrightnessLatLon();
    }
  }

  // Check and eventually reconnect WIFI connection
  const bool wifiEnabled = (liveData->settings.wifiEnabled == 1);
  const bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected)
  {
    liveData->params.wifiLastConnectedTime = liveData->params.currentTime;
  }
  if (wifiConnected && !lastWifiConnected)
  {
    checkFirmwareVersionOnServer();
  }
  lastWifiConnected = wifiConnected;

  pollEvdashPairingStatus();

  const bool allowWifiFallback = (!liveData->params.stopCommandQueue &&
                                  !liveData->params.wifiApMode &&
                                  wifiEnabled &&
                                  liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI);
  if (allowWifiFallback)
  {
    const bool disconnectedTooLong = (!wifiConnected &&
                                      liveData->params.currentTime - liveData->params.wifiLastConnectedTime > 60);
    const bool netFailedTooLong = (wifiConnected &&
                                   liveData->params.netFailureStartTime != 0 &&
                                   liveData->params.netFailureCount >= kNetFailureFallbackCount &&
                                   (liveData->params.currentTime - liveData->params.netFailureStartTime) > kNetFailureFallbackSec);
    if (disconnectedTooLong || netFailedTooLong)
    {
      wifiFallback();
    }
  }

  // SIM800L, WiFI remote upload, ABRP remote upload, MQTT
  netLoop();

  // SD card recording
  int64_t startTime5 = esp_timer_get_time();
  const bool sdcardJsonV2 = true;
  const bool sdcardWriteTick = sdcardJsonV2 ? true : liveData->params.sdcardCanNotify;
  const bool sdcardHasPayload =
      sdcardJsonV2 ? !isContributeV2SnapshotEffectivelyEmpty(liveData)
                   : (liveData->params.odoKm != -1 && liveData->params.socPerc != -1);
  if (!liveData->params.stopCommandQueue && liveData->params.sdcardInit && liveData->params.sdcardRecording && sdcardWriteTick &&
      sdcardHasPayload)
  {
    const size_t sdcardFlushSize = 2048;
    const uint32_t sdcardIntervalMs = static_cast<uint32_t>(liveData->settings.sdcardLogIntervalSec) * 1000U;
    const char *sdcardOpFilenameFmt = sdcardJsonV2 ? "/%llu_v2.json" : "/%llu.json";
    const char *sdcardGpsFilenameFmt = sdcardJsonV2 ? "/%y%m%d%H%M_v2.json" : "/%y%m%d%H%M.json";
    const size_t sdcardGpsFilenameMinLength = sdcardJsonV2 ? 18 : 15;

    // create filename
    if (liveData->params.operationTimeSec > 0 && strlen(liveData->params.sdcardFilename) == 0)
    {
      sprintf(liveData->params.sdcardFilename, sdcardOpFilenameFmt, uint64_t(liveData->params.operationTimeSec / 60));
      syslog->print("Log filename by opTimeSec: ");
      syslog->println(liveData->params.sdcardFilename);
    }
    if (liveData->params.currTimeSyncWithGps && strlen(liveData->params.sdcardFilename) < sdcardGpsFilenameMinLength)
    {
      if (cachedNowEpoch == 0)
      {
        getLocalTime(&now, 0);
      }
      strftime(liveData->params.sdcardFilename, sizeof(liveData->params.sdcardFilename), sdcardGpsFilenameFmt, &now);
      syslog->print("Log filename by GPS: ");
      syslog->println(liveData->params.sdcardFilename);
    }

    // append buffer, clear buffer & notify state
    if (strlen(liveData->params.sdcardFilename) != 0)
    {
      liveData->params.sdcardCanNotify = false;
      if (sdcardJsonV2)
      {
        const bool minuteTick = (lastContributeSdRecordTime == 0) ||
                                ((liveData->params.currentTime - lastContributeSdRecordTime) >= kContributeSampleWindowSec);
        const bool contributeOnlineNow =
            (liveData->settings.contributeData == 1) &&
            (liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI) &&
            (liveData->settings.wifiEnabled == 1) &&
            (WiFi.status() == WL_CONNECTED) &&
            liveData->params.netAvailable &&
            !isMobileRelayClientConnected();
        if (minuteTick && !contributeOnlineNow)
        {
          String jsonLine;
          if (buildContributePayloadV2(jsonLine, true))
          {
            jsonLine += ",\n";
            sdcardRecordBuffer += jsonLine;
            lastContributeSdRecordTime = liveData->params.currentTime;
          }
        }
      }
      else
      {
        String jsonLine;
        serializeParamsToJson(jsonLine);
        jsonLine += ",\n";
        sdcardRecordBuffer += jsonLine;
      }

      const bool timeToFlush = (sdcardIntervalMs > 0U) && ((nowMs - liveData->params.sdcardLastFlushMs) >= sdcardIntervalMs);
      const bool sizeToFlush = sdcardRecordBuffer.length() >= sdcardFlushSize;
      if ((timeToFlush || sizeToFlush) && sdcardRecordBuffer.length() > 0)
      {
        if (sdcardJsonV2 &&
            rotateSdV2FileIfNeeded(liveData->params.sdcardFilename,
                                   sizeof(liveData->params.sdcardFilename),
                                   sdcardRecordBuffer.length()))
        {
          syslog->print("SD v2 rollover file: ");
          syslog->println(liveData->params.sdcardFilename);
        }

        File file = SD.open(liveData->params.sdcardFilename, FILE_APPEND);
        if (!file)
        {
          syslog->println("Failed to open file for appending");
          file = SD.open(liveData->params.sdcardFilename, FILE_WRITE);
        }
        if (!file)
        {
          syslog->println("Failed to create file");
        }
        if (file)
        {
          syslog->info(DEBUG_SDCARD, "Save buffer to SD card");
          file.print(sdcardRecordBuffer);
          file.close();
          sdcardRecordBuffer = "";
          liveData->params.sdcardLastFlushMs = nowMs;
        }
      }
    }
  }

  int64_t endTime5 = esp_timer_get_time();

  // Calculate duration
  int64_t duration5 = endTime5 - startTime5;

  // Print the duration using syslog
  // Use String constructor to convert int64_t to String
  // syslog->println("Time taken by function: SD card write loop " + String(duration5) + " microseconds");

  // Read voltmeter INA3221 (if enabled)
  if (liveData->settings.voltmeterEnabled == 1 && liveData->params.currentTime - liveData->params.lastVoltageReadTime > 5)
  {
    liveData->params.auxVoltage = ina3221.getBusVoltage_V(1);
    liveData->params.lastVoltageReadTime = liveData->params.currentTime;
    if (liveData->params.auxVoltage > liveData->settings.voltmeterSleep)
    {
      liveData->params.lastVoltageOkTime = liveData->params.currentTime;
    }

    // Protect AUX battery in screen only mode
    if (liveData->settings.sleepModeLevel == SLEEP_MODE_SCREEN_ONLY &&
        liveData->params.auxVoltage > 5 && liveData->params.auxVoltage < liveData->settings.voltmeterCutOff)
    {
      syslog->print("AUX voltage under cut-off voltage: ");
      syslog->println(liveData->settings.voltmeterCutOff);
      shutdownDevice();
    }

    // Calculate AUX perc for ioniq2018
    if (liveData->settings.carType == CAR_HYUNDAI_IONIQ_2018)
    {
      float tmpAuxPerc = (float)(liveData->params.auxVoltage - 11.6) * 100 / (float)(12.8 - 11.6); // min 11.6V; max: 12.8V
      liveData->params.auxPerc = ((tmpAuxPerc > 100) ? 100 : ((tmpAuxPerc < 0) ? 0 : tmpAuxPerc));
    }
  }

  const bool recentlyCharging =
      (liveData->params.lastChargingOnTime != 0 &&
       liveData->params.currentTime >= liveData->params.lastChargingOnTime &&
       (liveData->params.currentTime - liveData->params.lastChargingOnTime) <= kChargingQueueHoldSec);
  const bool chargingActiveForQueue =
      (liveData->params.chargingOn ||
       liveData->params.chargerACconnected ||
       liveData->params.chargerDCconnected ||
       recentlyCharging);

  // Reset sentry session when car becomes active
  if (liveData->params.ignitionOn || chargingActiveForQueue)
  {
    if (liveData->params.sentrySessionActive)
    {
      liveData->params.sentrySessionActive = false;
      liveData->params.motionWakeLocked = false;
      liveData->params.gpsWakeCount = 0;
      liveData->params.gyroWakeCount = 0;
      liveData->params.motionWakeLastTime = 0;
    }
  }

  // Wake up from stopped command queue
  //  - ignitions on and aux >= 11.5v
  //  - ina3221 & voltage is >= 14V (DCDC is running)
  //  - gps speed >= 5kmh & 4+ satellites (only when voltmeter is disabled)
  //  - gyro motion (only when voltmeter is disabled)
  const bool queueStopped = liveData->params.stopCommandQueue;
  const bool queueSleeping = (queueStopped || liveData->params.stopCommandQueueTime != 0);
  const bool allowMotionWake = (liveData->settings.voltmeterEnabled == 0);
  static uint8_t gpsWakeConfirmCount = 0;
  static uint8_t gyroWakeConfirmCount = 0;

  if (!queueStopped)
  {
    gpsWakeConfirmCount = 0;
    gyroWakeConfirmCount = 0;

    // Charging started while autostop was preparing: cancel pending stop timer.
    if (liveData->params.stopCommandQueueTime != 0 && chargingActiveForQueue)
    {
      liveData->continueWithCommandQueue();
    }
  }

  if (queueSleeping)
  {
    if (liveData->params.motionWakeLastTime == 0)
    {
      liveData->params.motionWakeLastTime = liveData->params.currentTime;
    }
    else if (liveData->params.currentTime - liveData->params.motionWakeLastTime >= kMotionWakeResetSec)
    {
      liveData->params.gpsWakeCount = 0;
      liveData->params.gyroWakeCount = 0;
      liveData->params.motionWakeLocked = false;
      liveData->params.motionWakeLastTime = liveData->params.currentTime;
    }
  }
  const uint16_t maxGpsWakePerSession = 1;
  const uint16_t maxGyroWakePerSession = 5;
  const bool gpsWakeRemaining = (liveData->params.gpsWakeCount < maxGpsWakePerSession);
  const bool gyroWakeRemaining = (liveData->params.gyroWakeCount < maxGyroWakePerSession);
  const bool motionWakeLocked = (!gpsWakeRemaining && !gyroWakeRemaining);
  liveData->params.motionWakeLocked = motionWakeLocked;
  const bool motionWakeAllowed = allowMotionWake && !motionWakeLocked;
  const bool gpsWakeCandidate =
      motionWakeAllowed && gpsWakeRemaining &&
      (liveData->params.gpsValid && liveData->params.speedKmhGPS >= 5 && liveData->params.gpsSat >= 4); // 5 floor parking house, satelites 5 & gps speed = 274kmh :/
  const bool gyroWakeCandidate = motionWakeAllowed && gyroWakeRemaining && liveData->params.gyroSensorMotion;
  // Consume the motion latch: each pass then answers "any motion since the last
  // check", so all ~20 IMU samples of a Sentry idle window count, not just the last.
  liveData->params.gyroSensorMotion = false;

  if (queueStopped && gpsWakeCandidate)
  {
    if (gpsWakeConfirmCount < kGpsWakeConfirmSamples)
      gpsWakeConfirmCount++;
  }
  else
  {
    gpsWakeConfirmCount = 0;
  }

  if (queueStopped && gyroWakeCandidate)
  {
    if (gyroWakeConfirmCount < kGyroWakeConfirmSamples)
      gyroWakeConfirmCount++;
  }
  else if (!queueStopped)
  {
    gyroWakeConfirmCount = 0;
  }
  else if (gyroWakeConfirmCount > 0)
  {
    // Decay instead of hard reset: real driving over speed bumps or stop-and-go
    // produces intermittent motion windows; a single quiet second must not throw
    // away the whole confirmation streak, or the wake never accumulates.
    gyroWakeConfirmCount--;
  }

  const bool gpsWake = queueStopped && (gpsWakeConfirmCount >= kGpsWakeConfirmSamples);
  const bool gyroWake = queueStopped && (gyroWakeConfirmCount >= kGyroWakeConfirmSamples);
  const bool motionWake = gpsWake || gyroWake;
  if (queueStopped &&
      ((liveData->params.ignitionOn && (liveData->params.auxVoltage <= 3 || liveData->params.auxVoltage >= 11.5)) ||
       chargingActiveForQueue ||
       (liveData->settings.voltmeterEnabled == 1 && liveData->params.auxVoltage > 14.0) ||
       motionWake))
  {
    if (motionWake)
    {
      if (gpsWake)
      {
        liveData->params.gpsWakeCount++;
      }
      if (gyroWake)
      {
        liveData->params.gyroWakeCount++;
      }
      liveData->params.motionWakeLastTime = liveData->params.currentTime;
      liveData->params.motionWakeLocked =
          (liveData->params.gpsWakeCount >= maxGpsWakePerSession &&
           liveData->params.gyroWakeCount >= maxGyroWakePerSession);
    }
    gpsWakeConfirmCount = 0;
    gyroWakeConfirmCount = 0;
    liveData->continueWithCommandQueue();
    if (commInterface->isSuspended())
    {
      commInterface->resumeDevice();
    }

    // Some APs drop ESP32 station during prolonged Sentry inactivity.
    // Force a reconnect immediately after wake so recovery does not require reboot.
    const bool shouldReconnectWifiAfterWake =
        (!liveData->params.wifiApMode &&
         liveData->settings.wifiEnabled == 1 &&
         WiFi.status() != WL_CONNECTED);
    if (shouldReconnectWifiAfterWake)
    {
      syslog->println("Sentry wake: WiFi disconnected, forcing reconnect.");
      WiFi.enableSTA(true);
      WiFi.mode(WIFI_STA);

      if (liveData->params.wifiActiveIndex > 0)
      {
        wifiSwitchToIndex(liveData->params.wifiActiveIndex);
      }
      else
      {
        wifiSwitchToMain();
      }
    }
  }
  // Stop command queue
  //  - automatically turns off CAN scanning after 1-2 minutes of inactivity
  //  - ignition is off
  //  - AUX voltage is under 11.5V
  const time_t doorStateStaleAfterSec = 15;
  const bool doorStateStale = (liveData->params.currentTime - liveData->params.lastCanbusResponseTime > doorStateStaleAfterSec);
  const bool doorsClosed = (!liveData->params.leftFrontDoorOpen &&
                            !liveData->params.rightFrontDoorOpen &&
                            !liveData->params.trunkDoorOpen);
  const bool doorsOk = (doorsClosed || doorStateStale);
  const bool parkingLikely =
      (!liveData->params.ignitionOn &&
       !chargingActiveForQueue &&
       doorsOk);
  const bool lowAuxVoltage = (!chargingActiveForQueue &&
                              liveData->params.auxVoltage > 3 &&
                              liveData->params.auxVoltage < 11.5);
  const bool autoStopByCanSignals = (parkingLikely || lowAuxVoltage);
  const bool longParkingCandidate =
      (liveData->params.currentTime != 0 &&
       !chargingActiveForQueue &&
       !liveData->params.forwardDriveMode &&
       !liveData->params.reverseDriveMode &&
       (liveData->params.stopCommandQueueTime != 0 ||
        liveData->params.stopCommandQueue ||
        liveData->params.parkModeOrNeutral));

  if (longParkingCandidate)
  {
    if (liveData->params.parkedModeStartTime == 0)
    {
      liveData->params.parkedModeStartTime = liveData->params.currentTime;
    }
    else if (!liveData->params.clearDrivingStatsOnNextDrive &&
             liveData->params.currentTime - liveData->params.parkedModeStartTime >= kLongParkingClearSec)
    {
      liveData->params.clearDrivingStatsOnNextDrive = true;
    }
  }
  else if (chargingActiveForQueue)
  {
    liveData->params.parkedModeStartTime = 0;
    liveData->params.clearDrivingStatsOnNextDrive = false;
  }
  else if (!liveData->params.clearDrivingStatsOnNextDrive &&
           (liveData->params.forwardDriveMode || liveData->params.reverseDriveMode))
  {
    liveData->params.parkedModeStartTime = 0;
  }

  // Fallback when CAN never yields a valid response (e.g. adapter/config issue):
  // still allow Sentry autostop after a grace period so AUX battery is protected.
  const time_t autoStopWithoutCanGraceSec = 180;
  const bool canDataMissingTooLong =
      (!liveData->params.getValidResponse &&
       (liveData->params.currentTime - liveData->params.wakeUpTime > autoStopWithoutCanGraceSec));
  const bool dcDcLikelyRunning = (liveData->params.auxVoltage >= 13.8);
  const bool autoStopFallbackSafe = (parkingLikely && !dcDcLikelyRunning);

  if (liveData->settings.commandQueueAutoStop == 1 &&
      ((liveData->params.getValidResponse && autoStopByCanSignals) ||
       (canDataMissingTooLong && autoStopFallbackSafe)))
  {
    if (canDataMissingTooLong && !liveData->params.getValidResponse && liveData->params.stopCommandQueueTime == 0)
    {
      syslog->println("Sentry fallback: no valid CAN data, preparing autostop.");
    }
    liveData->prepareForStopCommandQueue();
  }
  if (!liveData->params.stopCommandQueue &&
      !chargingActiveForQueue &&
      ((liveData->params.stopCommandQueueTime != 0 && liveData->params.currentTime - liveData->params.stopCommandQueueTime > 60) ||
       (liveData->params.auxVoltage > 3 && liveData->params.auxVoltage < 11.0)))
  {
    liveData->params.stopCommandQueue = true;
    commInterface->suspendDevice();
    syslog->println("CAN Command queue stopped...");
  }
  updateGpsV21PpsMode();

  // Descrease loop fps
  if (liveData->params.stopCommandQueue)
  {
    const uint32_t idleWaitStartMs = millis();
    while (liveData->params.stopCommandQueue && (millis() - idleWaitStartMs) < 1000UL)
    {
      processSerialConsole();
      delay(kSentryIdleSliceMs);
      boardLoop();

      // Keep Sentry low-power pacing, but poll wake inputs often enough so touch wake feels immediate.
      isButtonPressed(pinButtonMiddle);
      if (!liveData->params.stopCommandQueue)
        break;
      isButtonPressed(pinButtonLeft);
      if (!liveData->params.stopCommandQueue)
        break;
      isButtonPressed(pinButtonRight);
    }
  }

  // Automatic sleep after inactivity
  if (liveData->params.currentTime - liveData->params.lastIgnitionOnTime > 10 &&
      liveData->settings.sleepModeLevel >= SLEEP_MODE_SCREEN_ONLY &&
      liveData->params.currentTime - liveData->params.lastButtonPushedTime > 30 &&
      (liveData->params.currentTime - liveData->params.wakeUpTime > 60))
  {
    turnOffScreen();
  }
  else
  {
    setBrightness();
  }

  // Read data from BLE/CAN
  commLoop();

  // force redraw (min 1 sec update; slower while in Sentry)
  const time_t redrawIntervalSec = liveData->params.stopCommandQueue ? 2 : 1;
  if (!screenSwipePreviewActive &&
      (liveData->params.currentTime - lastRedrawTime >= redrawIntervalSec || liveData->redrawScreenRequested))
  {
    redrawScreen();
  }

  // Calculating avg.speed and time in forward mode
  if (liveData->params.odoKm != -1 && forwardDriveOdoKmLast == -1)
  {
    forwardDriveOdoKmLast = liveData->params.odoKm;
  }
  if (liveData->params.forwardDriveMode != lastForwardDriveMode)
  {
    if (liveData->params.forwardDriveMode)
    {
      if (forwardDriveOdoKmStart != -1 ||
          (liveData->params.odoKm != -1 && forwardDriveOdoKmLast != -1 && liveData->params.odoKm != forwardDriveOdoKmLast))
      {
        if (forwardDriveOdoKmStart == -1)
          forwardDriveOdoKmStart = liveData->params.odoKm;
        lastForwardDriveModeStart = liveData->params.currentTime;
        lastForwardDriveMode = liveData->params.forwardDriveMode;
      }
    }
    else
    {
      if (lastForwardDriveModeStart != 0)
      {
        previousForwardDriveModeTotal = previousForwardDriveModeTotal + (liveData->params.currentTime - lastForwardDriveModeStart);
        lastForwardDriveModeStart = 0;
      }
      lastForwardDriveMode = liveData->params.forwardDriveMode;
    }
  }
  liveData->params.timeInForwardDriveMode = previousForwardDriveModeTotal +
                                            (lastForwardDriveModeStart == 0 ? 0 : liveData->params.currentTime - lastForwardDriveModeStart);
  if (liveData->params.odoKm != -1 && forwardDriveOdoKmLast != -1 && liveData->params.odoKm != forwardDriveOdoKmLast && liveData->params.timeInForwardDriveMode > 0)
  {
    forwardDriveOdoKmLast = liveData->params.odoKm;
    liveData->params.avgSpeedKmh = /*(double)*/ (liveData->params.odoKm - forwardDriveOdoKmStart) /
                                   (/*(double)*/ liveData->params.timeInForwardDriveMode / 3600.0);
  }

  // Automatic reset charging data or clear drive stats after charging / long parking transition.
  if (liveData->params.chargingOn && !lastChargingOn)
  {
    liveData->params.chargingStartTime = liveData->params.currentTime;
  }
  handleContributeChargingTransitions();
  lastChargingOn = liveData->params.chargingOn;
  recordContributeSample();

  if (liveData->params.chargingOn && liveData->params.carMode != CAR_MODE_CHARGING)
  {
    liveData->clearDrivingAndChargingStats(CAR_MODE_CHARGING);
  }
  else if ((liveData->params.clearDrivingStatsOnNextDrive &&
            liveData->params.forwardDriveMode) ||
           (liveData->params.forwardDriveMode &&
            (liveData->params.speedKmh > 15 || (liveData->params.speedKmhGPS > 15 && liveData->params.gpsSat >= 4)) &&
            liveData->params.carMode != CAR_MODE_DRIVE))
  {
    liveData->clearDrivingAndChargingStats(CAR_MODE_DRIVE);
  }
  /*else if (!liveData->params.chargingOn && !liveData->params.forwardDriveMode && liveData->params.carMode != CAR_MODE_NONE &&
           (!(liveData->params.speedKmh > 15 || (liveData->params.speedKmhGPS > 15 && liveData->params.gpsSat >= 4))) && liveData->params.currentTime - liveData->params.carModeChanged > 1800 &&
           liveData->params.currentTime - liveData->params.carModeChanged < 10 * 24 * 3600)
  {
    liveData->clearDrivingAndChargingStats(CAR_MODE_NONE);
  }*/
}

/**
 * Set the RTC time using the provided GPS time.
 * Converts the provided date/time components into a UNIX timestamp,
 * sets the system time using settimeofday(), and syncs other times.
 * Also sets the time on the M5Stack Core2 RTC module if being used.
 */
void Board320_240::setGpsTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds)
{
  liveData->params.currTimeSyncWithGps = true;

  struct tm tm = {0};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = seconds;
  time_t t = mktime(&tm);
  syslog->printf("%02d%02d%02d%02d%02d%02d\n", year - 2000, month, day, hour, minute, seconds);
  struct timeval now = {.tv_sec = t};
  settimeofday(&now, NULL);
  syncTimes(t);

  rtcWriteTime(t);
}

/**
 * Syncs related timestamp variables to the provided new time.
 * This is called after setting the system time, to update timestamp
 * variables that are relative to the current time. It calculates the
 * offset between the old current time and new current time, and adjusts
 * each timestamp variable by that offset.
 */
void Board320_240::syncTimes(time_t newTime)
{
  const time_t offset = newTime - liveData->params.currentTime;
  time_t *timeParams[] = {
      &liveData->params.chargingStartTime,
      &liveData->params.lastRemoteApiSent,
      &liveData->params.lastAbrpSent,
      &liveData->params.lastContributeSent,
      &liveData->params.lastSuccessNetSendTime,
      &liveData->params.lastButtonPushedTime,
      &liveData->params.wakeUpTime,
      &liveData->params.lastIgnitionOnTime,
      &liveData->params.stopCommandQueueTime,
      &liveData->params.motionWakeLastTime,
      &liveData->params.parkedModeStartTime,
      &liveData->params.lastChargingOnTime,
      &liveData->params.lastVoltageReadTime,
      &liveData->params.lastVoltageOkTime,
      &liveData->params.lastCanbusResponseTime};

  for (time_t *param : timeParams)
  {
    if (*param != 0)
    {
      *param = newTime - (liveData->params.currentTime - *param);
    }
  }

  syncContributeRelativeTimes(offset);

  // Reset avg speed counter
  lastForwardDriveModeStart = 0;
  lastForwardDriveMode = false;
}

/**
 * skipAdapterScan
 */
bool Board320_240::skipAdapterScan()
{
  return isButtonPressed(pinButtonMiddle) || isButtonPressed(pinButtonLeft) || isButtonPressed(pinButtonRight);
}

/**
 * Attempts to mount the SD card.
 *
 * Tries initializing the SD card multiple times if needed.
 * Checks the card type and size once mounted.
 * Updates liveData->params.sdcardInit if successful.
 *
 * @returns true if the SD card was mounted successfully, false otherwise.
 */
bool Board320_240::sdcardMount()
{
  // Check if SD card is already initialized
  if (liveData->params.sdcardInit)
  {
    syslog->println("SD card already mounted...");
    return true;
  }

  bool SdState = false;
  syslog->print("Initializing SD card...");
  SdState = SD.begin(TFCARD_CS_PIN, SPI, 40000000);
  if (SdState)
  {

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      syslog->println("No SD card attached");
      return false;
    }

    syslog->println("SD card found.");
    liveData->params.sdcardInit = true;

    syslog->print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
      syslog->println("MMC");
    }
    else if (cardType == CARD_SD)
    {
      syslog->println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
      syslog->println("SDHC");
    }
    else
    {
      syslog->println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    syslog->printf("SD Card Size: %lluMB\n", cardSize);

    if (liveData->settings.sdcardConsoleLogEnabled == 1 && !syslog->isSdLogging())
    {
      startSdcardConsoleLog();
    }

    return true;
  }

  syslog->println("Initialization failed!");

  return false;
}

/**
 * Toggles SD card recording on/off.
 *
 * Checks if SD card is initialized first.
 * Updates sdcardRecording parameter and filename as needed.
 */
void Board320_240::sdcardToggleRecording()
{
  if (!liveData->params.sdcardInit)
    return;

  syslog->println("Toggle SD card recording...");
  liveData->params.sdcardRecording = !liveData->params.sdcardRecording;
  if (liveData->params.sdcardRecording)
  {
    liveData->params.sdcardCanNotify = true;
    liveData->params.sdcardLastFlushMs = millis();
    lastContributeSdRecordTime = 0;
  }
  else
  {
    if (sdcardRecordBuffer.length() > 0 && strlen(liveData->params.sdcardFilename) != 0)
    {
      if (rotateSdV2FileIfNeeded(liveData->params.sdcardFilename,
                                 sizeof(liveData->params.sdcardFilename),
                                 sdcardRecordBuffer.length()))
      {
        syslog->print("SD v2 rollover file: ");
        syslog->println(liveData->params.sdcardFilename);
      }

      File file = SD.open(liveData->params.sdcardFilename, FILE_APPEND);
      if (!file)
      {
        syslog->println("Failed to open file for appending");
        file = SD.open(liveData->params.sdcardFilename, FILE_WRITE);
      }
      if (!file)
      {
        syslog->println("Failed to create file");
      }
      if (file)
      {
        syslog->info(DEBUG_SDCARD, "Save buffer to SD card");
        file.print(sdcardRecordBuffer);
        file.close();
      }
      sdcardRecordBuffer = "";
    }
    String tmpStr = "";
    tmpStr.toCharArray(liveData->params.sdcardFilename, tmpStr.length() + 1);
  }
}

void Board320_240::sdcardEraseLogs()
{
  if (!liveData->settings.sdcardEnabled || !sdcardMount())
  {
    displayMessage("SDCARD", "Not mounted");
    delay(2000);
    return;
  }

  if (liveData->params.sdcardRecording)
  {
    sdcardToggleRecording();
  }

  const bool wasConsoleLogging = syslog->isSdLogging();
  if (wasConsoleLogging)
  {
    stopSdcardConsoleLog();
  }

  File dir = SD.open("/");
  if (!dir || !dir.isDirectory())
  {
    displayMessage("SDCARD", "Open root failed");
    delay(2000);
    return;
  }

  uint16_t removed = 0;
  uint16_t failed = 0;
  while (true)
  {
    File entry = dir.openNextFile(FILE_READ);
    if (!entry)
    {
      break;
    }

    const bool isDir = entry.isDirectory();
    String fileName = String(entry.name());
    entry.close();

    if (isDir)
    {
      continue;
    }

    if (!fileName.endsWith(".json"))
    {
      continue;
    }

    if (SD.remove(fileName.c_str()))
    {
      removed++;
    }
    else
    {
      failed++;
    }
  }
  dir.close();

  // Also erase console logs in /logs
  if (SD.exists("/logs"))
  {
    File logsDir = SD.open("/logs");
    if (logsDir && logsDir.isDirectory())
    {
      while (true)
      {
        File entry = logsDir.openNextFile(FILE_READ);
        if (!entry)
        {
          break;
        }

        const bool isDir = entry.isDirectory();
        String fileName = String(entry.name());
        entry.close();

        if (isDir)
        {
          continue;
        }

        if (!fileName.endsWith(".log"))
        {
          continue;
        }

        String path = fileName;
        if (!path.startsWith("/logs/"))
        {
          if (path.startsWith("/"))
            path = "/logs" + path;
          else
            path = "/logs/" + path;
        }

        if (SD.remove(path.c_str()))
        {
          removed++;
        }
        else
        {
          failed++;
        }
      }
      logsDir.close();
    }
  }

  sdcardRecordBuffer = "";
  String tmpStr = "";
  tmpStr.toCharArray(liveData->params.sdcardFilename, tmpStr.length() + 1);
  tmpStr.toCharArray(liveData->params.sdcardAbrpFilename, tmpStr.length() + 1);

  if (wasConsoleLogging || liveData->settings.sdcardConsoleLogEnabled == 1)
  {
    startSdcardConsoleLog();
  }

  String msg1 = "Logs erased: " + String(removed);
  String msg2 = (failed == 0) ? "Done" : "Failed: " + String(failed);
  displayMessage(msg1.c_str(), msg2.c_str());
  delay(2000);
}

/**
 * Parse timestamp from log filename in format /logs/YYYY-MM-DD_HH_mm_ss.log
 */
static time_t parseSdLogFileTime(const char *name)
{
  const char *p = strrchr(name, '/');
  if (p)
    p++;
  else
    p = name;
  int y = 0, m = 0, d = 0, H = 0, M = 0, S = 0;
  if (sscanf(p, "%4d-%2d-%2d_%2d_%2d_%2d", &y, &m, &d, &H, &M, &S) == 6)
  {
    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_year = y - 1900;
    t.tm_mon = m - 1;
    t.tm_mday = d;
    t.tm_hour = H;
    t.tm_min = M;
    t.tm_sec = S;
    return mktime(&t);
  }
  return 0;
}

/**
 * Delete oldest console logs as long as logs take more than 10% of free SD space.
 */
void Board320_240::enforceSdLogSpaceLimit()
{
  if (!liveData->params.sdcardInit)
  {
    return;
  }

  if (!SD.exists("/logs"))
  {
    SD.mkdir("/logs");
    return;
  }

  uint16_t safetyCounter = 500;
  while (--safetyCounter > 0)
  {
    uint64_t totalBytes = SD.totalBytes();
    uint64_t usedBytes = SD.usedBytes();
    if (totalBytes == 0)
    {
      break;
    }

    uint64_t freeBytes = (totalBytes > usedBytes) ? (totalBytes - usedBytes) : 0;
    uint64_t maxLogsAllowedBytes = freeBytes / 10;

    File dir = SD.open("/logs");
    if (!dir || !dir.isDirectory())
    {
      if (dir)
      {
        dir.close();
      }
      break;
    }

    uint64_t totalLogsSize = 0;
    String oldestLogPath = "";
    time_t oldestLogTime = 0;
    bool foundAnyLog = false;

    while (true)
    {
      File entry = dir.openNextFile(FILE_READ);
      if (!entry)
      {
        break;
      }

      if (!entry.isDirectory())
      {
        String name = String(entry.name());
        if (name.endsWith(".log"))
        {
          String path = name;
          if (!path.startsWith("/logs/"))
          {
            if (path.startsWith("/"))
              path = "/logs" + path;
            else
              path = "/logs/" + path;
          }

          const char *activeLog = syslog->getSdLogPath();
          if (activeLog == nullptr || path != activeLog)
          {
            size_t sz = entry.size();
            totalLogsSize += sz;
            time_t fileTime = parseSdLogFileTime(path.c_str());
            if (fileTime == 0)
            {
              fileTime = entry.getLastWrite();
            }

            if (!foundAnyLog || fileTime < oldestLogTime)
            {
              foundAnyLog = true;
              oldestLogTime = fileTime;
              oldestLogPath = path;
            }
          }
        }
      }
      entry.close();
    }
    dir.close();

    if (!foundAnyLog || totalLogsSize <= maxLogsAllowedBytes)
    {
      break;
    }

    syslog->printf("SD logs total (%llu bytes) exceeds 10%% free SD space (%llu bytes). Deleting oldest: %s\n",
                   totalLogsSize, maxLogsAllowedBytes, oldestLogPath.c_str());
    if (!SD.remove(oldestLogPath.c_str()))
    {
      syslog->printf("Failed to remove oldest log: %s\n", oldestLogPath.c_str());
      break;
    }
  }
}

/**
 * Start console logging to SD card: /logs/YYYY-MM-DD_HH_mm_ss.log
 */
bool Board320_240::startSdcardConsoleLog()
{
  if (!liveData->params.sdcardInit && !sdcardMount())
  {
    syslog->println("SD card not mounted, cannot start console log");
    return false;
  }

  if (syslog->isSdLogging())
  {
    return true;
  }

  if (!SD.exists("/logs"))
  {
    SD.mkdir("/logs");
  }

  enforceSdLogSpaceLimit();

  char logPath[64];
  struct tm nowTm;
  if (!getLocalTime(&nowTm, 0))
  {
    memset(&nowTm, 0, sizeof(nowTm));
    nowTm.tm_year = 70; // 1970
    nowTm.tm_mday = 1;
    nowTm.tm_mon = 0;
  }

  strftime(logPath, sizeof(logPath), "/logs/%Y-%m-%d_%H_%M_%S.log", &nowTm);
  if (SD.exists(logPath))
  {
    for (int i = 1; i < 100; i++)
    {
      char candidate[64];
      snprintf(candidate, sizeof(candidate), "/logs/%04d-%02d-%02d_%02d_%02d_%02d_%d.log",
               nowTm.tm_year + 1900, nowTm.tm_mon + 1, nowTm.tm_mday,
               nowTm.tm_hour, nowTm.tm_min, nowTm.tm_sec, i);
      if (!SD.exists(candidate))
      {
        strncpy(logPath, candidate, sizeof(logPath));
        break;
      }
    }
  }

  if (syslog->startSdLogging(logPath))
  {
    syslog->printf("Started SD console logging to %s\n", logPath);
    return true;
  }
  else
  {
    syslog->printf("Failed to start SD console logging to %s\n", logPath);
    return false;
  }
}

/**
 * Stop console logging to SD card
 */
void Board320_240::stopSdcardConsoleLog()
{
  if (syslog->isSdLogging())
  {
    syslog->println("Stopping SD console logging");
    syslog->stopSdLogging();
  }
}

/**
 * Synchronizes GPS data with the liveData structure.
 *
 * This function updates GPS-related parameters such as latitude, longitude,
 * altitude, satellite count, speed, and time synchronization if valid data is received.
 */
void Board320_240::syncGPS()
{
  if (gps.satellites.isValid())
  {
    liveData->params.gpsSat = gps.satellites.value();
  }

  const float prevLat = liveData->params.gpsLat;
  const float prevLon = liveData->params.gpsLon;
  const bool hasPrevFix = isGpsFixUsable(liveData);

  bool accepted = false;
  float newLat = 0.0f;
  float newLon = 0.0f;

  if (gps.location.isValid())
  {
    newLat = gps.location.lat();
    newLon = gps.location.lng();

    if (isGpsCoordSane(newLat, newLon))
    {
      bool hasLastFix = (liveData->params.gpsLat != -1.0f && liveData->params.gpsLon != -1.0f &&
                         (liveData->params.gpsLastFixTime != 0 || liveData->params.gpsLastFixMs != 0));

      if (!hasLastFix)
      {
        accepted = true;
      }
      else
      {
        uint32_t dtSec = 0;
        if (liveData->params.currentTime > 0 && liveData->params.gpsLastFixTime > 0)
        {
          dtSec = (liveData->params.currentTime >= liveData->params.gpsLastFixTime)
                      ? static_cast<uint32_t>(liveData->params.currentTime - liveData->params.gpsLastFixTime)
                      : 0;
        }
        else if (liveData->params.gpsLastFixMs != 0)
        {
          uint32_t nowMs = millis();
          dtSec = (nowMs - liveData->params.gpsLastFixMs) / 1000;
        }

        if (dtSec == 0)
        {
          dtSec = 1;
        }

        float maxSpeedKmh = 130.0f;
        if (liveData->params.speedKmh > 0)
        {
          float candidate = liveData->params.speedKmh + 30.0f;
          if (candidate > maxSpeedKmh)
          {
            maxSpeedKmh = candidate;
          }
        }
        if (gps.speed.isValid())
        {
          float candidate = gps.speed.kmph() + 30.0f;
          if (candidate > maxSpeedKmh)
          {
            maxSpeedKmh = candidate;
          }
        }
        if (maxSpeedKmh > kGpsMaxSpeedKmh)
        {
          maxSpeedKmh = kGpsMaxSpeedKmh;
        }

        float distanceMeters = gpsDistanceMeters(liveData->params.gpsLat, liveData->params.gpsLon, newLat, newLon);
        float allowedMeters = (maxSpeedKmh * dtSec / 3.6f) + kGpsJitterMeters;
        bool shortJump = (dtSec <= kGpsShortJumpWindowSec && distanceMeters > kGpsMaxJumpMetersShort);
        bool speedJump = distanceMeters > allowedMeters;
        const uint32_t reacquireAfterSec =
            (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
                ? kGpsReacquireAfterSecV21
                : kGpsReacquireAfterSecDefault;

        if (!shortJump && !speedJump)
        {
          accepted = true;
        }
        else if (dtSec >= reacquireAfterSec)
        {
          accepted = true;
        }
      }
    }
  }

  if (accepted)
  {
    liveData->params.gpsValid = true;
    liveData->params.gpsLat = newLat;
    liveData->params.gpsLon = newLon;
    liveData->params.gpsAlt = gps.altitude.meters();
    liveData->params.gpsLastFixTime = liveData->params.currentTime;
    liveData->params.gpsLastFixMs = millis();
    calcAutomaticBrightnessLatLon(); // Adjust screen brightness based on location
  }
  else
  {
    liveData->params.gpsValid = false;
  }

  // Update GPS speed if valid and enough satellites are available
  if (gps.speed.isValid() && liveData->params.gpsSat >= 4)
  {
    liveData->params.speedKmhGPS = gps.speed.kmph();
  }
  else
  {
    liveData->params.speedKmhGPS = -1; // Invalid GPS speed
  }
  if (liveData->params.speedKmh == 0)
  {
    liveData->params.speedKmhGPS = 0;
  }
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
  {
    float headingFromMovement = -1.0f;
    if (accepted && hasPrevFix && liveData->params.gpsSat >= 4)
    {
      float movementMeters = gpsDistanceMeters(prevLat, prevLon, newLat, newLon);
      if (movementMeters >= kGpsHeadingMinDistanceMeters)
      {
        headingFromMovement = gpsHeadingFromCoords(prevLat, prevLon, newLat, newLon);
      }
    }

    // Some V2.1 modules provide an onboard course value even when movement between
    // two accepted fixes is too small for stable coordinate-based heading.
    if (headingFromMovement < 0.0f && gps.course.isValid() && liveData->params.gpsSat >= 4)
    {
      headingFromMovement = normalizeHeadingDeg(gps.course.deg());
    }

    if (headingFromMovement >= 0.0f)
    {
      liveData->params.gpsHeadingDeg = headingFromMovement;
    }
    else if (!isGpsFixUsable(liveData) || liveData->params.gpsSat < 4)
    {
      // Keep last known heading during brief low-movement periods; invalidate when GPS fix is stale.
      liveData->params.gpsHeadingDeg = -1;
    }
  }
  else if (gps.course.isValid() && liveData->params.gpsSat >= 4)
  {
    liveData->params.gpsHeadingDeg = normalizeHeadingDeg(gps.course.deg());
  }
  else
  {
    liveData->params.gpsHeadingDeg = -1;
  }

  // Synchronize time with GPS if it has not been synchronized yet.
  // When NTP is enabled, wait for the NTP priority window to expire before falling back to GPS time.
  if (!liveData->params.currTimeSyncWithGps && gps.date.isValid() && gps.time.isValid())
  {
    if (liveData->settings.ntpEnabled == 0 || liveData->params.ntpTimeSet || gpsTimeFallbackAllowed)
    {
      setGpsTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
    }
  }
}

/**
 * Initializes and connects to WiFi using the stored SSID and password.
 *
 * Enables STA mode, starts the connection, and updates the last connected time.
 *
 * @return True if WiFi initialization and connection succeeded, false otherwise.
 */
static bool isWifiSsidConfigured(const char *ssid)
{
  return ssid != nullptr && ssid[0] != '\0' && strcmp(ssid, "empty") != 0 && strcmp(ssid, "not_set") != 0;
}

/**
 * Initializes and connects to WiFi using the stored SSID and password.
 *
 * Enables STA mode, starts the connection, and updates the last connected time.
 *
 * @return True if WiFi initialization and connection succeeded, false otherwise.
 */
bool Board320_240::wifiSetup()
{
  liveData->params.wifiActiveIndex = 0;
  liveData->params.isWifiBackupLive = false;

  syslog->print("Initializing WiFi with SSID: ");
  syslog->println(liveData->settings.wifiSsid);

  // Enable Station mode and start connection
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(liveData->settings.wifiSsid, liveData->settings.wifiPassword);

  // Update the last connected time
  liveData->params.wifiLastConnectedTime = liveData->params.currentTime;

  return true;
}

/**
 * Handles switching between main and backup WiFi networks (primary, ssid2, ssid3, ssid4).
 *
 * Disconnects from the current WiFi network and attempts connection to the next configured AP.
 */
void Board320_240::wifiFallback()
{
  disconnectMqtt(false);
  WiFi.disconnect(false, false);

  uint8_t currentIndex = liveData->params.wifiActiveIndex;
  uint8_t targetIndex = 0;
  bool found = false;

  for (uint8_t i = 1; i <= 3; i++)
  {
    uint8_t candidate = (currentIndex + i) % 4;
    if (candidate == 0)
    {
      if (isWifiSsidConfigured(liveData->settings.wifiSsid))
      {
        targetIndex = 0;
        found = true;
        break;
      }
    }
    else if (candidate == 1)
    {
      if (liveData->settings.backupWifiEnabled == 1 && isWifiSsidConfigured(liveData->settings.wifiSsid2))
      {
        targetIndex = 1;
        found = true;
        break;
      }
    }
    else if (candidate == 2)
    {
      if (isWifiSsidConfigured(liveData->settings.wifiSsid3))
      {
        targetIndex = 2;
        found = true;
        break;
      }
    }
    else if (candidate == 3)
    {
      if (isWifiSsidConfigured(liveData->settings.wifiSsid4))
      {
        targetIndex = 3;
        found = true;
        break;
      }
    }
  }

  if (found)
  {
    wifiSwitchToIndex(targetIndex);
  }
  else
  {
    wifiSwitchToMain();
  }
}

/**
 * Switches to a specific WiFi network index (0=main, 1=ssid2, 2=ssid3, 3=ssid4).
 */
void Board320_240::wifiSwitchToIndex(uint8_t index)
{
  const char *ssid = "";
  const char *password = "";
  switch (index)
  {
  case 0:
    ssid = liveData->settings.wifiSsid;
    password = liveData->settings.wifiPassword;
    break;
  case 1:
    ssid = liveData->settings.wifiSsid2;
    password = liveData->settings.wifiPassword2;
    break;
  case 2:
    ssid = liveData->settings.wifiSsid3;
    password = liveData->settings.wifiPassword3;
    break;
  case 3:
    ssid = liveData->settings.wifiSsid4;
    password = liveData->settings.wifiPassword4;
    break;
  default:
    index = 0;
    ssid = liveData->settings.wifiSsid;
    password = liveData->settings.wifiPassword;
    break;
  }

  liveData->params.wifiActiveIndex = index;
  liveData->params.isWifiBackupLive = (index > 0);
  if (index == 1)
  {
    liveData->params.wifiBackupUptime = liveData->params.currentTime;
    syslog->print("Switching to 2nd AP: ");
    syslog->println(ssid);
  }
  else if (index == 2)
  {
    liveData->params.wifiBackupUptime = liveData->params.currentTime;
    syslog->print("Switching to 3rd AP: ");
    syslog->println(ssid);
  }
  else if (index == 3)
  {
    liveData->params.wifiBackupUptime = liveData->params.currentTime;
    syslog->print("Switching to 4th AP: ");
    syslog->println(ssid);
  }
  else
  {
    syslog->print("Switching to main WiFi: ");
    syslog->println(ssid);
  }

  WiFi.begin(ssid, password);
  liveData->params.wifiLastConnectedTime = liveData->params.currentTime;
}

/**
 * Switches to the backup WiFi network (index 1).
 */
void Board320_240::wifiSwitchToBackup()
{
  wifiSwitchToIndex(1);
}

/**
 * Restores the main WiFi connection (index 0).
 */
void Board320_240::wifiSwitchToMain()
{
  wifiSwitchToIndex(0);
}

bool Board320_240::wifiScanToMenu()
{
  displayMessage("Scanning WiFi...", "");
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);

  int found = WiFi.scanNetworks();
  liveData->wifiScanCount = 0;

  for (uint16_t i = 0; i < liveData->menuItemsCount; ++i)
  {
    if (liveData->menuItems[i].id >= LIST_OF_WIFI_1 && liveData->menuItems[i].id <= LIST_OF_WIFI_10)
    {
      strlcpy(liveData->menuItems[i].title, "-", sizeof(liveData->menuItems[i].title));
    }
  }

  if (found <= 0)
  {
    displayMessage("WiFi scan", "No networks found");
    delay(1500);
    showMenu();
    return false;
  }

  for (int i = 0; i < found && liveData->wifiScanCount < 10; i++)
  {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0)
      continue;

    int8_t rssi = WiFi.RSSI(i);
    uint8_t enc = WiFi.encryptionType(i);
    uint8_t slot = liveData->wifiScanCount;

    ssid.toCharArray(liveData->wifiScanSsid[slot], sizeof(liveData->wifiScanSsid[slot]));
    liveData->wifiScanRssi[slot] = rssi;
    liveData->wifiScanEnc[slot] = enc;

    String label = ssid;
    label += " ";
    label += String(rssi);
    label += "dBm";
    if (enc != WIFI_AUTH_OPEN)
      label += " *";

    for (uint16_t m = 0; m < liveData->menuItemsCount; ++m)
    {
      if (liveData->menuItems[m].id == (LIST_OF_WIFI_1 + slot))
      {
        label.toCharArray(liveData->menuItems[m].title, sizeof(liveData->menuItems[m].title));
        break;
      }
    }

    liveData->wifiScanCount++;
  }

  liveData->menuVisible = true;
  liveData->menuCurrent = LIST_OF_WIFI_DEV;
  liveData->menuItemSelected = 0;
  liveData->menuItemOffset = 0;
  showMenu();
  return true;
}

bool Board320_240::promptKeyboard(const char *title, String &value, bool mask, uint8_t maxLen)
{
  keyboardInputActive = true;
  const int16_t screenW = tft.width();
  const int16_t screenH = tft.height();
  const int16_t topBtnY = 4;
  const int16_t topBtnH = 24;
  const int16_t topExitW = 50;
  const int16_t topExitX = 6;
  const int16_t titleY = 6;
  const int16_t titleX = topExitX + topExitW + 8;
  const int16_t inputY = 32;
  const int16_t inputH = 34;
  const int16_t keyW = 30;
  const int16_t gap = 2;
  const int16_t rowsY = inputY + inputH + 8;
  const int16_t keyboardRows = 5;
  const int16_t modeW = 48;
  const int16_t shiftW = 56;
  const int16_t spaceW = 98;
  const int16_t bkspW = 52;
  const int16_t okW = 58;
  int16_t keyH = (screenH - 6 - rowsY - (keyboardRows - 1) * gap) / keyboardRows;
  if (keyH < 26)
    keyH = 26;

  enum KeyAction : uint8_t
  {
    KEY_NONE = 0,
    KEY_CHAR,
    KEY_MODE,
    KEY_SHIFT,
    KEY_SPACE,
    KEY_DEL,
    KEY_OK,
    KEY_EXIT
  };
  struct KeyHit
  {
    KeyAction action;
    char ch;
  };

  struct KeyRows
  {
    const char *row[4];
    uint8_t len[4];
  };

  bool shift = false;
  bool numericMode = false;
  bool touchActive = false;
  KeyAction activeAction = KEY_NONE;
  char activeChar = 0;
  char previewChar = 0;
  uint32_t previewUntil = 0;
  bool needsRedraw = true;

  auto loadRows = [&](KeyRows &rows)
  {
    if (!numericMode)
    {
      rows.row[0] = shift ? "QWERTYUIOP" : "qwertyuiop";
      rows.len[0] = 10;
      rows.row[1] = shift ? "ASDFGHJKL" : "asdfghjkl";
      rows.len[1] = 9;
      rows.row[2] = shift ? "ZXCVBNM" : "zxcvbnm";
      rows.len[2] = 7;
      rows.row[3] = "@._-/:;!?+";
      rows.len[3] = 10;
      return;
    }

    rows.row[0] = "1234567890";
    rows.len[0] = 10;
    if (!shift)
    {
      rows.row[1] = "!@#$%^&*()";
      rows.len[1] = 10;
      rows.row[2] = "-_=+[]{}";
      rows.len[2] = 8;
      rows.row[3] = ".,:;/?\\|";
      rows.len[3] = 8;
    }
    else
    {
      rows.row[1] = "~`<>\"'()";
      rows.len[1] = 8;
      rows.row[2] = "{}[]+=-_";
      rows.len[2] = 8;
      rows.row[3] = "#$%&*@!?";
      rows.len[3] = 8;
    }
  };

  auto hitTest = [&](int16_t tx, int16_t ty) -> KeyHit
  {
    if (ty >= topBtnY && ty <= topBtnY + topBtnH)
    {
      if (tx >= topExitX && tx <= topExitX + topExitW)
      {
        KeyHit hit = {KEY_EXIT, 0};
        return hit;
      }
    }

    KeyRows rows = {};
    loadRows(rows);

    for (int r = 0; r < 4; r++)
    {
      int len = rows.len[r];
      int rowWidth = len * keyW + (len - 1) * gap;
      int x0 = (screenW - rowWidth) / 2;
      int y0 = rowsY + r * (keyH + gap);
      if (ty < y0 || ty > y0 + keyH || tx < x0 || tx > x0 + rowWidth)
        continue;
      int col = (tx - x0) / (keyW + gap);
      if (col < 0 || col >= len)
        continue;
      KeyHit hit = {KEY_CHAR, rows.row[r][col]};
      return hit;
    }

    int ctrlY = rowsY + 4 * (keyH + gap);
    int totalW = modeW + shiftW + spaceW + bkspW + okW + gap * 4;
    int x = (screenW - totalW) / 2;
    int modeX = x;
    int shiftX = modeX + modeW + gap;
    int spaceX = shiftX + shiftW + gap;
    int bkspX = spaceX + spaceW + gap;
    int okX = bkspX + bkspW + gap;

    if (ty >= ctrlY && ty <= ctrlY + keyH)
    {
      if (tx >= modeX && tx <= modeX + modeW)
      {
        KeyHit hit = {KEY_MODE, 0};
        return hit;
      }
      if (tx >= shiftX && tx <= shiftX + shiftW)
      {
        KeyHit hit = {KEY_SHIFT, 0};
        return hit;
      }
      if (tx >= spaceX && tx <= spaceX + spaceW)
      {
        KeyHit hit = {KEY_SPACE, 0};
        return hit;
      }
      if (tx >= bkspX && tx <= bkspX + bkspW)
      {
        KeyHit hit = {KEY_DEL, 0};
        return hit;
      }
      if (tx >= okX && tx <= okX + okW)
      {
        KeyHit hit = {KEY_OK, 0};
        return hit;
      }
    }

    KeyHit hit = {KEY_NONE, 0};
    return hit;
  };

  auto drawKeyboard = [&](bool showPreview)
  {
    if (liveData->params.spriteInit)
    {
      spr.fillSprite(TFT_BLACK);
    }
    else
    {
      tft.fillScreen(TFT_BLACK);
    }

    auto drawRect = [&](int16_t x, int16_t y, int16_t w, int16_t h, uint16_t bg, uint16_t fg)
    {
      if (liveData->params.spriteInit)
      {
        spr.fillRoundRect(x, y, w, h, 4, bg);
        spr.drawRoundRect(x, y, w, h, 4, fg);
      }
      else
      {
        tft.fillRoundRect(x, y, w, h, 4, bg);
        tft.drawRoundRect(x, y, w, h, 4, fg);
      }
    };

    auto drawText = [&](const char *text, int16_t x, int16_t y, bool bigFont)
    {
      if (liveData->params.spriteInit)
      {
        sprSetFont(bigFont ? fontRobotoThin24 : fontFont2);
        spr.setTextColor(TFT_WHITE);
        spr.setTextDatum(MC_DATUM);
        sprDrawString(text, x, y);
      }
      else
      {
        tft.setFont(bigFont ? fontRobotoThin24 : fontFont2);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(text, x, y);
      }
    };

    auto drawLinePrim = [&](int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t col)
    {
      if (liveData->params.spriteInit)
      {
        spr.drawLine(x0, y0, x1, y1, col);
      }
      else
      {
        tft.drawLine(x0, y0, x1, y1, col);
      }
    };

    auto drawEnterIcon = [&](int16_t x, int16_t y, int16_t w, int16_t h, uint16_t col)
    {
      // Simple bent "enter" arrow.
      int16_t cx = x + w / 2 + 7;
      int16_t top = y + 6;
      int16_t mid = y + h / 2 + 1;
      drawLinePrim(cx, top, cx, mid, col);
      drawLinePrim(cx, mid, x + 10, mid, col);
      drawLinePrim(x + 10, mid, x + 14, mid - 4, col);
      drawLinePrim(x + 10, mid, x + 14, mid + 4, col);
    };

    auto isActive = [&](KeyAction action, char ch) -> bool
    {
      return touchActive && activeAction == action && activeChar == ch;
    };

    drawRect(topExitX, topBtnY, topExitW, topBtnH, isActive(KEY_EXIT, 0) ? TFT_RED : 0x1082, isActive(KEY_EXIT, 0) ? TFT_CYAN : 0x39E7);
    drawText("EXIT", topExitX + topExitW / 2, topBtnY + topBtnH / 2 - 1, false);

    if (liveData->params.spriteInit)
    {
      // Keep title style consistent with keyboard text style.
      sprSetFont(fontFont2);
      spr.setTextColor(TFT_WHITE);
      spr.setTextDatum(TL_DATUM);
      sprDrawString(title, titleX, titleY);
    }
    else
    {
      tft.setFont(fontFont2);
      tft.setTextColor(TFT_WHITE);
      tft.setTextDatum(TL_DATUM);
      tft.drawString(title, titleX, titleY);
    }

    String display = value;
    if (mask)
    {
      display = "";
      for (size_t i = 0; i < value.length(); i++)
        display += "*";
    }

    drawRect(6, inputY, screenW - 12, inputH, 0x0841, TFT_CYAN);
    if (liveData->params.spriteInit)
    {
      sprSetFont(fontRobotoThin24);
      spr.setTextColor(TFT_WHITE);
      spr.setTextDatum(TL_DATUM);
      sprDrawString(display.c_str(), 10, inputY + 7);
    }
    else
    {
      tft.setFont(fontRobotoThin24);
      tft.setTextColor(TFT_WHITE);
      tft.setTextDatum(TL_DATUM);
      tft.drawString(display.c_str(), 10, inputY + 6);
    }

    KeyRows rows = {};
    loadRows(rows);

    for (int r = 0; r < 4; r++)
    {
      int len = rows.len[r];
      int rowWidth = len * keyW + (len - 1) * gap;
      int x0 = (screenW - rowWidth) / 2;
      int y0 = rowsY + r * (keyH + gap);
      for (int c = 0; c < len; c++)
      {
        char key[2] = {rows.row[r][c], 0};
        int x = x0 + c * (keyW + gap);
        bool active = isActive(KEY_CHAR, rows.row[r][c]);
        drawRect(x, y0, keyW, keyH, active ? 0x001B : 0x1082, active ? TFT_CYAN : 0x39E7);
        drawText(key, x + keyW / 2, y0 + keyH / 2 - 2, true);
      }
    }

    int ctrlY = rowsY + 4 * (keyH + gap);
    int totalW = modeW + shiftW + spaceW + bkspW + okW + gap * 4;
    int x = (screenW - totalW) / 2;
    drawRect(x, ctrlY, modeW, keyH, isActive(KEY_MODE, 0) ? 0x001B : 0x1082, isActive(KEY_MODE, 0) ? TFT_CYAN : 0x39E7);
    drawText(numericMode ? "abc" : "123", x + modeW / 2, ctrlY + keyH / 2 - 2, false);
    x += modeW + gap;
    drawRect(x, ctrlY, shiftW, keyH, isActive(KEY_SHIFT, 0) ? 0x001B : (shift ? TFT_BLUE : 0x1082),
             isActive(KEY_SHIFT, 0) ? TFT_CYAN : 0x39E7);
    drawText("SHIFT", x + shiftW / 2, ctrlY + keyH / 2 - 2, false);
    x += shiftW + gap;
    drawRect(x, ctrlY, spaceW, keyH, isActive(KEY_SPACE, 0) ? 0x001B : 0x1082, isActive(KEY_SPACE, 0) ? TFT_CYAN : 0x39E7);
    drawText("SPACE", x + spaceW / 2, ctrlY + keyH / 2 - 2, false);
    x += spaceW + gap;
    drawRect(x, ctrlY, bkspW, keyH, isActive(KEY_DEL, 0) ? 0x001B : 0x1082, isActive(KEY_DEL, 0) ? TFT_CYAN : 0x39E7);
    drawText("DEL", x + bkspW / 2, ctrlY + keyH / 2 - 2, false);
    x += bkspW + gap;
    drawRect(x, ctrlY, okW, keyH, isActive(KEY_OK, 0) ? TFT_GREEN : TFT_DARKGREEN2, isActive(KEY_OK, 0) ? TFT_CYAN : 0x39E7);
    drawEnterIcon(x, ctrlY, okW, keyH, TFT_WHITE);

    if (showPreview && previewChar != 0)
    {
      const int16_t bubbleW = 144;
      const int16_t bubbleH = 68;
      const int16_t bubbleX = (screenW - bubbleW) / 2;
      const int16_t bubbleY = 4;
      char previewText[2] = {previewChar, 0};
      drawRect(bubbleX, bubbleY, bubbleW, bubbleH, 0x52AA, 0x7BEF);
      if (liveData->params.spriteInit)
      {
        sprSetFont(fontOrbitronLight32);
        spr.setTextColor(TFT_WHITE);
        spr.setTextDatum(MC_DATUM);
        sprDrawString(previewText, bubbleX + bubbleW / 2, bubbleY + bubbleH / 2 + 1);
      }
      else
      {
        tft.setFont(fontOrbitronLight32);
        tft.setTextColor(TFT_WHITE);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(previewText, bubbleX + bubbleW / 2, bubbleY + bubbleH / 2 + 1);
      }
    }

    if (liveData->params.spriteInit)
      spr.pushSprite(0, 0);
  };

  drawKeyboard(false);
  bool previousPressed = false;

  auto readTouchRaw = [&](int16_t &x, int16_t &y) -> bool
  {
#ifdef BOARD_M5STACK_CORE2
    if (M5.Touch.ispressed() && M5.Touch.points > 0 && M5.Touch.point[0].valid())
    {
      x = M5.Touch.point[0].x;
      y = M5.Touch.point[0].y;
      if (x < 0 || y < 0 || x >= screenW || y >= screenH)
        return false;
      return true;
    }
#endif
#ifdef BOARD_M5STACK_CORES3
    auto t = CoreS3.Touch.getDetail();
    if (t.isPressed())
    {
      x = t.x;
      y = t.y;
      if (x < 0 || y < 0 || x >= screenW || y >= screenH)
        return false;
      return true;
    }
#endif
    return false;
  };

  while (true)
  {
    boardLoop();
    uint32_t nowMs = millis();
    int16_t tx = 0, ty = 0;
    bool touched = readTouchRaw(tx, ty);

    if (touched)
    {
      KeyHit hit = hitTest(tx, ty);
      if (!touchActive || hit.action != activeAction || hit.ch != activeChar)
      {
        activeAction = hit.action;
        activeChar = hit.ch;
        needsRedraw = true;
      }
      touchActive = true;
      if (hit.action == KEY_CHAR && hit.ch != 0)
      {
        previewChar = hit.ch;
        previewUntil = nowMs + 500;
        needsRedraw = true;
      }
      else if (previewChar != 0)
      {
        previewChar = 0;
        needsRedraw = true;
      }
    }
    else if (touchActive && previousPressed)
    {
      if (activeAction == KEY_CHAR && activeChar != 0)
      {
        if (value.length() < maxLen)
        {
          value += activeChar;
          if (shift && activeChar >= 'A' && activeChar <= 'Z')
            shift = false;
        }
        previewChar = activeChar;
        previewUntil = nowMs + 500;
      }
      else if (activeAction == KEY_MODE)
      {
        numericMode = !numericMode;
        shift = false;
      }
      else if (activeAction == KEY_SHIFT)
      {
        shift = !shift;
      }
      else if (activeAction == KEY_SPACE)
      {
        if (value.length() < maxLen)
          value += " ";
      }
      else if (activeAction == KEY_DEL)
      {
        if (value.length() > 0)
          value.remove(value.length() - 1);
      }
      else if (activeAction == KEY_OK)
      {
        keyboardInputActive = false;
        suppressTouchInputFor();
        return true;
      }
      else if (activeAction == KEY_EXIT)
      {
        keyboardInputActive = false;
        suppressTouchInputFor();
        return false;
      }

      touchActive = false;
      activeAction = KEY_NONE;
      activeChar = 0;
      needsRedraw = true;
    }
    else if (previewChar != 0 && nowMs >= previewUntil)
    {
      previewChar = 0;
      needsRedraw = true;
    }

    if (needsRedraw)
    {
      drawKeyboard(previewChar != 0);
      needsRedraw = false;
    }

    previousPressed = touched;
    if (!touched)
      delay(5);
  }
}

void Board320_240::suppressTouchInputFor(uint16_t durationMs)
{
  suppressTouchInputUntilMs = millis() + durationMs;
}

bool Board320_240::isTouchInputSuppressed() const
{
  if (suppressTouchInputUntilMs == 0)
    return false;
  return static_cast<int32_t>(suppressTouchInputUntilMs - millis()) > 0;
}

bool Board320_240::isKeyboardInputActive() const
{
  return keyboardInputActive;
}

bool Board320_240::isMessageDialogVisible() const
{
  return messageDialogVisible;
}

bool Board320_240::dismissMessageDialog()
{
  if (!messageDialogVisible)
    return false;

  messageDialogVisible = false;
  menuTouchHoverIndex = -1;
  suppressTouchInputFor();

  if (liveData->menuVisible)
  {
    menuDragScrollActive = true;
    showMenu();
    menuDragScrollActive = false;
  }
  else
  {
    redrawScreen();
  }

  return true;
}

bool Board320_240::promptWifiPassword(const char *ssid, String &outPassword, bool isOpenNetwork)
{
  if (isOpenNetwork)
  {
    outPassword = "";
    return true;
  }
  String value = outPassword;
  String title = String("Passwd for ") + ssid;
  bool ok = promptKeyboard(title.c_str(), value, false, sizeof(liveData->settings.wifiPassword) - 1);
  if (ok)
    outPassword = value;
  return ok;
}

bool Board320_240::canStatusMessageVisible()
{
  if (liveData->settings.commType != COMM_TYPE_CAN_COMMU)
    return false;

  String status = commInterface->getConnectStatus();
  if (status == "")
    return false;

  if (dismissedCanStatusText != "" && status == dismissedCanStatusText)
    return false;

  // New status text arrived, clear previous dismissal.
  if (dismissedCanStatusText != "" && status != dismissedCanStatusText)
    dismissedCanStatusText = "";

  return true;
}

bool Board320_240::canStatusMessageHitTest(int16_t x, int16_t y)
{
  if (!canStatusMessageVisible() && !netStatusMessageVisible())
    return false;
  return (x >= 0 && x < 320 && y >= 185 && y < 235);
}

void Board320_240::dismissCanStatusMessage()
{
  if (liveData->settings.commType == COMM_TYPE_CAN_COMMU)
  {
    String status = commInterface->getConnectStatus();
    if (status != "")
      dismissedCanStatusText = status;
  }

  if (netStatusMessageVisible())
  {
    dismissedNetFailureTime = liveData->params.netLastFailureTime;
  }
}

bool Board320_240::netStatusMessageVisible() const
{
  if (liveData->params.netAvailable)
    return false;
  if (liveData->params.netLastFailureTime == 0)
    return false;
  if (liveData->params.currentTime < liveData->params.netLastFailureTime)
    return false;
  if ((liveData->params.currentTime - liveData->params.netLastFailureTime) > kNetFailureStaleResetSec)
    return false;
  if (dismissedNetFailureTime != 0 && dismissedNetFailureTime == liveData->params.netLastFailureTime)
    return false;
  return true;
}

bool Board320_240::isContributeKeyValid(const char *key) const
{
  if (key == nullptr)
    return false;

  String normalized = String(key);
  normalized.trim();
  if (normalized.length() < 12)
    return false;
  if (normalized == "empty" || normalized == "not_set")
    return false;

  for (uint16_t i = 0; i < normalized.length(); i++)
  {
    const char ch = normalized.charAt(i);
    if (ch <= ' ' || ch > '~')
      return false;
  }

  return true;
}

String Board320_240::ensureContributeKey()
{
  String key = String(liveData->settings.contributeToken);
  key.trim();
  if (isContributeKeyValid(key.c_str()))
  {
    return key;
  }

  const uint64_t efuse = ESP.getEfuseMac();
  const uint32_t rnd = esp_random();
  const uint32_t nowMs = millis();
  char generated[sizeof(liveData->settings.contributeToken)] = {0};
  snprintf(generated, sizeof(generated), "k%08lX%08lX%08lX", (uint32_t)(efuse & 0xFFFFFFFFULL), (uint32_t)rnd, nowMs);
  key = String(generated);
  key.toCharArray(liveData->settings.contributeToken, sizeof(liveData->settings.contributeToken));
  saveSettings();
  syslog->print("Generated contribute key: ");
  syslog->println(key);
  return key;
}

String Board320_240::getHardwareDeviceId() const
{
  const uint64_t efuse = (ESP.getEfuseMac() & 0xFFFFFFFFFFFFULL);
  const uint32_t uuidPart1 = static_cast<uint32_t>((efuse >> 16) & 0xFFFFFFFFULL);
  const uint16_t uuidPart2 = static_cast<uint16_t>(efuse & 0xFFFFU);
  const uint16_t uuidPart3 = static_cast<uint16_t>(((efuse >> 32) & 0x0FFFU) | 0x4000U); // UUID version 4 layout
  const uint16_t uuidPart4 = static_cast<uint16_t>(((efuse >> 20) & 0x3FFFU) | 0x8000U); // UUID variant 1 layout
#ifdef BOARD_M5STACK_CORES3
  const uint8_t boardTag = 0x03U;
#else
  const uint8_t boardTag = 0x02U;
#endif
  const uint64_t uuidPart5 = ((efuse ^ (static_cast<uint64_t>(boardTag) << 40)) & 0xFFFFFFFFFFFFULL);
  char deviceId[40] = {0};
  snprintf(deviceId, sizeof(deviceId), "%08lX-%04X-%04X-%04X-%012llX",
           uuidPart1, uuidPart2, uuidPart3, uuidPart4, uuidPart5);
  return String(deviceId);
}

String getTraccarDeviceIdFromEfuse()
{
  char deviceId[9] = {0};
  uint64_t seed = ESP.getEfuseMac() >> 8;
  for (int i = 0; i < 8; i++, seed >>= 5)
  {
    byte x = (byte)seed & 0x1f;
    if (x >= 10)
    {
      x = x - 10 + 'A';
      switch (x)
      {
      case 'B':
        x = 'W';
        break;
      case 'D':
        x = 'X';
        break;
      case 'I':
        x = 'Y';
        break;
      case 'O':
        x = 'Z';
        break;
      }
    }
    else
    {
      x += '0';
    }
    deviceId[i] = x;
  }
  deviceId[8] = 0;
  return String(deviceId);
}

String Board320_240::getPairDeviceId() const
{
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());
  if (hardwareDeviceId.length() == 0)
  {
    return "";
  }
  return hardwareDeviceId;
}

bool Board320_240::requestPairingStart(String &outCode, uint32_t &outExpiresInSec)
{
  outCode = "";
  outExpiresInSec = 0;

  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }

  const String pairDeviceId = getPairDeviceId();
  if (pairDeviceId.length() == 0)
  {
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(kPairHttpTimeoutMs);

  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(kPairHttpTimeoutMs);
  http.setTimeout(kPairHttpTimeoutMs);

  String url = String(PAIR_START_URL);
  url += (url.indexOf('?') == -1) ? "?" : "&";
  url += "id=" + pairDeviceId;

  if (!http.begin(client, url))
  {
    syslog->println("Pair start: begin failed");
    return false;
  }

  http.addHeader("User-Agent", String("evDash/") + String(APP_VERSION));
  const int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK)
  {
    syslog->print("Pair start HTTP code: ");
    syslog->println(httpCode);
    http.end();
    return false;
  }

  const String response = http.getString();
  http.end();

  StaticJsonDocument<512> jsonDoc;
  const DeserializationError jsonErr = deserializeJson(jsonDoc, response);
  if (jsonErr)
  {
    syslog->print("Pair start JSON parse error: ");
    syslog->println(jsonErr.c_str());
    return false;
  }

  const char *statusRaw = jsonDoc["status"];
  if (statusRaw == nullptr || strcmp(statusRaw, "ok") != 0)
  {
    syslog->println("Pair start: status not ok");
    return false;
  }

  const char *pairCodeRaw = jsonDoc["pairCode"];
  if (pairCodeRaw == nullptr)
  {
    syslog->println("Pair start: missing pairCode");
    return false;
  }

  String code = String(pairCodeRaw);
  code.trim();
  if (code.length() != 6)
  {
    syslog->println("Pair start: invalid pairCode");
    return false;
  }
  for (uint8_t i = 0; i < 6; i++)
  {
    const char ch = code.charAt(i);
    if (ch < '0' || ch > '9')
    {
      syslog->println("Pair start: non-numeric pairCode");
      return false;
    }
  }

  uint32_t expiresInSec = jsonDoc["expiresInSec"] | 0;
  if (expiresInSec < 30)
  {
    expiresInSec = 300;
  }

  outCode = code;
  outExpiresInSec = expiresInSec;
  return true;
}

uint8_t Board320_240::requestPairingStatus(const String &pairCode, String &outCarName)
{
  outCarName = "";

  if (WiFi.status() != WL_CONNECTED)
  {
    return 255;
  }
  if (pairCode.length() != 6)
  {
    return 255;
  }

  const String pairDeviceId = getPairDeviceId();
  if (pairDeviceId.length() == 0)
  {
    return 255;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(kPairHttpTimeoutMs);

  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(kPairHttpTimeoutMs);
  http.setTimeout(kPairHttpTimeoutMs);

  String url = String(PAIR_STATUS_URL);
  url += (url.indexOf('?') == -1) ? "?" : "&";
  url += "id=" + pairDeviceId;
  url += "&code=" + pairCode;

  if (!http.begin(client, url))
  {
    syslog->println("Pair status: begin failed");
    return 255;
  }

  http.addHeader("User-Agent", String("evDash/") + String(APP_VERSION));
  const int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK)
  {
    syslog->print("Pair status HTTP code: ");
    syslog->println(httpCode);
    http.end();
    return 255;
  }

  const String response = http.getString();
  http.end();

  StaticJsonDocument<512> jsonDoc;
  const DeserializationError jsonErr = deserializeJson(jsonDoc, response);
  if (jsonErr)
  {
    syslog->print("Pair status JSON parse error: ");
    syslog->println(jsonErr.c_str());
    return 255;
  }

  const char *statusRaw = jsonDoc["status"];
  if (statusRaw == nullptr || strcmp(statusRaw, "ok") != 0)
  {
    return 255;
  }

  const char *pairStatusRaw = jsonDoc["pairStatus"];
  if (pairStatusRaw == nullptr)
  {
    return 255;
  }

  String pairStatus = String(pairStatusRaw);
  pairStatus.toLowerCase();
  if (pairStatus == "pending")
  {
    return 1;
  }
  if (pairStatus == "paired")
  {
    const char *carNameRaw = jsonDoc["carName"];
    if (carNameRaw != nullptr)
    {
      outCarName = String(carNameRaw);
      outCarName.trim();
    }
    return 2;
  }
  if (pairStatus == "expired")
  {
    return 3;
  }
  if (pairStatus == "none")
  {
    return 0;
  }

  return 255;
}

void Board320_240::startEvdashPairing()
{
  if (WiFi.status() != WL_CONNECTED || liveData->settings.wifiEnabled != 1)
  {
    displayMessage("Pair with evdash.eu", "WiFi not connected");
    return;
  }

  String code;
  uint32_t expiresInSec = 0;
  if (!requestPairingStart(code, expiresInSec))
  {
    displayMessage("Pairing failed", "Try again");
    return;
  }

  strncpy(pairPendingCode, code.c_str(), sizeof(pairPendingCode) - 1);
  pairPendingCode[sizeof(pairPendingCode) - 1] = '\0';

  time_t nowTs = liveData->params.currentTime;
  if (nowTs <= 0)
  {
    nowTs = static_cast<time_t>(millis() / 1000U);
  }
  if (expiresInSec < 30)
  {
    expiresInSec = 300;
  }
  pairPendingExpiresAt = nowTs + static_cast<time_t>(expiresInSec);
  pairLastStatusPollMs = 0;
  pairLastKnownState = 1;

  String row3 = String("Pin/Code ") + code;
  displayMessage("Open evdash.eu", "Settings / cars -> pair", row3.c_str());
}

void Board320_240::pollEvdashPairingStatus()
{
  if (pairPendingCode[0] == '\0')
  {
    return;
  }

  time_t nowTs = liveData->params.currentTime;
  if (nowTs <= 0)
  {
    nowTs = static_cast<time_t>(millis() / 1000U);
  }

  if (pairPendingExpiresAt > 0 && nowTs > pairPendingExpiresAt)
  {
    pairPendingCode[0] = '\0';
    pairPendingExpiresAt = 0;
    if (pairLastKnownState != 3)
    {
      displayMessage("Pair code expired", "Generate new code");
    }
    pairLastKnownState = 3;
    return;
  }

  if (WiFi.status() != WL_CONNECTED || liveData->settings.wifiEnabled != 1)
  {
    return;
  }

  const uint32_t nowMs = millis();
  if (pairLastStatusPollMs != 0 &&
      static_cast<uint32_t>(nowMs - pairLastStatusPollMs) < kPairStatusPollIntervalMs)
  {
    return;
  }
  pairLastStatusPollMs = nowMs;

  String carName;
  const uint8_t state = requestPairingStatus(String(pairPendingCode), carName);
  if (state == 255)
  {
    return;
  }

  if (state == 2)
  {
    pairPendingCode[0] = '\0';
    pairPendingExpiresAt = 0;
    pairLastKnownState = 2;
    if (carName.length() == 0)
    {
      carName = "Device linked";
    }
    displayMessage("Paired with evdash.eu", carName.c_str());
    return;
  }

  if (state == 3)
  {
    pairPendingCode[0] = '\0';
    pairPendingExpiresAt = 0;
    if (pairLastKnownState != 3)
    {
      displayMessage("Pair code expired", "Generate new code");
    }
    pairLastKnownState = 3;
    return;
  }

  pairLastKnownState = state;
}

int Board320_240::compareVersionTags(const String &left, const String &right) const
{
  const auto parse = [](const String &input, int out[4]) -> bool
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      out[i] = 0;
    }

    String normalized = input;
    normalized.trim();
    if (normalized.length() == 0)
    {
      return false;
    }
    if (normalized.charAt(0) == 'v' || normalized.charAt(0) == 'V')
    {
      normalized.remove(0, 1);
    }
    if (normalized.length() == 0)
    {
      return false;
    }

    uint8_t partIndex = 0;
    int value = -1;
    for (uint16_t i = 0; i < normalized.length(); i++)
    {
      const char ch = normalized.charAt(i);
      if (ch >= '0' && ch <= '9')
      {
        if (value < 0)
        {
          value = 0;
        }
        value = (value * 10) + (ch - '0');
      }
      else if (ch == '.')
      {
        if (value < 0 || partIndex >= 4)
        {
          return false;
        }
        out[partIndex++] = value;
        value = -1;
        if (partIndex == 4)
        {
          return true;
        }
      }
      else
      {
        break;
      }
    }

    if (value >= 0 && partIndex < 4)
    {
      out[partIndex++] = value;
    }

    return partIndex >= 3;
  };

  int leftParts[4] = {0, 0, 0, 0};
  int rightParts[4] = {0, 0, 0, 0};
  if (!parse(left, leftParts) || !parse(right, rightParts))
  {
    return 0;
  }

  for (uint8_t i = 0; i < 4; i++)
  {
    if (leftParts[i] < rightParts[i])
    {
      return -1;
    }
    if (leftParts[i] > rightParts[i])
    {
      return 1;
    }
  }

  return 0;
}

void Board320_240::checkFirmwareVersionOnServer()
{
  const uint32_t nowMs = millis();
  if (lastFirmwareVersionCheckMs != 0 &&
      static_cast<uint32_t>(nowMs - lastFirmwareVersionCheckMs) < kFirmwareVersionCheckCooldownMs)
  {
    return;
  }
  lastFirmwareVersionCheckMs = nowMs;

  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(kFirmwareVersionHttpTimeoutMs);

  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(kFirmwareVersionHttpTimeoutMs);
  http.setTimeout(kFirmwareVersionHttpTimeoutMs);

  String firmwareCheckUrl = String(FW_VERSION_CHECK_URL);
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());
  String firmwareCheckId = "";
  if (hardwareDeviceId.length() > 0)
  {
    firmwareCheckId = hardwareDeviceId;
  }

  String appVersionForApi = String(APP_VERSION);
  appVersionForApi.trim();
  if (appVersionForApi.startsWith("v") || appVersionForApi.startsWith("V"))
  {
    appVersionForApi.remove(0, 1);
  }

  if (firmwareCheckId.length() > 0)
  {
    firmwareCheckUrl += (firmwareCheckUrl.indexOf('?') == -1) ? "?" : "&";
    firmwareCheckUrl += "id=" + firmwareCheckId;
  }
  if (appVersionForApi.length() > 0)
  {
    firmwareCheckUrl += (firmwareCheckUrl.indexOf('?') == -1) ? "?" : "&";
    firmwareCheckUrl += "v=" + appVersionForApi;
  }

  if (!http.begin(client, firmwareCheckUrl))
  {
    syslog->println("Firmware check: begin failed");
    return;
  }

  http.addHeader("User-Agent", String("evDash/") + String(APP_VERSION));
  const int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK)
  {
    syslog->print("Firmware check HTTP code: ");
    syslog->println(httpCode);
    http.end();
    return;
  }

  const String response = http.getString();
  http.end();

  StaticJsonDocument<384> jsonDoc;
  const DeserializationError jsonErr = deserializeJson(jsonDoc, response);
  if (jsonErr)
  {
    syslog->print("Firmware check JSON parse error: ");
    syslog->println(jsonErr.c_str());
    return;
  }

  const char *latestRaw = jsonDoc["version"];
  if (latestRaw == nullptr || latestRaw[0] == '\0')
  {
    syslog->println("Firmware check: missing version field");
    return;
  }

  String latestVersion = String(latestRaw);
  latestVersion.trim();
  if (latestVersion.length() == 0)
  {
    syslog->println("Firmware check: empty version field");
    return;
  }
  if (latestVersion.charAt(0) != 'v' && latestVersion.charAt(0) != 'V')
  {
    latestVersion = String("v") + latestVersion;
  }
  else if (latestVersion.charAt(0) == 'V')
  {
    latestVersion.setCharAt(0, 'v');
  }

  String webFlasherUrl = String(WEBFLASHER_URL);
  const char *webFlasherRaw = jsonDoc["webflasher"];
  if ((webFlasherRaw == nullptr || webFlasherRaw[0] == '\0') && jsonDoc["url"].is<const char *>())
  {
    webFlasherRaw = jsonDoc["url"];
  }
  if (webFlasherRaw != nullptr && webFlasherRaw[0] != '\0')
  {
    webFlasherUrl = String(webFlasherRaw);
    webFlasherUrl.trim();
    if (webFlasherUrl.startsWith("https://"))
    {
      webFlasherUrl.remove(0, 8);
    }
    else if (webFlasherUrl.startsWith("http://"))
    {
      webFlasherUrl.remove(0, 7);
    }
    if (webFlasherUrl.startsWith("www."))
    {
      webFlasherUrl.remove(0, 4);
    }
  }

  String currentVersion = String(APP_VERSION);
  currentVersion.trim();
  if (currentVersion.length() == 0)
  {
    syslog->println("Firmware check: invalid APP_VERSION");
    return;
  }
  if (currentVersion.charAt(0) == 'V')
  {
    currentVersion.setCharAt(0, 'v');
  }

  const int cmp = compareVersionTags(latestVersion, currentVersion);
  if (cmp <= 0)
  {
    syslog->print("Firmware check: up to date (");
    syslog->print(currentVersion);
    syslog->print(" / ");
    syslog->print(latestVersion);
    syslog->println(")");
    return;
  }

  if (latestVersion == lastNotifiedFirmwareVersion)
  {
    syslog->print("Firmware check: already notified ");
    syslog->println(latestVersion);
    return;
  }

  lastNotifiedFirmwareVersion = latestVersion;
  String line2 = String("available ") + latestVersion;
  displayMessage("New version", line2.c_str());
  delay(1800);
  displayMessage("Update here", webFlasherUrl.c_str());
  delay(1800);
}

void Board320_240::addWifiTransferredBytes(size_t bytes)
{
  if (bytes == 0)
  {
    return;
  }
  const uint32_t nowMs = millis();
  const bool transferIconWasIdle =
      (wifiTransferLastActivityMs == 0 ||
       (nowMs - wifiTransferLastActivityMs) > kWifiTransferIndicatorWindowMs);
  wifiTransferLastActivityMs = nowMs;
  liveData->redrawScreenRequested = true;
  if (transferIconWasIdle &&
      currentBrightness != 0 &&
      !liveData->menuVisible &&
      !messageDialogVisible &&
      liveData->params.displayScreen != SCREEN_BLANK)
  {
    redrawScreen();
  }
  if (bytes > (UINT32_MAX - wifiTransferredBytes))
  {
    wifiTransferredBytes = UINT32_MAX;
    return;
  }
  wifiTransferredBytes += static_cast<uint32_t>(bytes);
}

void Board320_240::updateNetAvailability(bool success)
{
  if (success)
  {
    liveData->params.netAvailable = true;
    liveData->params.netLastFailureTime = 0;
    liveData->params.netFailureStartTime = 0;
    liveData->params.netFailureCount = 0;
    dismissedNetFailureTime = 0;
  }
  else
  {
    liveData->params.netAvailable = false;
    liveData->params.netLastFailureTime = (liveData->params.currentTime != 0) ? liveData->params.currentTime : 1;
    if (liveData->params.netFailureStartTime == 0)
    {
      liveData->params.netFailureStartTime = (liveData->params.currentTime != 0) ? liveData->params.currentTime : 1;
    }
    if (liveData->params.netFailureCount < 0xFFFFU)
    {
      liveData->params.netFailureCount++;
    }
  }
}

#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
bool Board320_240::ensureMqttConnected()
{
  if (liveData->settings.mqttEnabled != 1)
  {
    return false;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    return false;
  }
  if (strlen(liveData->settings.mqttServer) == 0 || strcmp(liveData->settings.mqttServer, "not_set") == 0)
  {
    return false;
  }

  // Choose transport
  Client *transport = nullptr;
  if (liveData->settings.mqttUseTls == 1)
  {
    if (mqttSecureClient == nullptr)
    {
      mqttSecureClient = new WiFiClientSecure();
    }
    mqttSecureClient->setInsecure();
    mqttSecureClient->setTimeout(5000);
    transport = mqttSecureClient;
  }
  else
  {
    if (mqttPlainClient == nullptr)
    {
      mqttPlainClient = new WiFiClient();
    }
    mqttPlainClient->setTimeout(5000);
    transport = mqttPlainClient;
  }

  if (mqttClient == nullptr)
  {
    mqttClient = new PubSubClient(*transport);
  }
  else
  {
    mqttClient->setClient(*transport);
  }

  if (mqttClient->connected())
  {
    return true;
  }

  // Check reconnect backoff to avoid blocking main loop on repeated connection failures
  if (lastMqttReconnectAttemptMs != 0 && (millis() - lastMqttReconnectAttemptMs) < kMqttReconnectBackoffMs)
  {
    return false;
  }

  lastMqttReconnectAttemptMs = millis();

  const uint16_t mqttPort = (liveData->settings.mqttPort != 0)
                                ? liveData->settings.mqttPort
                                : ((liveData->settings.mqttUseTls == 1) ? 8883 : 1883);

  syslog->infoNolf(DEBUG_NET, "Connecting to MQTT server: ");
  syslog->infoNolf(DEBUG_NET, liveData->settings.mqttServer);
  syslog->infoNolf(DEBUG_NET, ":");
  syslog->infoNolf(DEBUG_NET, String(mqttPort));
  syslog->info(DEBUG_NET, (liveData->settings.mqttUseTls == 1) ? " (TLS)" : " (plain)");

  mqttClient->setServer(liveData->settings.mqttServer, mqttPort);
  mqttClient->setSocketTimeout(5);
  mqttClient->setBufferSize(768);

  // Last Will and Testament (LWT) topic and message
  char willTopic[96];
  snprintf(willTopic, sizeof(willTopic), "%s/status", liveData->settings.mqttPubTopic);
  const char *willMessage = "offline";
  const uint8_t willQos = 0;
  const bool willRetain = true;

  bool connected = false;
  if (strlen(liveData->settings.mqttUsername) > 0)
  {
    connected = mqttClient->connect(liveData->settings.mqttId,
                                    liveData->settings.mqttUsername,
                                    liveData->settings.mqttPassword,
                                    willTopic, willQos, willRetain, willMessage);
  }
  else
  {
    connected = mqttClient->connect(liveData->settings.mqttId,
                                    willTopic, willQos, willRetain, willMessage);
  }

  if (connected)
  {
    syslog->info(DEBUG_NET, "MQTT connected successfully");
    publishMqttString(*mqttClient, liveData->settings.mqttPubTopic, "/status", "online", true);
    if (liveData->settings.mqttHomeAssistant == 1)
    {
      publishHomeAssistantDiscovery();
    }
    return true;
  }
  else
  {
    const char *stateDesc = "";
    switch (mqttClient->state())
    {
    case -4: stateDesc = " (timeout)"; break;
    case -3: stateDesc = " (connection lost)"; break;
    case -2: stateDesc = " (connect failed / unreachable)"; break;
    case -1: stateDesc = " (disconnected)"; break;
    case 1:  stateDesc = " (bad protocol)"; break;
    case 2:  stateDesc = " (bad client ID)"; break;
    case 3:  stateDesc = " (server unavailable)"; break;
    case 4:  stateDesc = " (bad credentials)"; break;
    case 5:  stateDesc = " (unauthorized)"; break;
    default: break;
    }
    syslog->infoNolf(DEBUG_NET, "MQTT connect failed, state: ");
    syslog->infoNolf(DEBUG_NET, String(mqttClient->state()));
    syslog->info(DEBUG_NET, stateDesc);
    return false;
  }
}

void Board320_240::disconnectMqtt(bool sendOfflineStatus)
{
  if (mqttClient != nullptr)
  {
    if (mqttClient->connected())
    {
      if (sendOfflineStatus && strlen(liveData->settings.mqttPubTopic) > 0)
      {
        publishMqttString(*mqttClient, liveData->settings.mqttPubTopic, "/status", "offline", true);
      }
      mqttClient->disconnect();
    }
  }
  if (mqttSecureClient != nullptr)
  {
    mqttSecureClient->stop();
  }
  if (mqttPlainClient != nullptr)
  {
    mqttPlainClient->stop();
  }
}

void Board320_240::publishHaSensor(const char *component, const char *objectId, const char *name,
                                   const char *deviceClass, const char *unit, const char *stateClass,
                                   const char *entityCategory, const char *payloadOn, const char *payloadOff)
{
  if (mqttClient == nullptr || !mqttClient->connected())
  {
    return;
  }

  const char *rawTopic = liveData->settings.mqttPubTopic;
  const char *baseTopic = (strlen(rawTopic) > 0) ? rawTopic : "evdash";
  const char *rawId = (strlen(liveData->settings.mqttId) > 0) ? liveData->settings.mqttId :
                      ((strlen(rawTopic) > 0) ? rawTopic : "evdash");
  const char *devName = (strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : rawId;

  char devId[64];
  size_t idx = 0;
  for (; rawId[idx] != '\0' && idx < sizeof(devId) - 1; idx++)
  {
    devId[idx] = (rawId[idx] == ' ' || rawId[idx] == '/') ? '_' : rawId[idx];
  }
  devId[idx] = '\0';

  char configTopic[128];
  snprintf(configTopic, sizeof(configTopic), "homeassistant/%s/%s/%s/config", component, devId, objectId);

  StaticJsonDocument<768> doc;
  doc["name"] = name;
  char uniqueId[96];
  snprintf(uniqueId, sizeof(uniqueId), "%s_%s", devId, objectId);
  doc["uniq_id"] = uniqueId;
  doc["object_id"] = uniqueId;
  char defaultEntityId[128];
  snprintf(defaultEntityId, sizeof(defaultEntityId), "%s.%s_%s", component, devId, objectId);
  doc["default_entity_id"] = defaultEntityId;
  doc["has_entity_name"] = true;

  char stateTopic[96];
  snprintf(stateTopic, sizeof(stateTopic), "%s/%s", baseTopic, objectId);
  doc["stat_t"] = stateTopic;

  if (deviceClass != nullptr && strlen(deviceClass) > 0)
  {
    doc["dev_cla"] = deviceClass;
  }
  if (unit != nullptr && strlen(unit) > 0)
  {
    doc["unit_of_meas"] = unit;
  }
  if (stateClass != nullptr && strlen(stateClass) > 0)
  {
    doc["stat_cla"] = stateClass;
  }
  if (entityCategory != nullptr && strlen(entityCategory) > 0)
  {
    doc["ent_cat"] = entityCategory;
  }
  if (payloadOn != nullptr)
  {
    doc["pl_on"] = payloadOn;
  }
  if (payloadOff != nullptr)
  {
    doc["pl_off"] = payloadOff;
  }

  char availTopic[96];
  snprintf(availTopic, sizeof(availTopic), "%s/status", baseTopic);
  doc["avty_t"] = availTopic;
  doc["pl_avail"] = "online";
  doc["pl_not_avail"] = "offline";

  JsonObject dev = doc.createNestedObject("dev");
  JsonArray ids = dev.createNestedArray("ids");
  ids.add(devId);
  dev["name"] = devName;

  const char *rawModel = (strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel :
                         ((strlen(rawTopic) > 0) ? rawTopic :
#if defined(BOARD_M5STACK_CORES3)
                          "CoreS3"
#else
                          "Core2"
#endif
                         );
  dev["mdl"] = rawModel;
  dev["mf"] = "evDash";

  const char *swVer = (APP_VERSION[0] == 'v') ? (APP_VERSION + 1) : APP_VERSION;
  dev["sw"] = swVer;

  char payload[768];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  if (len > 0 && len < sizeof(payload))
  {
    mqttClient->publish(configTopic, payload, true);
    mqttClient->loop();
  }
}

void Board320_240::publishHomeAssistantDiscovery()
{
  if (mqttClient == nullptr || !mqttClient->connected())
  {
    if (liveData->settings.mqttEnabled == 1)
    {
      ensureMqttConnected();
    }
    return;
  }

  syslog->info(DEBUG_NET, "Publishing Home Assistant MQTT discovery entities...");

  // Battery
  publishHaSensor("sensor", "soc", "Battery State of Charge", "battery", "%", "measurement");
  publishHaSensor("sensor", "soc_kwh", "Battery SoC in kWh", "energy", "kWh", "measurement");
  publishHaSensor("sensor", "soh", "Battery State of Health", "battery", "%", "measurement");
  publishHaSensor("sensor", "bat_power", "Battery Power", "power", "kW", "measurement");
  publishHaSensor("sensor", "bat_current", "Battery Current", "current", "A", "measurement");
  publishHaSensor("sensor", "bat_voltage", "Battery Voltage", "voltage", "V", "measurement");
  publishHaSensor("sensor", "bat_temp", "Battery Temperature", "temperature", "°C", "measurement");
  publishHaSensor("sensor", "cell_temp_min", "Cell Temperature Min", "temperature", "°C", "measurement");
  publishHaSensor("sensor", "cell_temp_max", "Cell Temperature Max", "temperature", "°C", "measurement");
  publishHaSensor("sensor", "cell_voltage_min", "Cell Voltage Min", "voltage", "V", "measurement");
  publishHaSensor("sensor", "cell_voltage_max", "Cell Voltage Max", "voltage", "V", "measurement");

  // 12V Aux
  publishHaSensor("sensor", "aux_voltage", "Aux 12V Battery Voltage", "voltage", "V", "measurement");
  publishHaSensor("sensor", "aux_current", "Aux 12V Battery Current", "current", "A", "measurement");
  publishHaSensor("sensor", "aux_soc", "Aux 12V State of Charge", "battery", "%", "measurement");

  // Charging
  publishHaSensor("binary_sensor", "charging_on", "Charging", "battery_charging", nullptr, nullptr, nullptr, "1", "0");
  publishHaSensor("binary_sensor", "charger_ac_connected", "Charger AC Connected", "plug", nullptr, nullptr, nullptr, "1", "0");
  publishHaSensor("binary_sensor", "charger_dc_connected", "Charger DC Fast Charging", "plug", nullptr, nullptr, nullptr, "1", "0");
  publishHaSensor("sensor", "charger_power", "Station Power", "power", "kW", "measurement");
  publishHaSensor("sensor", "charger_voltage", "Station Voltage", "voltage", "V", "measurement");
  publishHaSensor("sensor", "charger_current", "Station Current", "current", "A", "measurement");
  publishHaSensor("sensor", "charged_session_energy", "Charging Session Energy", "energy", "kWh", "measurement");
  publishHaSensor("sensor", "cumulative_energy_charged", "Total Charged Energy", "energy", "kWh", "total_increasing");

  // Driving & Drivetrain
  publishHaSensor("binary_sensor", "ignition_on", "Ignition", "power", nullptr, nullptr, nullptr, "1", "0");
  publishHaSensor("sensor", "speed", "Vehicle Speed", "speed", "km/h", "measurement");
  publishHaSensor("sensor", "odometer", "Odometer", "distance", "km", "total_increasing");
  publishHaSensor("sensor", "trip_distance", "Trip Distance", "distance", "km", "measurement");
  publishHaSensor("sensor", "avg_speed", "Average Speed", "speed", "km/h", "measurement");
  publishHaSensor("sensor", "consumption", "Energy Consumption", nullptr, "kWh/100km", "measurement");
  publishHaSensor("sensor", "discharged_session_energy", "Trip Discharged Energy", "energy", "kWh", "measurement");
  publishHaSensor("sensor", "cumulative_energy_discharged", "Total Discharged Energy", "energy", "kWh", "total_increasing");
  publishHaSensor("sensor", "motor1_rpm", "Motor 1 RPM", nullptr, "rpm", "measurement");
  publishHaSensor("sensor", "motor2_rpm", "Motor 2 RPM", nullptr, "rpm", "measurement");
  publishHaSensor("sensor", "motor1_torque", "Motor 1 Torque", nullptr, "Nm", "measurement");
  publishHaSensor("sensor", "motor2_torque", "Motor 2 Torque", nullptr, "Nm", "measurement");
  publishHaSensor("sensor", "motor_temp", "Motor Temperature", "temperature", "°C", "measurement");
  publishHaSensor("sensor", "inverter_temp", "Inverter Temperature", "temperature", "°C", "measurement");

  // Tires
  publishHaSensor("sensor", "tire_pressure_fl", "Tire Pressure Front Left", "pressure", "bar", "measurement");
  publishHaSensor("sensor", "tire_pressure_fr", "Tire Pressure Front Right", "pressure", "bar", "measurement");
  publishHaSensor("sensor", "tire_pressure_rl", "Tire Pressure Rear Left", "pressure", "bar", "measurement");
  publishHaSensor("sensor", "tire_pressure_rr", "Tire Pressure Rear Right", "pressure", "bar", "measurement");

  // Environment
  publishHaSensor("sensor", "outdoor_temp", "Outdoor Temperature", "temperature", "°C", "measurement");

  // GPS
  publishHaSensor("sensor", "gps_lat", "GPS Latitude", nullptr, "°", nullptr);
  publishHaSensor("sensor", "gps_lon", "GPS Longitude", nullptr, "°", nullptr);
  publishHaSensor("sensor", "gps_speed", "GPS Speed", "speed", "km/h", "measurement");
  publishHaSensor("sensor", "gps_alt", "GPS Altitude", "distance", "m", nullptr);
  publishHaSensor("sensor", "gps_heading", "GPS Heading", nullptr, "°", "measurement");

  // Diagnostics
  publishHaSensor("sensor", "status", "Device Status", nullptr, nullptr, nullptr, "diagnostic");
  publishHaSensor("binary_sensor", "car_connected", "Car Connected", "connectivity", nullptr, nullptr, "diagnostic", "1", "0");
  publishHaSensor("sensor", "wifi_rssi", "WiFi Signal", "signal_strength", "dBm", "measurement", "diagnostic");
  publishHaSensor("sensor", "uptime", "Uptime", "duration", "s", "measurement", "diagnostic");

  syslog->info(DEBUG_NET, "Home Assistant MQTT discovery published (retained).");
}
#else
bool Board320_240::ensureMqttConnected() { return false; }
void Board320_240::disconnectMqtt(bool sendOfflineStatus) {}
void Board320_240::publishHomeAssistantDiscovery() {}
void Board320_240::publishHaSensor(const char *, const char *, const char *, const char *, const char *, const char *, const char *, const char *, const char *) {}
#endif

/**
 * Net loop, send data over net
 * Checks if WiFi is connected, syncs NTP if needed, sends data to remote API if interval elapsed,
 * sends data to ABRP if interval elapsed, contributes data if enabled.
 */
void Board320_240::netLoop()
{
  if (liveData->params.wifiApMode)
  {
    return;
  }

  bool wifiReady = (liveData->settings.wifiEnabled == 1 && WiFi.status() == WL_CONNECTED);
  if (!wifiReady)
  {
    liveData->params.netAvailable = true;
    liveData->params.netLastFailureTime = 0;
    liveData->params.netFailureStartTime = 0;
    liveData->params.netFailureCount = 0;
    dismissedNetFailureTime = 0;
    disconnectMqtt(false);
  }
#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
  else if (liveData->settings.mqttEnabled == 1)
  {
    if (mqttClient != nullptr && mqttClient->connected())
    {
      mqttClient->loop();
    }
  }
#endif

  // Avoid stale "Net unavailable" state when no internet uploader is effectively active.
  const auto remoteApiConfigured = [this]() -> bool
  {
    if (liveData->settings.mqttEnabled == 1)
    {
      const char *server = liveData->settings.mqttServer;
      return server != nullptr && server[0] != '\0' && strcmp(server, "not_set") != 0;
    }
    if (liveData->settings.remoteUploadIntervalSec == 0)
    {
      return false;
    }
    const char *url = liveData->settings.remoteApiUrl;
    if (url == nullptr || url[0] == '\0')
    {
      return false;
    }
    if (strcmp(url, "not_set") == 0)
    {
      return false;
    }
    return strstr(url, "http") != nullptr;
  }();

  const auto abrpConfigured = [this]() -> bool
  {
    if (liveData->settings.remoteUploadAbrpIntervalSec == 0)
    {
      return false;
    }
    const char *token = liveData->settings.abrpApiToken;
    if (token == nullptr || token[0] == '\0')
    {
      return false;
    }
    if (strcmp(token, "empty") == 0 || strcmp(token, "not_set") == 0)
    {
      return false;
    }
    return true;
  }();

  const bool traccarConfigured = (liveData->settings.traccarEnabled == 1);
  const bool contributeConfigured = (liveData->settings.contributeData == 1);
  const bool internetTasksActive = remoteApiConfigured || abrpConfigured || traccarConfigured || contributeConfigured;

  if (wifiReady && !internetTasksActive)
  {
    liveData->params.netAvailable = true;
    liveData->params.netLastFailureTime = 0;
    liveData->params.netFailureStartTime = 0;
    liveData->params.netFailureCount = 0;
    dismissedNetFailureTime = 0;
  }
  else if (wifiReady && liveData->params.netAvailable == false &&
           liveData->params.netLastFailureTime != 0 &&
           liveData->params.currentTime >= liveData->params.netLastFailureTime &&
           (liveData->params.currentTime - liveData->params.netLastFailureTime) > kNetFailureStaleResetSec)
  {
    // Last error is too old, clear status and wait for the next real send attempt.
    liveData->params.netAvailable = true;
    liveData->params.netLastFailureTime = 0;
    liveData->params.netFailureStartTime = 0;
    liveData->params.netFailureCount = 0;
    dismissedNetFailureTime = 0;
  }

  bool netBackoffActive = (!liveData->params.netAvailable &&
                           liveData->params.netLastFailureTime != 0 &&
                           liveData->params.currentTime >= liveData->params.netLastFailureTime &&
                           (liveData->params.currentTime - liveData->params.netLastFailureTime) < kNetRetryIntervalSec);
  bool netReady = wifiReady && !netBackoffActive;

  if (!liveData->params.ntpTimeSet)
  {
    if (ntpAttemptStartMs == 0)
    {
      ntpAttemptStartMs = millis();
      gpsTimeFallbackAllowed = false;
    }
    else if (!gpsTimeFallbackAllowed && (millis() - ntpAttemptStartMs) >= kNtpPriorityWindowMs)
    {
      gpsTimeFallbackAllowed = true;
      syslog->println("NTP sync timeout (60s), falling back to GPS time.");
    }
  }

  // Sync NTP first, retry for up to 60 seconds before GPS time fallback.
  if (netReady && !liveData->params.ntpTimeSet && liveData->settings.ntpEnabled &&
      (millis() - ntpAttemptStartMs) < kNtpPriorityWindowMs &&
      (ntpLastAttemptMs == 0 || (millis() - ntpLastAttemptMs) >= kNtpRetryIntervalMs))
  {
    ntpLastAttemptMs = millis();
    ntpSync();
  }

  if (!liveData->params.ntpTimeSet && liveData->settings.ntpEnabled &&
      (s_ntpSyncCompleted || sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED))
  {
    s_ntpSyncCompleted = false;
    liveData->params.ntpTimeSet = true;
    syslog->println("NTP time synchronized.");
    showTime();
    syncRtcFromSystemTime();
  }

  if (liveData->params.ntpTimeSet)
  {
    gpsTimeFallbackAllowed = false;
  }

  // Upload to custom API or MQTT
  uint16_t remoteInterval = liveData->settings.remoteUploadIntervalSec;
  if (remoteInterval == 0 && liveData->settings.mqttEnabled == 1)
  {
    remoteInterval = 60; // Default 60s for MQTT if API interval is set to 0/off
  }
  const uint32_t remoteIntervalMs = static_cast<uint32_t>(remoteInterval) * 1000U;
  if (netReady && remoteApiConfigured && remoteIntervalMs > 0 &&
      (lastRemoteSendAtMs == 0 || (millis() - lastRemoteSendAtMs) > remoteIntervalMs))
  {
    lastRemoteSendAtMs = millis();
    liveData->params.lastRemoteApiSent = liveData->params.currentTime;
    syslog->info(DEBUG_NET, (liveData->settings.mqttEnabled == 1) ? "MQTT send tick" : "Remote send tick");
    int64_t startTime = esp_timer_get_time();
    netSendData(false);
    int64_t endTime = esp_timer_get_time();
    lastNetSendDurationMs = static_cast<uint32_t>((endTime - startTime) / 1000);
  }

  // Upload to ABRP (interval stored in 0.5-second steps: 1 => 0.5s, 10 => 5.0s)
  if (netReady && abrpConfigured)
  {
    const uint32_t abrpIntervalMs = static_cast<uint32_t>(liveData->settings.remoteUploadAbrpIntervalSec) * 500;
    if (abrpIntervalMs > 0 && (lastAbrpSendAtMs == 0 || (millis() - lastAbrpSendAtMs) > abrpIntervalMs))
    {
      syslog->info(DEBUG_ABRP, "ABRP send tick");
      int64_t startTime = esp_timer_get_time();
      netSendData(true);
      int64_t endTime = esp_timer_get_time();
      lastNetSendDurationMs = static_cast<uint32_t>((endTime - startTime) / 1000);
    }
  }

  // Upload GPS position to Traccar (default every 5 seconds)
  if (netReady && traccarConfigured)
  {
    const bool hasGpsFix = isGpsFixUsable(liveData);
    if (!hasGpsFix)
    {
      syslog->info(DEBUG_NET, "Traccar send skipped: no GPS fix");
    }
    if (hasGpsFix && (lastTraccarSendAtMs == 0 || (millis() - lastTraccarSendAtMs) > kTraccarIntervalMs))
    {
      const String traccarDeviceId = normalizeDeviceIdForApi(getTraccarDeviceIdFromEfuse());
      String traccarServerHost = String(liveData->settings.traccarServerHost);
      traccarServerHost.trim();
      syslog->info(DEBUG_NET, "Traccar deviceId: " + traccarDeviceId);
      bool sentOk = false;
      int httpCode = -1;
      if (traccarServerHost.length() == 0 || traccarServerHost == "empty")
      {
        syslog->info(DEBUG_NET, "Traccar send skipped: no server host");
      }
      else
      {
        const uint16_t port = liveData->settings.traccarServerPort ? liveData->settings.traccarServerPort : 5055;
        syslog->info(DEBUG_NET, "Traccar send tick (" + traccarServerHost + ":" + String(port) + ")");
        sentOk = Traccar::sendPosition(traccarServerHost.c_str(),
                                       port,
                                       traccarDeviceId,
                                       liveData->params.currentTime,
                                       liveData->params.gpsLat,
                                       liveData->params.gpsLon,
                                       liveData->params.speedKmhGPS,
                                       liveData->params.gpsAlt,
                                       liveData->params.gpsHeadingDeg,
                                       liveData->params.socPerc,
                                       liveData->params.chargingOn,
                                       httpCode);
      }

      lastTraccarSendAtMs = millis();
      if (sentOk)
      {
        liveData->params.lastSuccessNetSendTime = liveData->params.currentTime;
        updateNetAvailability(true);
      }
      else
      {
        syslog->info(DEBUG_NET, "Traccar send failed, HTTP=" + String(httpCode));
        updateNetAvailability(false);
      }
    }
  }

  // Contribute data
  const uint32_t nowMs = millis();
  if (liveData->settings.contributeData == 0)
  {
    liveData->params.contributeStatus = CONTRIBUTE_NONE;
    contributeStatusSinceMs = 0;
    nextContributeCycleAtMs = 0;
  }
  if (nextContributeCycleAtMs == 0)
  {
    nextContributeCycleAtMs = nowMs;
  }
  if (netReady && liveData->settings.contributeData == 1 &&
      liveData->params.contributeStatus == CONTRIBUTE_NONE &&
      static_cast<int32_t>(nowMs - nextContributeCycleAtMs) >= 0)
  {
    liveData->params.lastContributeSent = liveData->params.currentTime;
    liveData->params.contributeStatus = CONTRIBUTE_WAITING;
    contributeStatusSinceMs = nowMs;
  }
  bool allowContributeWaitFallback = false;
  if (liveData->settings.commType == COMM_TYPE_CAN_COMMU && commInterface != nullptr)
  {
    const String canStatus = commInterface->getConnectStatus();
    allowContributeWaitFallback = (canStatus.indexOf("No MCP2515") != -1);
  }
  if (netReady && liveData->settings.contributeData == 1 &&
      liveData->params.contributeStatus == CONTRIBUTE_WAITING &&
      allowContributeWaitFallback &&
      contributeStatusSinceMs != 0 &&
      (millis() - contributeStatusSinceMs) >= kContributeWaitFallbackMs)
  {
    syslog->info(DEBUG_NET, "contributeStatus ... waiting timeout fallback to ready");
    liveData->params.contributeStatus = CONTRIBUTE_READY_TO_SEND;
  }
  if (netReady && liveData->settings.contributeData == 1 &&
      liveData->params.contributeStatus == CONTRIBUTE_READY_TO_SEND)
  {
    netContributeData();
  }

  runSdV2BackgroundTasks(netReady);
}

/**
 * Send data
 **/
bool Board320_240::netSendData(bool sendAbrp)
{
  int64_t startTime2 = esp_timer_get_time();
  int rc = 0;
  const bool wifiReady = (liveData->settings.wifiEnabled == 1 && WiFi.status() == WL_CONNECTED);
  const String contributeKey = ensureContributeKey();
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());

  // For ABRP and custom HTTP POST API, valid car data (socPerc >= 0) is mandatory.
  // For MQTT, we still allow sending heartbeat, device status, and GPS even if car data is not yet available.
  const bool isMqtt = (!sendAbrp && liveData->settings.mqttEnabled == 1);
  if (!isMqtt && liveData->params.socPerc < 0)
  {
    syslog->info(sendAbrp ? DEBUG_ABRP : DEBUG_NET, "No valid data, skipping data send");
    return false;
  }

  // WIFI
  if (liveData->settings.remoteUploadModuleType == 1)
  {
    syslog->info(DEBUG_NET, (liveData->settings.mqttEnabled == 1) ? "Sending data via MQTT - via WIFI" : "Sending data to API - via WIFI");
  }
  else
  {
    if (liveData->settings.remoteUploadModuleType != 0)
    {
      syslog->info(DEBUG_NET, "Unsupported module");
    }
    return false;
  }

  if (!wifiReady)
  {
    syslog->info(DEBUG_NET, "WiFi not connected, skipping data send");
    return false;
  }

  if (!sendAbrp && (liveData->settings.remoteUploadIntervalSec != 0 || liveData->settings.mqttEnabled == 1))
  {
    if (liveData->settings.mqttEnabled != 1)
    {
      syslog->info(DEBUG_NET, "Start HTTP POST...");
    }
    if (liveData->settings.mqttEnabled == 1)
    {
      if (strlen(liveData->settings.mqttServer) == 0 ||
          strcmp(liveData->settings.mqttServer, "not_set") == 0)
      {
        syslog->info(DEBUG_NET, "MQTT server not set, skipping send");
        return false;
      }
    }
    else
    {
      if (strlen(liveData->settings.remoteApiUrl) == 0 ||
          strcmp(liveData->settings.remoteApiUrl, "not_set") == 0 ||
          strstr(liveData->settings.remoteApiUrl, "http") == nullptr)
      {
        syslog->info(DEBUG_NET, "Remote API URL not set, skipping send");
        return false;
      }
    }

    StaticJsonDocument<768> jsonData;

    jsonData["apikey"] = liveData->settings.remoteApiKey;
    jsonData["deviceKey"] = contributeKey;
    jsonData["deviceId"] = hardwareDeviceId;
    jsonData["carType"] = liveData->settings.carType;
    jsonData["ignitionOn"] = liveData->params.ignitionOn;
    jsonData["chargingOn"] = liveData->params.chargingOn;
    jsonData["chargingDc"] = liveData->params.chargerDCconnected;
    jsonData["socPerc"] = liveData->params.socPerc;
    if (liveData->params.socPercBms != -1)
      jsonData["socPercBms"] = liveData->params.socPercBms;
    jsonData["sohPerc"] = liveData->params.sohPerc;
    jsonData["batPowerKw"] = liveData->params.batPowerKw;
    jsonData["batPowerAmp"] = liveData->params.batPowerAmp;
    jsonData["batVoltage"] = liveData->params.batVoltage;
    jsonData["auxVoltage"] = liveData->params.auxVoltage;
    jsonData["auxAmp"] = liveData->params.auxCurrentAmp;
    jsonData["batMinC"] = liveData->params.batMinC;
    jsonData["batMaxC"] = liveData->params.batMaxC;
    jsonData["batInletC"] = liveData->params.batInletC;
    jsonData["extTemp"] = liveData->params.outdoorTemperature;
    jsonData["batFanStatus"] = liveData->params.batFanStatus;
    jsonData["speedKmh"] = liveData->params.speedKmh;
    jsonData["odoKm"] = liveData->params.odoKm;
    jsonData["cumulativeEnergyChargedKWh"] = liveData->params.cumulativeEnergyChargedKWh;
    jsonData["cumulativeEnergyDischargedKWh"] = liveData->params.cumulativeEnergyDischargedKWh;

    // Send GPS data via GPRS (if enabled && valid)
    if (isGpsFixUsable(liveData))
    {
      jsonData["gpsLat"] = liveData->params.gpsLat;
      jsonData["gpsLon"] = liveData->params.gpsLon;
      jsonData["gpsAlt"] = liveData->params.gpsAlt;
      jsonData["gpsSpeed"] = liveData->params.speedKmhGPS;
      if (liveData->params.gpsHeadingDeg >= 0)
        jsonData["gpsHeading"] = liveData->params.gpsHeadingDeg;
    }

    char payload[768];
    size_t payloadLen = measureJson(jsonData);
    if (payloadLen >= sizeof(payload))
    {
      syslog->info(DEBUG_NET, "Remote API payload too large, skipping send");
      return false;
    }
    serializeJson(jsonData, payload, sizeof(payload));

    if (liveData->settings.mqttEnabled != 1)
    {
      syslog->infoNolf(DEBUG_NET, "Sending payload: ");
      syslog->info(DEBUG_NET, payload);

      syslog->infoNolf(DEBUG_NET, "Remote API server: ");
      syslog->info(DEBUG_NET, liveData->settings.remoteApiUrl);
    }

    // WIFI remote upload
    rc = 0;
    if (liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI && liveData->settings.wifiEnabled == 1)
    {
      if (liveData->settings.mqttEnabled == 1)
      {
#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
        if (ensureMqttConnected())
        {
          bool published = true;
          // Device & connection heartbeat (always sent when connected to MQTT broker)
          published &= publishMqttString(*mqttClient, liveData->settings.mqttPubTopic, "/status", "online", true);
          published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/uptime", millis() / 1000);
          published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/wifi_rssi", WiFi.RSSI());
          published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/car_connected", (liveData->params.socPerc >= 0) ? 1 : 0);

          // Car telemetry (only when car CAN/BLE is communicating and socPerc >= 0)
          if (liveData->params.socPerc >= 0)
          {
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/soc", liveData->params.socPerc, 2, true);
            published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/charging_on", liveData->params.chargingOn ? 1 : 0);
            published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/ignition_on", liveData->params.ignitionOn ? 1 : 0);
            published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/charger_ac_connected", liveData->params.chargerACconnected ? 1 : 0);
            published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/charger_dc_connected", liveData->params.chargerDCconnected ? 1 : 0);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/bat_power", liveData->params.batPowerKw);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/bat_current", liveData->params.batPowerAmp);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/bat_voltage", liveData->params.batVoltage);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/aux_voltage", liveData->params.auxVoltage);
            if (liveData->params.batMinC > -90.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cell_temp_min", liveData->params.batMinC);
            }
            if (liveData->params.batMaxC > -90.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cell_temp_max", liveData->params.batMaxC);
            }
            float batTemp = (liveData->params.batTempC > -90.0f) ? liveData->params.batTempC :
                            ((liveData->params.batMinC > -90.0f && liveData->params.batMaxC > -90.0f) ?
                             (liveData->params.batMinC + liveData->params.batMaxC) / 2.0f : -100.0f);
            if (batTemp > -90.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/bat_temp", batTemp);
            }
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/outdoor_temp", liveData->params.outdoorTemperature);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/speed", liveData->params.speedKmh);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/odometer", liveData->params.odoKm);

            float socKwh = (liveData->params.batEnergyContent > 0) ? liveData->params.batEnergyContent :
                           ((liveData->params.batteryTotalAvailableKWh > 0) ?
                            (liveData->params.batteryTotalAvailableKWh * liveData->params.socPerc / 100.0f) : -1.0f);
            if (socKwh >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/soc_kwh", socKwh, 1, true);
            }
            if (liveData->params.sohPerc >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/soh", liveData->params.sohPerc, 1, true);
            }
            if (liveData->params.batCellMinV > 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cell_voltage_min", liveData->params.batCellMinV, 3);
            }
            if (liveData->params.batCellMaxV > 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cell_voltage_max", liveData->params.batCellMaxV, 3);
            }
            if (liveData->params.auxCurrentAmp > -900.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/aux_current", liveData->params.auxCurrentAmp, 2);
            }
            if (liveData->params.auxPerc >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/aux_soc", liveData->params.auxPerc, 0);
            }

            // Charging stats
            if (liveData->params.chargerVoltage > 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/charger_voltage", liveData->params.chargerVoltage, 1);
            }
            if (liveData->params.chargerCurrent > 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/charger_current", liveData->params.chargerCurrent, 1);
            }
            if (liveData->params.chargerVoltage > 0 && liveData->params.chargerCurrent > 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/charger_power", (liveData->params.chargerVoltage * liveData->params.chargerCurrent) / 1000.0f, 2);
            }
            if (liveData->params.cumulativeEnergyChargedKWh >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cumulative_energy_charged", liveData->params.cumulativeEnergyChargedKWh, 1);
              if (liveData->params.cumulativeEnergyChargedKWhStart >= 0 &&
                  (liveData->params.cumulativeEnergyChargedKWh - liveData->params.cumulativeEnergyChargedKWhStart) >= 0)
              {
                published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/charged_session_energy", liveData->params.cumulativeEnergyChargedKWh - liveData->params.cumulativeEnergyChargedKWhStart, 2);
              }
            }

            // Driving & Drivetrain stats
            if (liveData->params.motor1Rpm >= 0)
            {
              published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/motor1_rpm", round(liveData->params.motor1Rpm));
            }
            if (liveData->params.motor2Rpm >= 0)
            {
              published &= publishMqttInt(*mqttClient, liveData->settings.mqttPubTopic, "/motor2_rpm", round(liveData->params.motor2Rpm));
            }
            if (liveData->params.motor1TorqueNm > -500.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/motor1_torque", liveData->params.motor1TorqueNm, 1);
            }
            if (liveData->params.motor2TorqueNm > -500.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/motor2_torque", liveData->params.motor2TorqueNm, 1);
            }
            if (liveData->params.motorTempC > -90.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/motor_temp", liveData->params.motorTempC, 1);
            }
            if (liveData->params.inverterTempC > -90.0f)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/inverter_temp", liveData->params.inverterTempC, 1);
            }
            if (liveData->params.batPowerKwh100 >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/consumption", liveData->params.batPowerKwh100, 2);
            }
            if (liveData->params.avgSpeedKmh >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/avg_speed", liveData->params.avgSpeedKmh, 1);
            }
            if (liveData->params.odoKm >= 0 && liveData->params.odoKmStart >= 0 &&
                (liveData->params.odoKm - liveData->params.odoKmStart) >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/trip_distance", liveData->params.odoKm - liveData->params.odoKmStart, 1);
            }
            if (liveData->params.cumulativeEnergyDischargedKWh >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/cumulative_energy_discharged", liveData->params.cumulativeEnergyDischargedKWh, 1);
              if (liveData->params.cumulativeEnergyDischargedKWhStart >= 0 &&
                  (liveData->params.cumulativeEnergyDischargedKWh - liveData->params.cumulativeEnergyDischargedKWhStart) >= 0)
              {
                published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/discharged_session_energy", liveData->params.cumulativeEnergyDischargedKWh - liveData->params.cumulativeEnergyDischargedKWhStart, 2);
              }
            }

            // Tires
            if (liveData->params.tireFrontLeftPressureBar >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/tire_pressure_fl", liveData->params.tireFrontLeftPressureBar, 2, true);
            }
            if (liveData->params.tireFrontRightPressureBar >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/tire_pressure_fr", liveData->params.tireFrontRightPressureBar, 2, true);
            }
            if (liveData->params.tireRearLeftPressureBar >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/tire_pressure_rl", liveData->params.tireRearLeftPressureBar, 2, true);
            }
            if (liveData->params.tireRearRightPressureBar >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/tire_pressure_rr", liveData->params.tireRearRightPressureBar, 2, true);
            }
          }

          // Send GPS data via GPRS (if enabled && valid)
          if (isGpsFixUsable(liveData))
          {
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/gps_lat", liveData->params.gpsLat, 5);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/gps_lon", liveData->params.gpsLon, 5);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/gps_speed", liveData->params.speedKmhGPS);
            published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/gps_alt", liveData->params.gpsAlt);

            if (liveData->params.gpsHeadingDeg >= 0)
            {
              published &= publishMqttFloat(*mqttClient, liveData->settings.mqttPubTopic, "/gps_heading", liveData->params.gpsHeadingDeg);
            }
          }
          rc = published ? 200 : -1;
        }
        else
        {
          rc = -1;
        }
#else
        rc = -1;
#endif
      }
      else
      {
        // Standard http post
        WiFiClient wClient;
        HTTPClient http;

        http.begin(wClient, liveData->settings.remoteApiUrl);
        http.setConnectTimeout(500);
        http.addHeader("Content-Type", "application/json");
        addWifiTransferredBytes(payloadLen);
        rc = http.POST(payload);
        http.end();
      }
    }

    if (rc == 200)
    {
      syslog->info(DEBUG_NET, (liveData->settings.mqttEnabled == 1) ? "MQTT send successful" : "HTTP POST send successful");
      liveData->params.lastSuccessNetSendTime = liveData->params.currentTime;
      updateNetAvailability(true);
    }
    else
    {
      // Failed...
      syslog->infoNolf(DEBUG_NET, (liveData->settings.mqttEnabled == 1) ? "MQTT send error: " : "HTTP POST error: ");
      syslog->info(DEBUG_NET, rc);
      updateNetAvailability(false);
    }
  }
  else if (sendAbrp && liveData->settings.remoteUploadAbrpIntervalSec != 0)
  {
    if (strlen(liveData->settings.abrpApiToken) == 0 ||
        strcmp(liveData->settings.abrpApiToken, "empty") == 0 ||
        strcmp(liveData->settings.abrpApiToken, "not_set") == 0)
    {
      syslog->info(DEBUG_ABRP, "ABRP token not set, skipping send");
      return false;
    }

    StaticJsonDocument<768> jsonData;

    jsonData["car_model"] = getCarModelAbrpStr(liveData->settings.carType);
    if (strcmp(jsonData["car_model"], "n/a") == 0)
    {
      syslog->info(DEBUG_ABRP, "Car not supported by ABRP Uploader");
      return false;
    }

    // evDash uses negative values for discharge/consumption.
    // ABRP expects the opposite sign convention (consumption positive, charge/regen negative).
    const float abrpPowerKw = -liveData->params.batPowerKw;
    const float abrpCurrentA = -liveData->params.batPowerAmp;

    jsonData["utc"] = liveData->params.currentTime;
    jsonData["soc"] = liveData->params.socPerc;
    jsonData["power"] = abrpPowerKw;
    jsonData["is_parked"] = (liveData->params.parkModeOrNeutral) ? 1 : 0;
    if (liveData->params.speedKmhGPS > 0)
    {
      jsonData["speed"] = liveData->params.speedKmhGPS;
    }
    else
    {
      jsonData["speed"] = liveData->params.speedKmh;
    }
    jsonData["is_charging"] = (liveData->params.chargingOn) ? 1 : 0;
    if (liveData->params.chargingOn)
      jsonData["is_dcfc"] = (liveData->params.chargerDCconnected) ? 1 : 0;

    if (isGpsFixUsable(liveData))
    {
      jsonData["lat"] = liveData->params.gpsLat;
      jsonData["lon"] = liveData->params.gpsLon;
      jsonData["elevation"] = liveData->params.gpsAlt;
    }
    if (liveData->params.gpsHeadingDeg >= 0)
      jsonData["heading"] = liveData->params.gpsHeadingDeg;

    if (liveData->params.tireFrontLeftPressureBar >= 0)
      jsonData["tire_pressure_fl"] = liveData->params.tireFrontLeftPressureBar * 100.0f;
    if (liveData->params.tireFrontRightPressureBar >= 0)
      jsonData["tire_pressure_fr"] = liveData->params.tireFrontRightPressureBar * 100.0f;
    if (liveData->params.tireRearLeftPressureBar >= 0)
      jsonData["tire_pressure_rl"] = liveData->params.tireRearLeftPressureBar * 100.0f;
    if (liveData->params.tireRearRightPressureBar >= 0)
      jsonData["tire_pressure_rr"] = liveData->params.tireRearRightPressureBar * 100.0f;

    jsonData["capacity"] = liveData->params.batteryTotalAvailableKWh;
    jsonData["kwh_charged"] = liveData->params.cumulativeEnergyChargedKWh;
    jsonData["soh"] = liveData->params.sohPerc;
    jsonData["ext_temp"] = liveData->params.outdoorTemperature;
    if (liveData->params.indoorTemperature != -100)
    {
      jsonData["cabin_temp"] = liveData->params.indoorTemperature;
    }
    jsonData["batt_temp"] = liveData->params.batMinC;
    jsonData["voltage"] = liveData->params.batVoltage;
    jsonData["current"] = abrpCurrentA;
    if (liveData->params.odoKm > 0)
      jsonData["odometer"] = liveData->params.odoKm;

    size_t payloadLength = serializeJson(jsonData, gAbrpPayloadBuffer, sizeof(gAbrpPayloadBuffer));
    if (payloadLength == 0)
    {
      syslog->info(DEBUG_ABRP, "Failed to serialize ABRP payload");
      return false;
    }

    const size_t payloadStringLength = strlen(gAbrpPayloadBuffer);
    syslog->info(DEBUG_ABRP, "ABRP payload length (serializeJson): " + String(payloadLength));
    syslog->info(DEBUG_ABRP, "ABRP payload length (strlen): " + String(payloadStringLength));
    if (payloadStringLength != payloadLength)
    {
      syslog->info(DEBUG_ABRP, "ABRP payload length mismatch detected");
    }
    syslog->info(DEBUG_ABRP, "ABRP payload JSON: " + String(gAbrpPayloadBuffer));

    if (liveData->settings.abrpSdcardLog != 0 && liveData->settings.remoteUploadAbrpIntervalSec > 0)
    {
      queueAbrpSdLog(gAbrpPayloadBuffer, payloadLength, liveData->params.currentTime, liveData->params.operationTimeSec, liveData->params.currTimeSyncWithGps);
    }

    encodeQuotes(gAbrpEncodedPayloadBuffer, sizeof(gAbrpEncodedPayloadBuffer), gAbrpPayloadBuffer);
    syslog->info(DEBUG_ABRP, "ABRP encoded payload length: " + String(strlen(gAbrpEncodedPayloadBuffer)));

    int dtaLength = snprintf(gAbrpFormBuffer, sizeof(gAbrpFormBuffer), "api_key=%s&token=%s&tlm=%s", ABRP_API_KEY, liveData->settings.abrpApiToken, gAbrpEncodedPayloadBuffer);
    if (dtaLength < 0 || static_cast<size_t>(dtaLength) >= sizeof(gAbrpFormBuffer))
    {
      syslog->info(DEBUG_ABRP, "ABRP payload too large, skipping send");
      return false;
    }

    syslog->info(DEBUG_ABRP, "ABRP form payload length: " + String(dtaLength));
    syslog->infoNolf(DEBUG_ABRP, "Sending data: ");
    syslog->info(DEBUG_ABRP, gAbrpFormBuffer); // dta is total string sent to ABRP API including api-key and user-token (could be sensitive data to log)

    // Code for sending https data to ABRP api server
    rc = 0;
    if (liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI && liveData->settings.wifiEnabled == 1)
    {
      // Track ABRP attempt time only when payload is valid and we're about to do the actual HTTP request.
      liveData->params.lastAbrpSent = liveData->params.currentTime;
      lastAbrpSendAtMs = millis();

      WiFiClientSecure client;
      HTTPClient http;

      // Deliberately unvalidated: ABRP runs continuously during driving while BLE is
      // active, and loading a CA chain raises the TLS handshake's internal-heap use —
      // the exact pressure behind the contribute TLS-memory failures (issue #123) on
      // Core2. The leak risk here is only the ABRP token (telemetry), not RCE; the
      // RCE-relevant path (OTA) IS validated. Root CA is bundled in evdash_certs.h
      // (AMAZON_ROOT_CA_1) for anyone who wants to opt in on a PSRAM-roomy build.
      client.setInsecure();
      http.begin(client, "https://api.iternio.com/1/tlm/send");
      http.setConnectTimeout(kAbrpHttpsConnectTimeoutMs);
      http.setTimeout(kAbrpHttpsIoTimeoutMs);
      http.setReuse(false);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      const size_t bodyLength = static_cast<size_t>(dtaLength);
      addWifiTransferredBytes(bodyLength);
      syslog->info(DEBUG_ABRP, "ABRP POST body length: " + String(bodyLength));
      rc = http.POST((uint8_t *)gAbrpFormBuffer, bodyLength);
      syslog->info(DEBUG_ABRP, "ABRP HTTP status: " + String(rc));

      if (rc == HTTP_CODE_OK)
      {
        // Request successful
        String payload = http.getString();
        syslog->info(DEBUG_ABRP, "ABRP HTTP response body: " + payload);
      }
      else
      {
        // Handle different HTTP status codes
        syslog->info(DEBUG_ABRP, "HTTP Request failed with code: " + String(rc));
        if (rc > 0)
        {
          String payload = http.getString();
          syslog->info(DEBUG_ABRP, "ABRP HTTP error body: " + payload);
        }
      }

      http.end();
      client.stop();
    }

    if (rc == 200)
    {
      syslog->info(DEBUG_ABRP, "HTTP POST send successful");
      liveData->params.lastSuccessNetSendTime = liveData->params.currentTime;
      updateNetAvailability(true);
    }
    else
    {
      // Failed...
      syslog->infoNolf(DEBUG_ABRP, "HTTP POST error: ");
      syslog->info(DEBUG_ABRP, rc);
      updateNetAvailability(false);
    }
  }
  else
  {
    syslog->info(DEBUG_NET, "Well... This not gonna happen... (Board320_240::netSendData();)"); // Just for debug reasons...
  }
  // next three rows are for time measurement of this function
  int64_t endTime2 = esp_timer_get_time();
  int64_t duration2 = endTime2 - startTime2;
  // syslog->println("Time taken by function: netSendData() " + String(duration2) + " microseconds");

  return true;
}

void Board320_240::queueAbrpSdLog(const char *payload, size_t length, time_t currentTime, uint64_t operationTimeSec, bool timeSyncWithGps)
{
  if (payload == nullptr || length == 0)
  {
    return;
  }
  if (liveData->params.stopCommandQueue)
  {
    return;
  }
  if (!liveData->params.sdcardInit || !liveData->params.sdcardRecording)
  {
    return;
  }

  struct tm now;
  time_t logTime = currentTime;
  localtime_r(&logTime, &now);

  if (operationTimeSec > 0 && strlen(liveData->params.sdcardAbrpFilename) == 0)
  {
    sprintf(liveData->params.sdcardAbrpFilename, "/%llu.abrp.json", operationTimeSec / 60);
  }
  if (timeSyncWithGps && strlen(liveData->params.sdcardAbrpFilename) < 20)
  {
    strftime(liveData->params.sdcardAbrpFilename, sizeof(liveData->params.sdcardAbrpFilename), "/%y%m%d%H%M.abrp.json", &now);
  }

  if (strlen(liveData->params.sdcardAbrpFilename) == 0)
  {
    return;
  }

  File file = SD.open(liveData->params.sdcardAbrpFilename, FILE_APPEND);
  if (!file)
  {
    syslog->info(DEBUG_SDCARD, "Failed to open ABRP file for appending");
    file = SD.open(liveData->params.sdcardAbrpFilename, FILE_WRITE);
  }
  if (!file)
  {
    syslog->info(DEBUG_SDCARD, "Failed to create ABRP file");
    return;
  }

  const size_t writeLen = file.write((const uint8_t *)payload, length);
  if (writeLen != length)
  {
    syslog->info(DEBUG_SDCARD, "ABRP SD write truncated");
  }
  file.print(",\n");
  file.close();
}

/**
 * Contributes usage data if enabled in settings.
 * Sends data via WiFi to https://api.evdash.eu/v1/contribute.
 * Data includes vehicle info, location, temps, voltages, etc.
 * Receives a contribute token if successful.
 * Only called for M5Stack Core2 boards.
 **/
bool Board320_240::netContributeData()
{
  int rc = 0;

// Only for core2
#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)
  // Contribute data (api.evdash.eu/v1/contribute) to project author (nick.n17@gmail.com)
  if (liveData->settings.wifiEnabled == 1 && WiFi.status() == WL_CONNECTED &&
      liveData->params.contributeStatus == CONTRIBUTE_READY_TO_SEND)
  {
    syslog->info(DEBUG_NET, "Contribute data...");
    if (isMobileRelayClientConnected())
    {
      syslog->info(DEBUG_NET, "Contribute upload: mobile relay stays active");
    }
    const char *contributeHost = "api.evdash.eu";
    const char *contributeUrl = "https://api.evdash.eu/v1/contribute";
    const char *contributePath = "/v1/contribute";
    const String contributeUserAgent = String("evDash/") + String(APP_VERSION);
    char *payloadForPost = nullptr;
    size_t payloadForPostLen = 0;
    bool payloadForPostInPsram = false;
    auto scheduleNextContributeCycle = [&]()
    {
      liveData->params.contributeStatus = CONTRIBUTE_NONE;
      contributeStatusSinceMs = 0;
      nextContributeCycleAtMs = millis() + kContributeCycleIntervalMs;
    };
    {
      String payloadJson;
      payloadJson.reserve(4096);
      if (!buildContributePayloadV2(payloadJson, false))
      {
        syslog->info(DEBUG_NET, "Failed to build contribute v2 payload");
        scheduleNextContributeCycle();
        updateNetAvailability(false);
        return false;
      }
      if (isContributeV2SnapshotEffectivelyEmpty(liveData))
      {
        syslog->info(DEBUG_NET, "Contribute v2 empty snapshot, skipping send");
        scheduleNextContributeCycle();
        return false;
      }

      if (payloadJson.length() < 2 || payloadJson.charAt(0) != '{' || payloadJson.charAt(payloadJson.length() - 1) != '}')
      {
        syslog->info(DEBUG_NET, "Contribute payload invalid, skipping send");
        scheduleNextContributeCycle();
        updateNetAvailability(false);
        return false;
      }
      payloadForPostLen = payloadJson.length();
      payloadForPost = allocContributePayloadBuffer(payloadForPostLen, payloadForPostInPsram);
      if (payloadForPost == nullptr)
      {
        syslog->info(DEBUG_NET, "Contribute payload buffer allocation failed");
        scheduleNextContributeCycle();
        updateNetAvailability(false);
        return false;
      }
      memcpy(payloadForPost, payloadJson.c_str(), payloadForPostLen + 1);
    }
    syslog->infoNolf(DEBUG_NET, "Contribute payload bytes: ");
    syslog->info(DEBUG_NET, payloadForPostLen);
    syslog->infoNolf(DEBUG_NET, "Contribute payload buffer: ");
    syslog->info(DEBUG_NET, payloadForPostInPsram ? "psram" : "internal");

    auto printContributeHeap = [&]()
    {
      if (!syslog->isDebug(DEBUG_NET))
      {
        return;
      }
      syslog->infoNolf(DEBUG_NET, "Heap intFree/intLargest/psram: ");
      syslog->info(DEBUG_NET, String(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) + " / " +
                              String(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) + " / " +
                              String(ESP.getFreePsram()));
    };
    syslog->info(DEBUG_NET, "Contribute TLS: BLE stays active");
    printContributeHeap();

    String responsePayload = "";
    int lastTlsErrCode = 0;
    String lastTlsErrText = "";
    auto postContributePayload = [&](String &outResponse, uint8_t attemptNo) -> int
    {
      const uint32_t startedMs = millis();
      lastTlsErrCode = 0;
      lastTlsErrText = "";
      WiFiClientSecure client;
      HTTPClient http;
      // Deliberately unvalidated — this is the memory-critical contribute path that
      // already fails with TLS BIGNUM-alloc errors under BLE+WiFi on Core2 (issue
      // #123); adding CA-chain verification raises that pressure. Leaked data is the
      // contribute token + telemetry, not RCE. See evdash_certs.h / ABRP note above.
      client.setInsecure();
      client.setHandshakeTimeout((kContributeHttpsConnectTimeoutMs + 999) / 1000);
      client.setTimeout((kContributeHttpsIoTimeoutMs + 999) / 1000);
      const bool beginOk = http.begin(client, contributeUrl);
      if (!beginOk)
      {
        syslog->infoNolf(DEBUG_NET, "Contribute POST attempt ");
        syslog->infoNolf(DEBUG_NET, attemptNo);
        syslog->info(DEBUG_NET, ": http.begin failed");
        outResponse = "";
        return -1;
      }
      http.setConnectTimeout(kContributeHttpsConnectTimeoutMs);
      http.setTimeout(kContributeHttpsIoTimeoutMs);
      http.useHTTP10(true);
      http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
      http.addHeader("Content-Type", "application/json");
      http.addHeader("User-Agent", contributeUserAgent);
      http.addHeader("Accept", "application/json");
      http.addHeader("Connection", "close");
      addWifiTransferredBytes(payloadForPostLen);
      const int postRc = http.POST((uint8_t *)payloadForPost, payloadForPostLen);
      const uint32_t elapsedMs = millis() - startedMs;
      syslog->infoNolf(DEBUG_NET, "Contribute POST attempt ");
      syslog->infoNolf(DEBUG_NET, attemptNo);
      syslog->infoNolf(DEBUG_NET, " rc=");
      syslog->infoNolf(DEBUG_NET, postRc);
      syslog->infoNolf(DEBUG_NET, " (");
      syslog->infoNolf(DEBUG_NET, elapsedMs);
      syslog->info(DEBUG_NET, "ms)");
      outResponse = "";
      if (postRc > 0)
      {
        outResponse = http.getString();
      }
      else
      {
        char tlsErrBuf[160] = {0};
        lastTlsErrCode = client.lastError(tlsErrBuf, sizeof(tlsErrBuf));
        lastTlsErrText = String(tlsErrBuf);
        syslog->infoNolf(DEBUG_NET, "Contribute TLS lastError: ");
        syslog->infoNolf(DEBUG_NET, lastTlsErrCode);
        syslog->infoNolf(DEBUG_NET, " ");
        syslog->info(DEBUG_NET, tlsErrBuf);
        printContributeHeap();
      }
      http.end();
      client.stop();
      return postRc;
    };

    auto postContributePayloadRawTls = [&](String &outResponse, const IPAddress &ip, uint8_t attemptNo) -> int
    {
      const uint32_t startedMs = millis();
      outResponse = "";
      lastTlsErrCode = 0;
      lastTlsErrText = "";

      WiFiClientSecure client;
      client.setInsecure();
      client.setHandshakeTimeout((kContributeHttpsConnectTimeoutMs + 999) / 1000);
      client.setTimeout((kContributeHttpsIoTimeoutMs + 999) / 1000);

      if (!client.connect(ip, 443, contributeHost, nullptr, nullptr, nullptr))
      {
        char tlsErrBuf[160] = {0};
        lastTlsErrCode = client.lastError(tlsErrBuf, sizeof(tlsErrBuf));
        lastTlsErrText = String(tlsErrBuf);
        const uint32_t elapsedMs = millis() - startedMs;
        syslog->infoNolf(DEBUG_NET, "Contribute RAW TLS attempt ");
        syslog->infoNolf(DEBUG_NET, attemptNo);
        syslog->infoNolf(DEBUG_NET, " connect failed (");
        syslog->infoNolf(DEBUG_NET, elapsedMs);
        syslog->info(DEBUG_NET, "ms)");
        syslog->infoNolf(DEBUG_NET, "Contribute TLS lastError: ");
        syslog->infoNolf(DEBUG_NET, lastTlsErrCode);
        syslog->infoNolf(DEBUG_NET, " ");
        syslog->info(DEBUG_NET, tlsErrBuf);
        client.stop();
        return -1;
      }

      String headers = String("POST ") + contributePath + " HTTP/1.0\r\n" +
                       "Host: " + contributeHost + "\r\n" +
                       "Content-Type: application/json\r\n" +
                       "User-Agent: " + contributeUserAgent + "\r\n" +
                       "Accept: application/json\r\n" +
                       "Connection: close\r\n" +
                       "Content-Length: " + String(payloadForPostLen) + "\r\n\r\n";

      addWifiTransferredBytes(headers.length() + payloadForPostLen);
      client.print(headers);
      const size_t written = client.write((const uint8_t *)payloadForPost, payloadForPostLen);
      if (written != payloadForPostLen)
      {
        syslog->infoNolf(DEBUG_NET, "Contribute RAW TLS payload short write: ");
        syslog->infoNolf(DEBUG_NET, written);
        syslog->infoNolf(DEBUG_NET, "/");
        syslog->info(DEBUG_NET, payloadForPostLen);
      }

      String rawResponse = "";
      const uint32_t readTimeoutMs = kContributeHttpsIoTimeoutMs;
      uint32_t lastDataMs = millis();
      while ((millis() - lastDataMs) < readTimeoutMs)
      {
        while (client.available())
        {
          const char nextChar = static_cast<char>(client.read());
          if (rawResponse.length() < kContributeResponseBufferCap)
          {
            rawResponse += nextChar;
          }
          lastDataMs = millis();
        }
        if (!client.connected())
        {
          break;
        }
        delay(2);
      }

      int statusCode = -1;
      const int lineEnd = rawResponse.indexOf("\r\n");
      if (lineEnd > 0)
      {
        const String statusLine = rawResponse.substring(0, lineEnd);
        const int sp1 = statusLine.indexOf(' ');
        if (sp1 > 0)
        {
          const int sp2 = statusLine.indexOf(' ', sp1 + 1);
          if (sp2 > sp1)
          {
            statusCode = statusLine.substring(sp1 + 1, sp2).toInt();
          }
          else
          {
            statusCode = statusLine.substring(sp1 + 1).toInt();
          }
        }
      }

      const int bodyPos = rawResponse.indexOf("\r\n\r\n");
      if (bodyPos >= 0)
      {
        outResponse = rawResponse.substring(bodyPos + 4);
      }
      else
      {
        outResponse = rawResponse;
      }

      const uint32_t elapsedMs = millis() - startedMs;
      syslog->infoNolf(DEBUG_NET, "Contribute RAW TLS attempt ");
      syslog->infoNolf(DEBUG_NET, attemptNo);
      syslog->infoNolf(DEBUG_NET, " rc=");
      syslog->infoNolf(DEBUG_NET, statusCode);
      syslog->infoNolf(DEBUG_NET, " (");
      syslog->infoNolf(DEBUG_NET, elapsedMs);
      syslog->info(DEBUG_NET, "ms)");

      client.stop();
      return statusCode;
    };

    auto postContributePayloadHttp = [&](String &outResponse, uint8_t attemptNo) -> int
    {
      const uint32_t startedMs = millis();
      outResponse = "";

      WiFiClient client;
      client.setTimeout((kContributeHttpReadTimeoutMs + 999) / 1000);

      if (!client.connect(contributeHost, 80, kContributeHttpConnectTimeoutMs))
      {
        const uint32_t elapsedMs = millis() - startedMs;
        syslog->infoNolf(DEBUG_NET, "Contribute HTTP fallback attempt ");
        syslog->infoNolf(DEBUG_NET, attemptNo);
        syslog->infoNolf(DEBUG_NET, " connect failed (");
        syslog->infoNolf(DEBUG_NET, elapsedMs);
        syslog->info(DEBUG_NET, "ms)");
        client.stop();
        return -1;
      }

      String headers = String("POST ") + contributePath + " HTTP/1.0\r\n" +
                       "Host: " + contributeHost + "\r\n" +
                       "Content-Type: application/json\r\n" +
                       "User-Agent: " + contributeUserAgent + "\r\n" +
                       "Accept: application/json\r\n" +
                       "Connection: close\r\n" +
                       "Content-Length: " + String(payloadForPostLen) + "\r\n\r\n";

      addWifiTransferredBytes(headers.length() + payloadForPostLen);
      client.print(headers);
      const size_t written = client.write((const uint8_t *)payloadForPost, payloadForPostLen);
      if (written != payloadForPostLen)
      {
        syslog->infoNolf(DEBUG_NET, "Contribute HTTP fallback payload short write: ");
        syslog->infoNolf(DEBUG_NET, written);
        syslog->infoNolf(DEBUG_NET, "/");
        syslog->info(DEBUG_NET, payloadForPostLen);
      }

      String rawResponse = "";
      const uint32_t readTimeoutMs = kContributeHttpReadTimeoutMs;
      uint32_t lastDataMs = millis();
      while ((millis() - lastDataMs) < readTimeoutMs)
      {
        while (client.available())
        {
          const char nextChar = static_cast<char>(client.read());
          if (rawResponse.length() < kContributeResponseBufferCap)
          {
            rawResponse += nextChar;
          }
          lastDataMs = millis();
        }
        if (!client.connected())
        {
          break;
        }
        delay(2);
      }

      int statusCode = -1;
      const int lineEnd = rawResponse.indexOf("\r\n");
      if (lineEnd > 0)
      {
        const String statusLine = rawResponse.substring(0, lineEnd);
        const int sp1 = statusLine.indexOf(' ');
        if (sp1 > 0)
        {
          const int sp2 = statusLine.indexOf(' ', sp1 + 1);
          if (sp2 > sp1)
          {
            statusCode = statusLine.substring(sp1 + 1, sp2).toInt();
          }
          else
          {
            statusCode = statusLine.substring(sp1 + 1).toInt();
          }
        }
      }

      const int bodyPos = rawResponse.indexOf("\r\n\r\n");
      if (bodyPos >= 0)
      {
        outResponse = rawResponse.substring(bodyPos + 4);
      }
      else
      {
        outResponse = rawResponse;
      }

      const uint32_t elapsedMs = millis() - startedMs;
      syslog->infoNolf(DEBUG_NET, "Contribute HTTP fallback attempt ");
      syslog->infoNolf(DEBUG_NET, attemptNo);
      syslog->infoNolf(DEBUG_NET, " rc=");
      syslog->infoNolf(DEBUG_NET, statusCode);
      syslog->infoNolf(DEBUG_NET, " (");
      syslog->infoNolf(DEBUG_NET, elapsedMs);
      syslog->info(DEBUG_NET, "ms)");

      client.stop();
      return statusCode;
    };

    IPAddress resolvedHost;
    int dnsRc = WiFi.hostByName(contributeHost, resolvedHost);
    syslog->infoNolf(DEBUG_NET, "Contribute DNS ");
    syslog->infoNolf(DEBUG_NET, contributeHost);
    syslog->infoNolf(DEBUG_NET, ": ");
    if (dnsRc == 1)
    {
      syslog->info(DEBUG_NET, resolvedHost.toString());
    }
    else
    {
      syslog->info(DEBUG_NET, "resolve_failed");
    }

    bool usedRawTlsFirst = false;
    if (dnsRc == 1)
    {
      usedRawTlsFirst = true;
      syslog->info(DEBUG_NET, "Contribute HTTPS: raw TLS POST with SNI...");
      rc = postContributePayloadRawTls(responsePayload, resolvedHost, 1);
    }
    else
    {
      rc = postContributePayload(responsePayload, 1);
    }

    bool tlsMemIssue = isTlsMemoryIssue(lastTlsErrCode, lastTlsErrText);
    if (rc < 0)
    {
      if (syslog->isDebug(DEBUG_NET))
      {
        syslog->info(DEBUG_NET, String("WiFi RSSI/ch/BSSID: ") + String(WiFi.RSSI()) + " / " + String(WiFi.channel()) + " / " + WiFi.BSSIDstr());
      }
      if (dnsRc == 1 && kContributeEnableTcpProbe)
      {
        WiFiClient tcpProbe;
        const uint32_t tcpStartedMs = millis();
        const int tcpRc = tcpProbe.connect(resolvedHost, 443, 2000);
        const uint32_t tcpElapsedMs = millis() - tcpStartedMs;
        syslog->infoNolf(DEBUG_NET, "Contribute TCP probe ");
        syslog->infoNolf(DEBUG_NET, resolvedHost.toString());
        syslog->infoNolf(DEBUG_NET, ":443 rc=");
        syslog->infoNolf(DEBUG_NET, tcpRc);
        syslog->infoNolf(DEBUG_NET, " (");
        syslog->infoNolf(DEBUG_NET, tcpElapsedMs);
        syslog->info(DEBUG_NET, "ms)");
        tcpProbe.stop();
      }
      if (!tlsMemIssue)
      {
        if (kContributeRetryOnceOnFail)
        {
          syslog->info(DEBUG_NET, "Retry contribute POST once...");
          delay(250);
          if (dnsRc == 1)
          {
            rc = postContributePayloadRawTls(responsePayload, resolvedHost, 2);
          }
          else
          {
            rc = postContributePayload(responsePayload, 2);
          }
          tlsMemIssue = isTlsMemoryIssue(lastTlsErrCode, lastTlsErrText);
        }
        else
        {
          syslog->info(DEBUG_NET, "Contribute HTTPS retry disabled (stability mode)");
        }
      }
      else
      {
        syslog->info(DEBUG_NET, "Contribute HTTPS retry skipped (TLS memory issue on attempt 1)");
      }
    }

    if (kContributeRawTlsFallbackOnTlsMem && rc < 0 && dnsRc == 1 && !usedRawTlsFirst)
    {
      syslog->info(DEBUG_NET, "Contribute HTTPS fallback: raw TLS POST with SNI...");
      rc = postContributePayloadRawTls(responsePayload, resolvedHost, 3);
      tlsMemIssue = isTlsMemoryIssue(lastTlsErrCode, lastTlsErrText);
    }

    if (kContributeHttpFallbackOnTlsMem && rc < 0 && tlsMemIssue)
    {
      syslog->info(DEBUG_NET, "Contribute TLS memory workaround: plain HTTP fallback...");
      rc = postContributePayloadHttp(responsePayload, 4);
      if (rc > 0)
      {
        tlsMemIssue = false;
      }
    }

    if (rc < 0 && tlsMemIssue)
    {
      syslog->info(DEBUG_NET, "Contribute HTTPS TLS memory issue detected in low-memory mode.");
    }

    if (rc == HTTP_CODE_OK)
    {
      bool responseAccepted = true;
      syslog->infoNolf(DEBUG_NET, "HTTP Response (");
      syslog->infoNolf(DEBUG_NET, contributeHost);
      syslog->infoNolf(DEBUG_NET, "): ");
      syslog->info(DEBUG_NET, responsePayload);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, responsePayload);
      if (!error)
      {
        const char *status = doc["status"];
        if (status != nullptr && strcmp(status, "ok") != 0)
        {
          responseAccepted = false;
          syslog->infoNolf(DEBUG_NET, "Contribute rejected by server: ");
          syslog->info(DEBUG_NET, status);
        }

        const char *token = doc["token"];
        if (token != nullptr && strlen(token) > 10 &&
            strcmp(liveData->settings.contributeToken, token) != 0)
        {
          syslog->infoNolf(DEBUG_NET, "Assigned token: ");
          syslog->info(DEBUG_NET, token);
          strncpy(liveData->settings.contributeToken, token, sizeof(liveData->settings.contributeToken) - 1);
          liveData->settings.contributeToken[sizeof(liveData->settings.contributeToken) - 1] = '\0';
          saveSettings();
        }
      }
      else
      {
        // Keep upload as successful even when payload is non-JSON due proxy/WAF text.
        syslog->infoNolf(DEBUG_NET, "Contribute response parse error: ");
        syslog->info(DEBUG_NET, error.c_str());
      }

      if (responseAccepted)
      {
        scheduleNextContributeCycle();
        liveData->params.lastSuccessNetSendTime = liveData->params.currentTime;
        updateNetAvailability(true);
      }
      else
      {
        scheduleNextContributeCycle();
        updateNetAvailability(false);
      }
    }
    else
    {
      // Failed...
      if (rc > 0)
      {
        syslog->infoNolf(DEBUG_NET, "HTTP POST status: ");
        syslog->info(DEBUG_NET, rc);
        if (responsePayload.length() > 0 && syslog->isDebug(DEBUG_NET))
        {
          String responsePreview = responsePayload;
          responsePreview.replace('\r', ' ');
          responsePreview.replace('\n', ' ');
          if (responsePreview.length() > 180)
          {
            responsePreview = responsePreview.substring(0, 180) + "...";
          }
          syslog->infoNolf(DEBUG_NET, "HTTP Response preview (");
          syslog->infoNolf(DEBUG_NET, contributeHost);
          syslog->infoNolf(DEBUG_NET, "): ");
          syslog->info(DEBUG_NET, responsePreview);
        }
      }
      else
      {
        syslog->infoNolf(DEBUG_NET, "HTTP POST error: ");
        syslog->info(DEBUG_NET, rc);
        syslog->infoNolf(DEBUG_NET, "HTTP POST error text: ");
        syslog->info(DEBUG_NET, HTTPClient::errorToString(rc).c_str());
      }
      if (syslog->isDebug(DEBUG_NET))
      {
        syslog->infoNolf(DEBUG_NET, "WiFi status/IP/GW/DNS: ");
        syslog->info(DEBUG_NET, String(WiFi.status()) + " / " +
                        WiFi.localIP().toString() + " / " +
                        WiFi.gatewayIP().toString() + " / " +
                        WiFi.dnsIP(0).toString());
      }
      scheduleNextContributeCycle();
      updateNetAvailability(false);
    }
    free(payloadForPost);
  }
#endif // BOARD_M5STACK_CORE2 || BOARD_M5STACK_CORES3

  return true;
}

void Board320_240::runSdV2BackgroundTasks(bool netReady)
{
  const uint32_t nowMs = millis();

  if (nextSdV2CleanupAtMs == 0 || static_cast<int32_t>(nowMs - nextSdV2CleanupAtMs) >= 0)
  {
    cleanupUploadedSdV2Logs();
    nextSdV2CleanupAtMs = nowMs + kSdV2CleanupIntervalMs;
  }

  if (!liveData->params.sdcardInit)
  {
    resetSdV2UploadState();
    sdV2BackgroundStartAtMs = 0;
    nextSdV2BackgroundUploadAtMs = 0;
    return;
  }
  const bool uploadEligible =
      (netReady &&
       liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI &&
       liveData->settings.wifiEnabled == 1 &&
       !liveData->params.wifiApMode);
  if (!uploadEligible)
  {
    sdV2BackgroundStartAtMs = 0;
    nextSdV2BackgroundUploadAtMs = 0;
    return;
  }

  if (sdV2BackgroundStartAtMs == 0)
  {
    sdV2BackgroundStartAtMs = nowMs + kSdV2BackgroundStartDelayMs;
    nextSdV2BackgroundUploadAtMs = 0;
    return;
  }
  if (static_cast<int32_t>(nowMs - sdV2BackgroundStartAtMs) < 0)
  {
    return;
  }

  if (nextSdV2BackgroundUploadAtMs == 0)
  {
    nextSdV2BackgroundUploadAtMs = nowMs;
  }
  if (static_cast<int32_t>(nowMs - nextSdV2BackgroundUploadAtMs) < 0)
  {
    return;
  }
  nextSdV2BackgroundUploadAtMs = nowMs + kSdV2BackgroundUploadIntervalMs;

  const String activeLogFilename = toAbsoluteSdPath(String(liveData->params.sdcardFilename));
  if (!ensureSdV2UploadFileSelected(activeLogFilename))
  {
    return;
  }

  if (!processSdV2UploadChunk())
  {
    nextSdV2BackgroundUploadAtMs = nowMs + kSdV2BackgroundRetryBackoffMs;
  }
}

bool Board320_240::ensureSdV2UploadFileSelected(const String &activeLogFilename)
{
  if (sdV2UploadFilePath.length() > 0 && sdV2UploadFileName.length() > 0)
  {
    return true;
  }

  File dir = SD.open("/");
  if (!dir || !dir.isDirectory())
  {
    if (dir)
    {
      dir.close();
    }
    return false;
  }

  String selectedPath = "";
  String selectedName = "";
  while (true)
  {
    File entry = dir.openNextFile(FILE_READ);
    if (!entry)
    {
      break;
    }

    if (!entry.isDirectory())
    {
      const String filePath = toAbsoluteSdPath(String(entry.name()));
      const bool isActiveLog = (activeLogFilename.length() > 0 && filePath == activeLogFilename);
      if (isPendingSdV2LogFile(filePath) && !isActiveLog)
      {
        if (selectedPath.length() == 0 || filePath < selectedPath)
        {
          selectedPath = filePath;
          selectedName = (filePath.charAt(0) == '/') ? filePath.substring(1) : filePath;
        }
      }
    }
    entry.close();
  }
  dir.close();

  if (selectedPath.length() == 0)
  {
    return false;
  }

  sdV2UploadFilePath = selectedPath;
  sdV2UploadFileName = selectedName;
  sdV2UploadPart = 0;
  sdV2UploadOffset = 0;
  return true;
}

bool Board320_240::processSdV2UploadChunk()
{
  if (sdV2UploadFilePath.length() == 0 || sdV2UploadFileName.length() == 0)
  {
    return false;
  }

  auto finalizeUploadedFile = [&]() -> bool
  {
    const String uploadedPath = toUploadedSdV2Path(sdV2UploadFilePath);
    bool renamed = false;
    if (uploadedPath.length() > 0)
    {
      if (SD.exists(uploadedPath.c_str()))
      {
        SD.remove(uploadedPath.c_str());
      }
      renamed = SD.rename(sdV2UploadFilePath.c_str(), uploadedPath.c_str());
    }
    resetSdV2UploadState();
    return renamed;
  };

  File file = SD.open(sdV2UploadFilePath.c_str(), FILE_READ);
  if (!file || file.isDirectory())
  {
    if (file)
    {
      file.close();
    }
    resetSdV2UploadState();
    return false;
  }

  const size_t fileSize = static_cast<size_t>(file.size());
  if (sdV2UploadOffset > fileSize)
  {
    file.close();
    resetSdV2UploadState();
    return false;
  }
  if (sdV2UploadOffset == fileSize)
  {
    file.close();
    return finalizeUploadedFile();
  }

  if (!file.seek(sdV2UploadOffset))
  {
    file.close();
    resetSdV2UploadState();
    return false;
  }

  const size_t readBytes = file.read(gSdLogUploadBuffer, kSdLogUploadChunkSize);
  file.close();
  if (readBytes == 0)
  {
    resetSdV2UploadState();
    return false;
  }

  if (!postSdLogChunkToEvDash(sdV2UploadFileName, sdV2UploadPart, gSdLogUploadBuffer, readBytes))
  {
    return false;
  }

  sdV2UploadOffset += static_cast<uint32_t>(readBytes);
  sdV2UploadPart++;
  if (sdV2UploadOffset >= fileSize)
  {
    return finalizeUploadedFile();
  }
  return true;
}

void Board320_240::resetSdV2UploadState()
{
  sdV2UploadFilePath = "";
  sdV2UploadFileName = "";
  sdV2UploadPart = 0;
  sdV2UploadOffset = 0;
}

bool Board320_240::postSdLogChunkToEvDash(const String &fileName, uint32_t part, const uint8_t *data, size_t length, String *responsePayload, int *responseCode, bool preferManualTimeouts)
{
  if (responsePayload != nullptr)
  {
    *responsePayload = "";
  }
  if (responseCode != nullptr)
  {
    *responseCode = -1;
  }

  if (fileName.length() == 0 || data == nullptr || length == 0)
  {
    return false;
  }
  const bool debugLog = (responseCode != nullptr && ((liveData->settings.debugLevel & DEBUG_SDCARD) != 0));

  if (WiFi.status() != WL_CONNECTED)
  {
    if (debugLog)
    {
      syslog->println("Log upload: WiFi not connected");
    }
    return false;
  }

  const String contributeKey = ensureContributeKey();
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());
  const String registerApiKey = String(liveData->settings.remoteApiKey);

  const String query = "?token=" + contributeKey +
                       "&key=" + contributeKey +
                       "&deviceKey=" + contributeKey +
                       "&hwDeviceId=" + hardwareDeviceId +
                       "&deviceId=" + hardwareDeviceId +
                       "&dev=" + String(getCompiledDeviceTypeForApi()) +
                       "&apiKey=" + registerApiKey +
                       "&register=1" +
                       "&filename=" + fileName +
                       "&part=" + String(part);

  int rc = -1;
  String payload = "";
  WiFiClientSecure client;
  HTTPClient http;
  const uint16_t connectTimeoutMs = (preferManualTimeouts ? kSdLogUploadManualConnectTimeoutMs : kSdLogUploadConnectTimeoutMs);
  const uint16_t ioTimeoutMs = (preferManualTimeouts ? kSdLogUploadManualIoTimeoutMs : kSdLogUploadIoTimeoutMs);
  client.setInsecure();
  client.setHandshakeTimeout((connectTimeoutMs + 999) / 1000);
  client.setTimeout((ioTimeoutMs + 999) / 1000);

  const String url = String(kSdLogUploadBaseUrl) + query;
  if (debugLog)
  {
    syslog->print("Log upload try: ");
    syslog->println(url);
  }
  if (!http.begin(client, url))
  {
    client.stop();
    rc = -1;
    if (debugLog)
    {
      syslog->println("Log upload: http.begin failed");
    }
  }
  else
  {
    http.setConnectTimeout(connectTimeoutMs);
    http.setTimeout(ioTimeoutMs);
    http.useHTTP10(true);
    http.setReuse(false);
    addWifiTransferredBytes(length);
    rc = http.POST((uint8_t *)data, length);
    payload = "";
    if (rc == HTTP_CODE_OK)
    {
      payload = http.getString();
    }
    else if (debugLog)
    {
      syslog->print("Log upload post rc=");
      syslog->print(rc);
      syslog->print(" err=");
      syslog->println(HTTPClient::errorToString(rc).c_str());
    }
    http.end();
    client.stop();
  }

  if (responsePayload != nullptr)
  {
    *responsePayload = payload;
  }
  if (responseCode != nullptr)
  {
    *responseCode = rc;
  }

  if (rc != HTTP_CODE_OK)
  {
    if (debugLog)
    {
      IPAddress resolved;
      const int dnsRc = WiFi.hostByName("api.evdash.eu", resolved);
      syslog->print("Log upload DNS api.evdash.eu: ");
      if (dnsRc == 1)
      {
        syslog->println(resolved.toString());
      }
      else
      {
        syslog->println("resolve_failed");
      }
      syslog->print("WiFi status/IP/GW/DNS: ");
      syslog->println(String(WiFi.status()) + " / " +
                      WiFi.localIP().toString() + " / " +
                      WiFi.gatewayIP().toString() + " / " +
                      WiFi.dnsIP(0).toString());
      if (dnsRc == 1)
      {
        WiFiClient tcpProbe;
        const int tcpRc = tcpProbe.connect(resolved, 443, 2500);
        syslog->print("Log upload TCP probe ");
        syslog->print(resolved.toString());
        syslog->print(":443 rc=");
        syslog->println(tcpRc);
        tcpProbe.stop();
      }
    }
    return false;
  }

  const bool statusOk = (payload.indexOf("\"status\":\"ok\"") != -1);
  const bool storedFalse = (payload.indexOf("\"stored\":false") != -1);
  return (statusOk && !storedFalse);
}

bool Board320_240::cleanupUploadedSdV2Logs()
{
  if (!liveData->params.sdcardInit)
  {
    return false;
  }

  const time_t nowTime = liveData->params.currentTime;
  const bool hasReliableWallClock =
      (liveData->params.currTimeSyncWithGps ||
       liveData->params.ntpTimeSet);
  if (!hasReliableWallClock || nowTime <= 0)
  {
    return false;
  }

  struct tm nowTm = {};
  if (localtime_r(&nowTime, &nowTm) == nullptr)
  {
    return false;
  }
  const int nowYear = nowTm.tm_year + 1900;
  if (nowYear < 2025 || nowYear > 2040)
  {
    return false;
  }

  File dir = SD.open("/");
  if (!dir || !dir.isDirectory())
  {
    if (dir)
    {
      dir.close();
    }
    return false;
  }

  bool removedAny = false;
  while (true)
  {
    File entry = dir.openNextFile(FILE_READ);
    if (!entry)
    {
      break;
    }

    if (entry.isDirectory())
    {
      entry.close();
      continue;
    }

    const String filePath = toAbsoluteSdPath(String(entry.name()));
    if (!isUploadedSdV2LogFile(filePath))
    {
      entry.close();
      continue;
    }

    time_t fileTime = entry.getLastWrite();
    if (fileTime <= 0 || fileTime > nowTime)
    {
      time_t parsedTime = 0;
      if (parseSdLogYyMmDdHhMm(filePath, parsedTime))
      {
        fileTime = parsedTime;
      }
    }
    entry.close();

    if (fileTime <= 0 || nowTime <= fileTime)
    {
      continue;
    }
    if ((nowTime - fileTime) < kSdV2UploadedRetentionSec)
    {
      continue;
    }

    if (SD.remove(filePath.c_str()))
    {
      removedAny = true;
    }
  }
  dir.close();
  return removedAny;
}

/**
 * Initializes the hardware GPS module.
 * Configures the hardware serial port and baud rate.
 * Sends configuration commands to the GPS module to enable desired sentences,
 * set update rate, enable SBAS, and configure navigation model.
 */
void Board320_240::initGPS()
{
  syslog->print("GPS initialization on hwUart: ");
  syslog->println(liveData->settings.gpsHwSerialPort);

  gpsHwUart = new HardwareSerial(liveData->settings.gpsHwSerialPort);
  auto beginGpsUart = [&](unsigned long baud)
  {
    if (liveData->settings.gpsHwSerialPort == 1 || liveData->settings.gpsHwSerialPort == 2)
    {
      gpsHwUart->begin(baud, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
    }
    else
    {
      gpsHwUart->begin(baud);
    }
    delay(120);
    while (gpsHwUart->available())
    {
      gpsHwUart->read();
    }
  };

  auto hasValidNmea = [&](uint32_t timeoutMs) -> bool
  {
    TinyGPSPlus probeGps;
    const uint32_t startMs = millis();
    while (millis() - startMs < timeoutMs)
    {
      while (gpsHwUart->available())
      {
        int ch = gpsHwUart->read();
        if (ch >= 0)
        {
          probeGps.encode(static_cast<char>(ch));
        }
      }
      if (probeGps.passedChecksum() > 0)
      {
        return true;
      }
      delay(5);
    }
    return false;
  };

  auto sendNmeaCommand = [&](const char *payload)
  {
    if (payload == nullptr || *payload == '\0')
    {
      return;
    }

    uint8_t checksum = 0;
    for (const char *p = payload; *p != '\0'; p++)
    {
      checksum ^= static_cast<uint8_t>(*p);
    }

    char command[96];
    snprintf(command, sizeof(command), "$%s*%02X\r\n", payload, checksum);
    gpsHwUart->print(command);
    delay(60);
  };

  unsigned long detectedBaud = liveData->settings.gpsSerialPortSpeed;
  unsigned long baudCandidates[4] = {0, 0, 0, 0};
  uint8_t baudCandidateCount = 0;

  auto pushBaudCandidate = [&](unsigned long baud)
  {
    if (baud == 0)
    {
      return;
    }
    for (uint8_t i = 0; i < baudCandidateCount; i++)
    {
      if (baudCandidates[i] == baud)
      {
        return;
      }
    }
    if (baudCandidateCount < (sizeof(baudCandidates) / sizeof(baudCandidates[0])))
    {
      baudCandidates[baudCandidateCount++] = baud;
    }
  };

  pushBaudCandidate(liveData->settings.gpsSerialPortSpeed);
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_M5_GNSS)
  {
    pushBaudCandidate(38400);
    pushBaudCandidate(115200);
    pushBaudCandidate(9600);
  }
  else if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
  {
    pushBaudCandidate(115200);
    pushBaudCandidate(38400);
    pushBaudCandidate(9600);
  }
  else if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_NEO_M8N)
  {
    pushBaudCandidate(9600);
    pushBaudCandidate(38400);
    pushBaudCandidate(115200);
  }
  else
  {
    pushBaudCandidate(9600);
    pushBaudCandidate(38400);
    pushBaudCandidate(115200);
  }

  bool baudDetected = false;
  for (uint8_t i = 0; i < baudCandidateCount; i++)
  {
    beginGpsUart(baudCandidates[i]);
    if (hasValidNmea(1500))
    {
      detectedBaud = baudCandidates[i];
      baudDetected = true;
      break;
    }
  }

  if (!baudDetected)
  {
    detectedBaud = liveData->settings.gpsSerialPortSpeed;
    beginGpsUart(detectedBaud);
    syslog->print("GPS baud auto-detect failed, using configured speed: ");
    syslog->println(detectedBaud);
  }
  else
  {
    if (detectedBaud != liveData->settings.gpsSerialPortSpeed)
    {
      syslog->print("GPS baud auto-detected: ");
      syslog->println(detectedBaud);
      liveData->settings.gpsSerialPortSpeed = detectedBaud;
    }
    else
    {
      syslog->print("GPS baud confirmed: ");
      syslog->println(detectedBaud);
    }
  }

  // M5 GPS MODULE with int.&ext. antenna (u-blox NEO-M8N)
  // https://shop.m5stack.com/products/gps-module
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_NEO_M8N)
  {
    // Enable static hold
    // https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf
    // https://github.com/noerw/mobile-sensebox/blob/master/esp8266-gps/gps.h
    // uBlox NEO-7M can't persist settings, so we update them on runtime to get a higher update rate
    // commands extracted via u-center (https://www.youtube.com/watch?v=iWd0gCOYsdo)
    uint8_t ubloxconfig[] = {
        // disable sleep mode
        0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37,
        // enable GPGGA & RMC sentences (only these are evaluated by TinyGPS++)
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0x38, // GGA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x09, 0x54, // RMC
        // disable all other NMEA sentences to save bandwith
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GLL
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GSA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GSV
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // VTG
        // setup SBAS search to EGNOS only
        0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x01, 0x03, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x84, 0x15,
        // set NAV5 model to automotive, static hold on 0.5m/s, 3m
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
        0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x32, 0x3C, 0x00, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x78,
        // 150ms update interval
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x96, 0x00, 0x01, 0x00, 0x01, 0x00, 0xAC, 0x3E,
        // 100ms update interval (needs higher baudrate, whose config doesnt work?)
        // 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
        // uart to baud 115200 and nmea only  -> wont work?! TODO :^(
        // 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
        // 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x78,
        // save changes
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0x1D, 0xAB};

    gpsHwUart->write(ubloxconfig, sizeof(ubloxconfig));
  }
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_M5_GNSS)
  {
    uint8_t ubloxconfig[] = {
        // Disable sleep mode
        0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37,
        // Enable GPGGA & RMC sentences (only these are evaluated by TinyGPS++)
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0x38, // GGA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x09, 0x54, // RMC
        // Disable all other NMEA sentences to save bandwidth
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GLL
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GSA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GSV
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // VTG
        // Enable GPS, GLONASS, Galileo, BeiDou
        0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00, 0x00, 0x20, 0x20, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01,
        0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01,
        0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0xA4, 0x47,
        // Set NAV5 model to automotive, static hold on 0.5m/s, 3m
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
        0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x32, 0x3C, 0x00, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x78,
        // Dynamic Model: Automotive
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
        0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x32, 0x3C, 0x00, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x78,
        // Disable SBAS
        0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x00, 0x03, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x84, 0x15,
        // 100ms update interval for higher refresh rate
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12,
        // Save changes
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0x1D, 0xAB};

    gpsHwUart->write(ubloxconfig, sizeof(ubloxconfig));
  }
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
  {
    // M5 GPS Module v2.1 (ATGM336H/AT6668): use CASIC PCAS NMEA commands (not UBX).
    // Keep only GGA + RMC output and a faster 200 ms update interval.
    sendNmeaCommand("PCAS02,200");
    sendNmeaCommand("PCAS03,1,0,0,1,0,0,0,0");
    sendNmeaCommand("PCAS00");
    gpsV21PpsModeKnown = false;
    setGpsV21Pps(false);
    syslog->println("GPS v2.1 PCAS init applied (200ms, GGA+RMC).");
  }
}

/**
 * Send CASIC binary command to GPS module.
 */
void Board320_240::sendCasicGpsCommand(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t payloadLen)
{
  if (gpsHwUart == NULL || payload == NULL || payloadLen > 48 || (payloadLen % 4) != 0)
  {
    return;
  }

  uint8_t command[58];
  command[0] = 0xBA;
  command[1] = 0xCE;
  command[2] = payloadLen & 0xFF;
  command[3] = (payloadLen >> 8) & 0xFF;
  command[4] = msgClass;
  command[5] = msgId;
  for (uint16_t i = 0; i < payloadLen; i++)
  {
    command[6 + i] = payload[i];
  }

  uint32_t checksum = 0;
  for (uint16_t i = 2; i < payloadLen + 6; i += 4)
  {
    checksum += ((uint32_t)command[i]) |
                ((uint32_t)command[i + 1] << 8) |
                ((uint32_t)command[i + 2] << 16) |
                ((uint32_t)command[i + 3] << 24);
  }

  uint16_t checksumOffset = payloadLen + 6;
  command[checksumOffset] = checksum & 0xFF;
  command[checksumOffset + 1] = (checksum >> 8) & 0xFF;
  command[checksumOffset + 2] = (checksum >> 16) & 0xFF;
  command[checksumOffset + 3] = (checksum >> 24) & 0xFF;
  gpsHwUart->write(command, payloadLen + 10);
  delay(60);
}

/**
 * Enable or disable GPS v2.1 PPS output.
 */
void Board320_240::setGpsV21Pps(bool enabled)
{
  if (liveData->settings.gpsModuleType != GPS_MODULE_TYPE_GPS_V21_GNSS || gpsHwUart == NULL ||
      (gpsV21PpsModeKnown && gpsV21PpsEnabled == enabled))
  {
    return;
  }

  uint8_t payload[16] = {
      0x40, 0x42, 0x0F, 0x00, // 1s interval
      0xA0, 0x86, 0x01, 0x00, // 100ms pulse width
      (uint8_t)(enabled ? 2 : 0),
      0x00, 0x00, 0x08,
      0x00, 0x00, 0x00, 0x00};

  sendCasicGpsCommand(0x06, 0x03, payload, sizeof(payload));
  gpsV21PpsModeKnown = true;
  gpsV21PpsEnabled = enabled;
  syslog->println(enabled ? "GPS v2.1 PPS LED enabled." : "GPS v2.1 PPS LED disabled.");
}

/**
 * Sync GPS v2.1 PPS output with Sentry/suspend state.
 */
void Board320_240::updateGpsV21PpsMode()
{
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
  {
    setGpsV21Pps(liveData->params.stopCommandQueue);
  }
}

/**
 * This function uploads log files from the SD card to the EvDash server.
 */
void Board320_240::uploadSdCardLogToEvDashServer(bool silent)
{
  if (!silent)
  {
    syslog->println("uploadSdCardLogToEvDashServer");
  }
  if (!liveData->params.sdcardInit)
  {
    if (!silent)
    {
      syslog->println("SD card not initialized");
      displayMessage("SDCARD", "Not mounted");
    }
    return;
  }
  if (!(liveData->settings.remoteUploadModuleType == REMOTE_UPLOAD_WIFI && liveData->settings.wifiEnabled == 1))
  {
    if (!silent)
    {
      displayMessage("Error", "WiFi not enabled");
    }
    return;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    if (!silent)
    {
      syslog->println("Upload logs: WiFi not connected");
      displayMessage("Error", "WiFi not connected");
    }
    return;
  }

  auto flushPendingSdcardBuffer = [&]() -> bool
  {
    if (sdcardRecordBuffer.length() == 0)
    {
      return true;
    }
    if (strlen(liveData->params.sdcardFilename) == 0)
    {
      return false;
    }

    if (rotateSdV2FileIfNeeded(liveData->params.sdcardFilename,
                               sizeof(liveData->params.sdcardFilename),
                               sdcardRecordBuffer.length()))
    {
      syslog->print("SD v2 rollover file: ");
      syslog->println(liveData->params.sdcardFilename);
    }

    File file = SD.open(liveData->params.sdcardFilename, FILE_APPEND);
    if (!file)
    {
      syslog->println("Failed to open file for appending");
      file = SD.open(liveData->params.sdcardFilename, FILE_WRITE);
    }
    if (!file)
    {
      syslog->println("Failed to create file");
      return false;
    }

    syslog->info(DEBUG_SDCARD, "Save buffer to SD card");
    file.print(sdcardRecordBuffer);
    file.close();
    sdcardRecordBuffer = "";
    liveData->params.sdcardLastFlushMs = millis();
    return true;
  };

  if (!flushPendingSdcardBuffer() && !silent)
  {
    syslog->println("Upload logs: active SD buffer flush failed");
  }

  File dir = SD.open("/");
  if (!dir || !dir.isDirectory())
  {
    if (dir)
    {
      dir.close();
    }
    if (!silent)
    {
      displayMessage("Upload logs", "Open root failed");
    }
    return;
  }

  String uploadedStr;
  uint32_t cntLogs = 0;
  uint32_t cntUploaded = 0;
  const String activeLogFilename = toAbsoluteSdPath(String(liveData->params.sdcardFilename));
  String activeQueuedFile = "";
  String queuedFiles = "";
  queuedFiles.reserve(1024);

  while (true)
  {
    File entry = dir.openNextFile(FILE_READ);
    if (!entry)
    {
      break;
    }
    const bool isDir = entry.isDirectory();
    const String fileName = String(entry.name());
    entry.close();

    if (!isDir)
    {
      const String filePath = toAbsoluteSdPath(fileName);
      const bool isJsonLog = filePath.endsWith(".json");
      const bool isActiveLog = (activeLogFilename.length() > 0 && filePath == activeLogFilename);
      const bool isAlreadyUploadedV2Log = isUploadedSdV2LogFile(filePath);
      if (isJsonLog && !isAlreadyUploadedV2Log)
      {
        if (isActiveLog)
        {
          activeQueuedFile = filePath;
        }
        else
        {
          queuedFiles += filePath;
          queuedFiles += '\n';
        }
      }
    }
  }
  dir.close();

  if (activeQueuedFile.length() > 0)
  {
    queuedFiles += activeQueuedFile;
    queuedFiles += '\n';
  }

  int queuePos = 0;
  while (queuePos < queuedFiles.length())
  {
    int lineEnd = queuedFiles.indexOf('\n', queuePos);
    if (lineEnd < 0)
    {
      lineEnd = queuedFiles.length();
    }
    const String filePath = queuedFiles.substring(queuePos, lineEnd);
    queuePos = lineEnd + 1;
    if (filePath.length() == 0)
    {
      continue;
    }

    const String uploadFileName = (filePath.charAt(0) == '/') ? filePath.substring(1) : filePath;
    cntLogs++;
    int32_t lastProgressShownKb = -1;
    if (!silent)
    {
      displayMessage(filePath.c_str(), "Uploading...");
    }

    uint32_t part = 0;
    uint32_t uploaded = 0;
    File uploadFileMeta = SD.open(filePath.c_str(), FILE_READ);
    const size_t totalSize = uploadFileMeta ? static_cast<size_t>(uploadFileMeta.size()) : 0U;
    bool uploadFailed = false;
    if (!uploadFileMeta || uploadFileMeta.isDirectory())
    {
      uploadFailed = true;
    }
    if (uploadFileMeta)
    {
      uploadFileMeta.close();
    }
    if (!uploadFailed && totalSize > 0U)
    {
      size_t manualChunkSize = kSdLogUploadManualChunkSize;
      bool chunkSizeFallbackUsed = false;
      while (uploaded < totalSize)
      {
        const size_t remaining = totalSize - uploaded;
        const size_t bytesToRead = (remaining < manualChunkSize) ? remaining : manualChunkSize;
        File uploadChunk = SD.open(filePath.c_str(), FILE_READ);
        if (!uploadChunk || uploadChunk.isDirectory())
        {
          if (uploadChunk)
          {
            uploadChunk.close();
          }
          uploadFailed = true;
          if (!silent)
          {
            syslog->println("Log upload local read failed: open");
          }
          break;
        }
        if (!uploadChunk.seek(uploaded))
        {
          uploadChunk.close();
          uploadFailed = true;
          if (!silent)
          {
            syslog->println("Log upload local read failed: seek");
          }
          break;
        }
        const size_t sz = uploadChunk.read(gSdLogUploadBuffer, bytesToRead);
        uploadChunk.close();
        if (sz == 0U)
        {
          uploadFailed = true;
          if (!silent)
          {
            syslog->println("Log upload local read failed: read");
          }
          break;
        }

        if (!silent)
        {
          const int32_t uploadedKb = static_cast<int32_t>(uploaded / 1024);
          const bool showProgress = (part == 0) || (lastProgressShownKb < 0) || (uploadedKb - lastProgressShownKb >= 16);
          if (showProgress)
          {
            lastProgressShownKb = uploadedKb;
            uploadedStr = "Uploading... " + String(uploadedKb) + " / " + String(totalSize / 1024) + "kB";
            displayMessage(filePath.c_str(), uploadedStr.c_str());
          }
        }

        String uploadResponse = "";
        int uploadRc = -1;
        if (!postSdLogChunkToEvDash(uploadFileName, part, gSdLogUploadBuffer, sz, &uploadResponse, &uploadRc, true))
        {
          const bool canFallbackChunkSize = (manualChunkSize > kSdLogUploadChunkSize);
          if (canFallbackChunkSize)
          {
            manualChunkSize = kSdLogUploadChunkSize;
            chunkSizeFallbackUsed = true;
            if (!silent)
            {
              syslog->println("Log upload fallback: manual chunk 4KB");
            }
            continue;
          }

          uploadFailed = true;
          if (!silent)
          {
            syslog->print("Log upload failed rc=");
            syslog->println(uploadRc);
            if (uploadResponse.length() > 0)
            {
              syslog->println(uploadResponse);
            }
          }
          break;
        }

        uploaded += static_cast<uint32_t>(sz);
        part++;
        yield();
      }
      if (!uploadFailed && uploaded < totalSize)
      {
        uploadFailed = true;
        if (!silent)
        {
          syslog->print("Log upload local read short: ");
          syslog->print(uploaded);
          syslog->print(" / ");
          syslog->println(totalSize);
        }
      }
      if (!uploadFailed && chunkSizeFallbackUsed && !silent)
      {
        syslog->println("Log upload completed with 4KB chunk fallback");
      }
    }

    if (!silent)
    {
      displayMessage(filePath.c_str(), (uploadFailed ? "Upload error..." : "Uploaded..."));
    }
    if (!uploadFailed)
    {
      bool finalized = false;
      if (isPendingSdV2LogFile(filePath))
      {
        const String uploadedPath = toUploadedSdV2Path(filePath);
        if (uploadedPath.length() > 0)
        {
          if (SD.exists(uploadedPath.c_str()))
          {
            SD.remove(uploadedPath.c_str());
          }
          finalized = SD.rename(filePath.c_str(), uploadedPath.c_str());
        }
      }
      else
      {
        finalized = SD.remove(filePath.c_str());
      }
      if (finalized)
      {
        cntUploaded++;
      }
    }
    yield();
  }

  if (!silent)
  {
    if (cntLogs == 0)
    {
      displayMessage("Upload logs to server", "No files found");
    }
    else
    {
      uploadedStr = String(cntUploaded) + " of " + String(cntLogs);
      displayMessage("Uploaded", uploadedStr.c_str());
    }
  }
}
