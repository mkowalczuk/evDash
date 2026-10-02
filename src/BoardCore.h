#pragma once

#include "BoardInterface.h"
#include <TinyGPS++.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "SDL_Arduino_INA3221.h"

/**
 * Display-free board logic.
 *
 * BoardCore holds everything evDash does that is independent of a screen:
 * time handling, networking, telemetry upload, SD recording, GPS and device
 * identity. Board320_240 layers a 320x240 display on top of it; a headless
 * board derives directly from BoardCore and inherits the no-op defaults here
 * for every capability it does not implement.
 *
 * This class exists so that display-specific code does not have to be compiled
 * out of a headless build. Display members are declared as virtual with empty
 * defaults rather than left pure virtual, so a subclass with no screen is still
 * instantiable without overriding anything.
 */
class BoardCore : public BoardInterface
{
public:
  //
  // RTC
  //
  // Boards with a battery-backed RTC override these to seed the system clock at
  // boot and to persist time corrections. Boards without one (or with only
  // SNTP) inherit the defaults: nothing is read, and writes are discarded.
  // A real-time clock is distinct from a display, so this seam is separate from
  // the graphics ones below.
  virtual time_t rtcReadTime();
  virtual void rtcWriteTime(time_t newTime);
  //
  // Display
  //
  // These default to doing nothing so a headless board is instantiable without
  // a screen. Board320_240 overrides the ones it draws with; the rest are pure
  // display bookkeeping that BoardCore calls unconditionally.
  virtual void handleUiInput();
  virtual void updateScreen();
  //
  // Storage
  //
  // How the SD/TF card is electrically attached varies by board: the M5 boards
  // use SPI with a chip-select pin, the Waveshare board uses the ESP32-S3's SDMMC
  // peripheral with fixed clock/command/data pins. sdBegin() is that choice;
  // everything above it (recording, log rotation, chunked upload) is shared.
  virtual bool sdBegin();

  //
  // Identity
  //
  // Reported to the server and shown in Home Assistant discovery. Defaults to
  // the M5 Core2 name; every other board overrides it, because this string is
  // what distinguishes one evDash build from another in the fleet.
  virtual const char *hardwareModelName() { return "Core2"; }

  // Byte mixed into the efuse-derived device UUID. It has to differ per board,
  // otherwise a device that changes hardware keeps claiming the previous one's
  // identity on the server. M5 Core2 keeps the original 0x02, CoreS3 0x03.
  virtual uint8_t hardwareIdTag() const { return 0x02; }

  //
  // Clock
  //
  // Seeds the system clock at boot and registers the SNTP completion callback.
  // Shared rather than per-board because the callback and the flag the network
  // loop tests are file-scope state: splitting seeding and the loop across
  // translation units would leave each looking at its own copy.
  void seedSystemClock();

  //
  // Loop and lifecycle
  //
  // Still pure while the remaining shared logic lives in Board320_240. They are
  // defaulted here (and become real overrides again on each board) so that
  // BoardCore itself is instantiable - a headless board can be brought up and
  // flashed before every one of these has been moved across.
  virtual void updateDisplayFps() {}
  virtual void initBoard() {}
  virtual void afterSetup() {}
  virtual void commLoop() {}
  virtual void boardLoop() {}
  virtual void mainLoop();
  virtual void enterSleepMode(int secs) { (void)secs; }
  virtual void ntpSync() {}
  bool netSendData(bool sendAbrp) override;
  void disconnectMqtt(bool sendOfflineStatus = false) override;
  virtual void sdcardToggleRecording() {}
  virtual void displayMessage(const char *row1, const char *row2) { (void)row1; (void)row2; }
  virtual void displayMessage(const char *row1, const char *row2, const char *row3)
  {
    (void)row1; (void)row2; (void)row3;
  }
  virtual void turnOffScreen() {}
  virtual void setBrightness() {}
  virtual void redrawScreen() {}
  virtual void logDisplayHealth() {}
  virtual void showMenu() {}
  virtual void hideMenu() {}
  virtual void otaUpdate() {}
  virtual void showBootProgress(const char *step, const char *detail, uint16_t bgColor = 0) { (void)step; (void)detail; (void)bgColor; }
  //
  // Screen count drives the button-driven screen rotation. A headless board has
  // no screens to rotate through, so 0 is meaningful here rather than a bug.
  void setDisplayScreenCount(uint8_t count) { displayScreenCount = count; }

protected:
  //
  // Hardware handles and shared (display-free) runtime state.
  //
  HardwareSerial *gpsHwUart = NULL;
  SDL_Arduino_INA3221 ina3221;
  TinyGPSPlus gps;
  // time in fwd mode for avg speed calc.
  time_t previousForwardDriveModeTotal = 0;
  time_t lastForwardDriveModeStart = 0;
  bool lastForwardDriveMode = false;
  float forwardDriveOdoKmStart = -1;
  float forwardDriveOdoKmLast = -1;
  uint32_t mainLoopStart = 0;
  uint32_t lastTimeUpdateMs = 0;
  uint32_t suppressTouchInputUntilMs = 0;
  bool keyboardInputActive = false;
  bool messageDialogVisible = false;
  time_t cachedNowEpoch = 0;
  struct tm cachedNow = {};
  float displayFps = 0;
  bool modalDialogActive = false;
  static constexpr time_t kMenuAutoHideTimeoutSec = 60;
  bool screenSwipePreviewActive = false;
  String dismissedCanStatusText = "";
  time_t dismissedNetFailureTime = 0;
  bool lastChargingOn = false;
  uint32_t lastNetSendDurationMs = 0;
  uint32_t lastRemoteSendAtMs = 0;
  uint32_t lastAbrpSendAtMs = 0;
  uint32_t lastTraccarSendAtMs = 0;
  WiFiClient *mqttPlainClient = nullptr;
  WiFiClientSecure *mqttSecureClient = nullptr;
  PubSubClient *mqttClient = nullptr;
  uint32_t lastMqttReconnectAttemptMs = 0;
  static constexpr uint32_t kMqttReconnectBackoffMs = 15000;
  bool ensureMqttConnected();
  void publishHomeAssistantDiscovery();
  void publishHaSensor(const char *component, const char *objectId, const char *name,
                       const char *deviceClass, const char *unit, const char *stateClass,
                       const char *entityCategory = nullptr, const char *payloadOn = nullptr, const char *payloadOff = nullptr);
  uint32_t wifiTransferredBytes = 0;
  uint32_t wifiTransferLastActivityMs = 0;
  uint32_t lastFirmwareVersionCheckMs = 0;
  bool lastWifiConnected = false;
  uint32_t ntpAttemptStartMs = 0;
  uint32_t ntpLastAttemptMs = 0;
  bool gpsTimeFallbackAllowed = false;
  bool gpsV21PpsModeKnown = false;
  bool gpsV21PpsEnabled = false;
  String lastNotifiedFirmwareVersion = "";
  char pairPendingCode[7] = {0};
  time_t pairPendingExpiresAt = 0;
  uint32_t pairLastStatusPollMs = 0;
  uint8_t pairLastKnownState = 0; // 0-none, 1-pending, 2-paired, 3-expired
  static constexpr uint8_t kContributeSampleSlots = 12;
  static constexpr time_t kContributeSampleIntervalSec = 5;
  static constexpr time_t kContributeSampleWindowSec = 60;
  static constexpr uint32_t kContributeCycleIntervalMs = static_cast<uint32_t>(kContributeSampleWindowSec) * 1000U;
  static constexpr uint32_t kContributeWaitFallbackMs = 4000;
  struct ContributeMotionSample
  {
    time_t time = 0;
    bool hasGpsFix = false;
    float lat = -1.0f;
    float lon = -1.0f;
    float speedKmh = -1.0f;
    float headingDeg = -1.0f;
    float cellMinV = -1.0f;
    float cellMaxV = -1.0f;
    uint8_t cellMinNo = 255;
  };
  struct ContributeChargingSample
  {
    time_t time = 0;
    float soc = -1.0f;
    float batV = -1.0f;
    float batA = -1000.0f;
    float powKw = -1000.0f;
  };
  struct ContributeChargingEvent
  {
    bool valid = false;
    time_t time = 0;
    float soc = -1.0f;
    float batV = -1.0f;
    float batA = -1000.0f;
    float cellMinV = -1.0f;
    float cellMaxV = -1.0f;
    uint8_t cellMinNo = 255;
    float batMinC = -100.0f;
    float batMaxC = -100.0f;
    float cecKWh = -1.0f;
    float cedKWh = -1.0f;
  };
  ContributeMotionSample contributeMotionSamples[kContributeSampleSlots] = {};
  ContributeChargingSample contributeChargingSamples[kContributeSampleSlots] = {};
  uint8_t contributeMotionSampleCount = 0;
  uint8_t contributeMotionSampleNext = 0;
  uint8_t contributeChargingSampleCount = 0;
  uint8_t contributeChargingSampleNext = 0;
  time_t lastContributeSampleTime = 0;
  time_t lastContributeSdRecordTime = 0;
  uint32_t contributeStatusSinceMs = 0;
  uint32_t nextContributeCycleAtMs = 0;
  static constexpr uint32_t kSdV2BackgroundUploadIntervalMs = 5000U;
  static constexpr uint32_t kSdV2BackgroundStartDelayMs = 5U * 60U * 1000U;
  static constexpr uint32_t kSdV2BackgroundRetryBackoffMs = 60U * 60U * 1000U;
  static constexpr uint32_t kSdV2CleanupIntervalMs = 6U * 60U * 60U * 1000U;
  static constexpr time_t kSdV2UploadedRetentionSec = 30 * 24 * 60 * 60;
  uint32_t nextSdV2BackgroundUploadAtMs = 0;
  uint32_t sdV2BackgroundStartAtMs = 0;
  uint32_t nextSdV2CleanupAtMs = 0;
  String sdV2UploadFilePath = "";
  String sdV2UploadFileName = "";
  uint32_t sdV2UploadPart = 0;
  uint32_t sdV2UploadOffset = 0;
  ContributeChargingEvent contributeLastBeforeCharge = {};
  ContributeChargingEvent contributeLastDuringCharge = {};
  ContributeChargingEvent contributeChargingStartEvent = {};
  ContributeChargingEvent contributeChargingEndEvent = {};
  void updateNetAvailability(bool success);
  bool netStatusMessageVisible() const;
  bool isContributeKeyValid(const char *key) const;
  String ensureContributeKey();
  String getHardwareDeviceId() const;
  String getPairDeviceId() const;
  int compareVersionTags(const String &left, const String &right) const;
  void checkFirmwareVersionOnServer();
  bool requestPairingStart(String &outCode, uint32_t &outExpiresInSec);
  uint8_t requestPairingStatus(const String &pairCode, String &outCarName);
  void startEvdashPairing();
  void pollEvdashPairingStatus();
  void addWifiTransferredBytes(size_t bytes);
  void recordContributeSample();
  ContributeChargingEvent captureContributeChargingEventSnapshot(time_t eventTime) const;
  void handleContributeChargingTransitions();
  void syncContributeRelativeTimes(time_t offset);
  void runSdV2BackgroundTasks(bool netReady);
  bool ensureSdV2UploadFileSelected(const String &activeLogFilename);
  bool processSdV2UploadChunk();
  void resetSdV2UploadState();
  bool postSdLogChunkToEvDash(const String &fileName, uint32_t part, const uint8_t *data, size_t length, String *responsePayload = nullptr, int *responseCode = nullptr, bool preferManualTimeouts = false);
  bool cleanupUploadedSdV2Logs();
  void sendCasicGpsCommand(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t payloadLen);
  void setGpsV21Pps(bool enabled);
  void updateGpsV21PpsMode();
  void updateGyroSensorMotion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ);
  void initGPS();
  void syncGPS();
  void syncTimes(time_t newTime);
  virtual void setGpsTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds);
  // Notwork
  bool wifiSetup();
  void netLoop();
  bool netContributeData();
  void wifiFallback();
  void wifiSwitchToMain();
  void wifiSwitchToBackup();
  void wifiSwitchToIndex(uint8_t index);
  void uploadSdCardLogToEvDashServer(bool silent = false);
  void queueAbrpSdLog(const char *payload, size_t length, time_t currentTime, uint64_t operationTimeSec, bool timeSyncWithGps);
protected:
  // Pin pair for the external NMEA GPS UART, or a negative value when the board
  // has no separate NMEA line. The M5 boards take their pins from the M5 headers;
  // a board whose GNSS arrives as NMEA over the modem's AT UART has none.
  virtual int gpsUartRxPin() { return -1; }
  virtual int gpsUartTxPin() { return -1; }

public:
  // Button GPIOs. Read by the shared main loop; a headless board leaves them 0
  // and simply never matches a button press.
  byte pinButtonLeft = 0;
  byte pinButtonRight = 0;
  byte pinButtonMiddle = 0;
  // Screen brightness as currently applied. The network path reads it to dim the
  // transfer indicator without a screen, so it lives with the shared state.
  uint8_t currentBrightness = 255;
};
