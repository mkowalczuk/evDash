#include "BoardCore.h"
#include <esp_heap_caps.h>
#include "config.h"
#include "CarModelUtils.h"
#include "BoardShared.h"

static String getTraccarDeviceIdFromEfuse();

/**
 * Default RTC read: no battery-backed clock on this board.
 * Returns 0, which tells the caller to fall back to SNTP or GPS.
 */
time_t BoardCore::rtcReadTime()
{
  return 0;
}

/**
 * Default RTC write: nothing to persist a time correction to.
 */
void BoardCore::rtcWriteTime(time_t newTime)
{
  (void)newTime;
}

/**
 * Default UI handling: nothing to poll on a board with no buttons or touch.
 */
void BoardCore::handleUiInput()
{
}

/**
 * Default screen update: nothing to draw without a display.
 */
void BoardCore::updateScreen()
{
}

/**
 * Default storage bring-up: no card interface on this board, so mounting fails
 * rather than pretending to succeed. A board with a slot overrides this.
 */
bool BoardCore::sdBegin()
{
  return false;
}

/**
 * Default display bring-up.
 *
 * Everything in this loop is display-independent: the serial console, the board
 * loop, buttons (via handleUiInput), GPS, the net upload and SD recording, and
 * the sentry/command-queue state machine. Display work is reached only through
 * the seams above, which are no-ops here. The only display state left is the
 * FPS counter, which updateDisplayFps() keeps in Board320_240.
 */
void BoardCore::mainLoop()
{
  updateDisplayFps();

  // Serial console commands
  processSerialConsole();

  // board loop
  boardLoop();

  // Buttons, touch, menu
  handleUiInput();

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
    liveData->params.wifiConnectAttemptStartMs = 0;
  }
  else if (liveData->params.wifiConnectAttemptStartMs != 0 &&
           (millis() - liveData->params.wifiConnectAttemptStartMs) >= 15000)
  {
    liveData->params.wifiConnectAttemptStartMs = 0;
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
    if (commInterface != nullptr && commInterface->isSuspended())
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
    if (commInterface != nullptr)
    {
      commInterface->suspendDevice();
    }
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

  // Display sleep, brightness and redraw
  updateScreen();

  // Read data from BLE/CAN
  commLoop();

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
   Init board
*/
void BoardCore::seedSystemClock()
{
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
 * Sync NTP time
 */
void BoardCore::ntpSync()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    syslog->printf("[NTP] Cannot sync: WiFi not connected (status=%d). Ensure WiFi is connected first.\n", WiFi.status());
    return;
  }

  syslog->printf("[NTP] Starting time sync via SNTP (tz=%d, dst=%d)...\n",
                 liveData->settings.timezone,
                 liveData->settings.daylightSaving);
  const char *ntpServer1 = "pool.ntp.org";
  const char *ntpServer2 = "time.cloudflare.com";
  const char *ntpServer3 = "129.6.15.28"; // NIST, avoids DNS dependency
  configTime(liveData->settings.timezone * 3600, liveData->settings.daylightSaving * 3600,
             ntpServer1, ntpServer2, ntpServer3);
}

/**
 * Synchronize hardware RTC from system clock (called after NTP sync)
 */
void BoardCore::syncRtcFromSystemTime()
{
  rtcWriteTime(time(nullptr));
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
void BoardCore::updateGyroSensorMotion(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ)
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

void BoardCore::recordContributeSample()
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

BoardCore::ContributeChargingEvent BoardCore::captureContributeChargingEventSnapshot(time_t eventTime) const
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

void BoardCore::handleContributeChargingTransitions()
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

void BoardCore::syncContributeRelativeTimes(time_t offset)
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

static time_t utcToEpoch(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds)
{
  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || seconds > 60)
  {
    return 0;
  }

  const uint32_t y = year;
  uint32_t days = (y - 1970) * 365 + ((y - 1969) / 4) - ((y - 1901) / 100) + ((y - 1601) / 400);

  static const uint16_t daysToMonth[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  days += daysToMonth[month - 1];

  const bool isLeap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
  if (isLeap && month > 2)
  {
    days += 1;
  }

  days += (day - 1);

  return static_cast<time_t>(days * 86400ULL + hour * 3600ULL + minute * 60ULL + seconds);
}

/**
 * Set the RTC time using the provided GPS time.
 * Converts the provided date/time components into a UNIX timestamp,
 * sets the system time using settimeofday(), and syncs other times.
 * Also sets the time on the M5Stack Core2 RTC module if being used.
 */
void BoardCore::setGpsTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t seconds)
{
  // Sanity check incoming GPS date and time values.
  // GPS modules can output uninitialized/dummy dates (e.g. 2000-00-00 or 1980-01-06) before acquiring satellite lock.
  if (year < 2024 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || seconds > 60)
  {
    if (liveData->settings.debugLevel & DEBUG_GPS)
    {
      syslog->printf("GPS time rejected: %04u-%02u-%02u %02u:%02u:%02u is not a valid date\n",
                     year, month, day, hour, minute, seconds);
    }
    return;
  }

  time_t t = utcToEpoch(year, month, day, hour, minute, seconds);
  if (t < 1704067200) // 2024-01-01 00:00:00 UTC
  {
    return;
  }

  struct timeval now = {.tv_sec = t, .tv_usec = 0};
  settimeofday(&now, NULL);

  struct tm check;
  if (!getLocalTime(&check, 50))
  {
    syslog->println("GPS time set rejected by system clock.");
    return;
  }

  liveData->params.currTimeSyncWithGps = true;
  syslog->printf("[GPS] System time synced: %04u-%02u-%02u %02u:%02u:%02u UTC\n",
                 year, month, day, hour, minute, seconds);

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
void BoardCore::syncTimes(time_t newTime)
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
 * Synchronizes GPS data with the liveData structure.
 *
 * This function updates GPS-related parameters such as latitude, longitude,
 * altitude, satellite count, speed, and time synchronization if valid data is received.
 */
void BoardCore::syncGPS()
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
  // Validate calendar components: many GPS modules stream uninitialized dummy dates (e.g. 2000-00-00) before lock.
  if (!liveData->params.currTimeSyncWithGps &&
      gps.date.isValid() && gps.time.isValid() &&
      gps.date.year() >= 2024 && gps.date.year() <= 2099 &&
      gps.date.month() >= 1 && gps.date.month() <= 12 &&
      gps.date.day() >= 1 && gps.date.day() <= 31)
  {
    if (liveData->settings.ntpEnabled == 0 || liveData->params.ntpTimeSet || gpsTimeFallbackAllowed)
    {
      setGpsTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
    }
  }
}

/**
 * Show GPS diagnostics and TinyGPSPlus statistics
 */
void BoardCore::showGps()
{
  BoardInterface::showGps();
  syslog->printf("Chars Processed:  %u\n", (uint32_t)gps.charsProcessed());
  syslog->printf("Sentences (Fix):  %u\n", (uint32_t)gps.sentencesWithFix());
  syslog->printf("Checksum Pass:    %u\n", (uint32_t)gps.passedChecksum());
  syslog->printf("Checksum Fail:    %u\n", (uint32_t)gps.failedChecksum());
}

const char *BoardCore::getWifiDisconnectReasonStr(uint8_t reason)
{
  switch (reason)
  {
  case 1: return "UNSPECIFIED";
  case 2: return "AUTH_EXPIRE";
  case 3: return "AUTH_LEAVE";
  case 4: return "ASSOC_EXPIRE";
  case 5: return "ASSOC_TOOMANY";
  case 6: return "NOT_AUTHED";
  case 7: return "NOT_ASSOCED";
  case 8: return "ASSOC_LEAVE";
  case 9: return "ASSOC_NOT_AUTHED";
  case 10: return "DISASSOC_PWRCAP_BAD";
  case 11: return "DISASSOC_SUPCHAN_BAD";
  case 12: return "BSS_TRANSITION_DISASSOC";
  case 13: return "IE_INVALID";
  case 14: return "MIC_FAILURE";
  case 15: return "4WAY_HANDSHAKE_TIMEOUT (check password / signal)";
  case 16: return "GROUP_KEY_UPDATE_TIMEOUT";
  case 17: return "IE_IN_4WAY_DIFFERS";
  case 18: return "GROUP_CIPHER_INVALID";
  case 19: return "PAIRWISE_CIPHER_INVALID";
  case 20: return "AKMP_INVALID";
  case 21: return "UNSUPP_RSN_IE_VERSION";
  case 22: return "INVALID_RSN_IE_CAP";
  case 23: return "802_1X_AUTH_FAILED";
  case 24: return "CIPHER_SUITE_REJECTED";
  case 200: return "BEACON_TIMEOUT (out of range / lost AP)";
  case 201: return "NO_AP_FOUND (SSID not found or out of range)";
  case 202: return "AUTH_FAIL (incorrect password)";
  case 203: return "ASSOC_FAIL";
  case 204: return "HANDSHAKE_TIMEOUT (check password / signal)";
  case 205: return "CONNECTION_FAIL";
  default: return "UNKNOWN";
  }
}

void BoardCore::registerWifiEvents()
{
  static bool s_wifiEventsRegistered = false;
  if (s_wifiEventsRegistered)
    return;
  s_wifiEventsRegistered = true;

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      syslog->printf("[WiFi] Connected to AP: \"%.*s\" (channel: %u)\n",
                     info.wifi_sta_connected.ssid_len,
                     reinterpret_cast<const char *>(info.wifi_sta_connected.ssid),
                     info.wifi_sta_connected.channel);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      syslog->printf("[WiFi] Disconnected from \"%.*s\" (reason %u: %s)\n",
                     info.wifi_sta_disconnected.ssid_len,
                     reinterpret_cast<const char *>(info.wifi_sta_disconnected.ssid),
                     info.wifi_sta_disconnected.reason,
                     getWifiDisconnectReasonStr(info.wifi_sta_disconnected.reason));
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      syslog->printf("[WiFi] IP acquired: %s (Mask: %s, Gateway: %s, DNS: %s)\n",
                     IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str(),
                     IPAddress(info.got_ip.ip_info.netmask.addr).toString().c_str(),
                     IPAddress(info.got_ip.ip_info.gw.addr).toString().c_str(),
                     WiFi.dnsIP(0).toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      syslog->println("[WiFi] Lost IP address.");
      break;
    default:
      break;
    }
  });
}

/**
 * Initializes and connects to WiFi using the stored SSID and password.
 *
 * Enables STA mode, starts the connection, and updates the last connected time.
 *
 * @return True if WiFi initialization and connection succeeded, false otherwise.
 */
bool BoardCore::wifiSetup()
{
  liveData->params.wifiActiveIndex = 0;
  liveData->params.isWifiBackupLive = false;

  registerWifiEvents();

  if (strlen(liveData->settings.wifiSsid) == 0)
  {
    syslog->println("[WiFi] Warning: Primary WiFi SSID is empty. Set with 'wifiSsid=<name>'.");
  }

  syslog->printf("[WiFi] Starting connection to SSID: \"%s\"\n", liveData->settings.wifiSsid);

  // Enable Station mode and start connection.
  // NOTE: Do NOT call WiFi.setSleep(false) here — ESP-IDF requires modem sleep to be
  // enabled when both WiFi and Bluetooth are active. Disabling it triggers an abort():
  // "Should enable WiFi modem sleep when both WiFi and Bluetooth are enabled".
  WiFi.enableSTA(true);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(liveData->settings.wifiSsid, liveData->settings.wifiPassword);

  // Update the last connected time
  liveData->params.wifiLastConnectedTime = liveData->params.currentTime;
  liveData->params.wifiConnectAttemptStartMs = millis();

  return true;
}

/**
 * Handles switching between main and backup WiFi networks (primary, ssid2, ssid3, ssid4).
 *
 * Disconnects from the current WiFi network and attempts connection to the next configured AP.
 */
void BoardCore::wifiFallback()
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
void BoardCore::wifiSwitchToIndex(uint8_t index)
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
  liveData->params.wifiConnectAttemptStartMs = millis();
}

/**
 * Switches to the backup WiFi network (index 1).
 */
void BoardCore::wifiSwitchToBackup()
{
  wifiSwitchToIndex(1);
}

/**
 * Restores the main WiFi connection (index 0).
 */
void BoardCore::wifiSwitchToMain()
{
  wifiSwitchToIndex(0);
}

bool BoardCore::netStatusMessageVisible() const
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

bool BoardCore::isContributeKeyValid(const char *key) const
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

String BoardCore::ensureContributeKey()
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

String BoardCore::getHardwareDeviceId() const
{
  const uint64_t efuse = (ESP.getEfuseMac() & 0xFFFFFFFFFFFFULL);
  const uint32_t uuidPart1 = static_cast<uint32_t>((efuse >> 16) & 0xFFFFFFFFULL);
  const uint16_t uuidPart2 = static_cast<uint16_t>(efuse & 0xFFFFU);
  const uint16_t uuidPart3 = static_cast<uint16_t>(((efuse >> 32) & 0x0FFFU) | 0x4000U); // UUID version 4 layout
  const uint16_t uuidPart4 = static_cast<uint16_t>(((efuse >> 20) & 0x3FFFU) | 0x8000U); // UUID variant 1 layout
  const uint8_t boardTag = hardwareIdTag();
  const uint64_t uuidPart5 = ((efuse ^ (static_cast<uint64_t>(boardTag) << 40)) & 0xFFFFFFFFFFFFULL);
  char deviceId[40] = {0};
  snprintf(deviceId, sizeof(deviceId), "%08lX-%04X-%04X-%04X-%012llX",
           uuidPart1, uuidPart2, uuidPart3, uuidPart4, uuidPart5);
  return String(deviceId);
}

String BoardCore::getPairDeviceId() const
{
  const String hardwareDeviceId = normalizeDeviceIdForApi(getHardwareDeviceId());
  if (hardwareDeviceId.length() == 0)
  {
    return "";
  }
  return hardwareDeviceId;
}

bool BoardCore::requestPairingStart(String &outCode, uint32_t &outExpiresInSec)
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

uint8_t BoardCore::requestPairingStatus(const String &pairCode, String &outCarName)
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

void BoardCore::startEvdashPairing()
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

void BoardCore::pollEvdashPairingStatus()
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

int BoardCore::compareVersionTags(const String &left, const String &right) const
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

void BoardCore::checkFirmwareVersionOnServer()
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

void BoardCore::addWifiTransferredBytes(size_t bytes)
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

void BoardCore::updateNetAvailability(bool success)
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

void BoardCore::disconnectMqtt(bool sendOfflineStatus)
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

bool BoardCore::ensureMqttConnected()
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

void BoardCore::publishHaSensor(const char *component, const char *objectId, const char *name,
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
                         ((strlen(rawTopic) > 0) ? rawTopic : hardwareModelName());
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

void BoardCore::publishHomeAssistantDiscovery()
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

/**
 * Send data
 **/
bool BoardCore::netSendData(bool sendAbrp)
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
          if (mqttClient != nullptr && mqttClient->connected())
          {
            mqttClient->loop();
          }
        }
        else
        {
          rc = -1;
        }
      }
      else
      {
        // Standard http post
        WiFiClient wClient;
        HTTPClient http;

        http.begin(wClient, liveData->settings.remoteApiUrl);
        http.setConnectTimeout(2000);
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
    syslog->info(DEBUG_NET, "Well... This not gonna happen... (BoardCore::netSendData();)"); // Just for debug reasons...
  }
  // next three rows are for time measurement of this function
  int64_t endTime2 = esp_timer_get_time();
  int64_t duration2 = endTime2 - startTime2;
  // syslog->println("Time taken by function: netSendData() " + String(duration2) + " microseconds");

  return true;
}

/**
 * Net loop, send data over net
 * Checks if WiFi is connected, syncs NTP if needed, sends data to remote API if interval elapsed,
 * sends data to ABRP if interval elapsed, contributes data if enabled.
 */
void BoardCore::netLoop()
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
  else if (liveData->settings.mqttEnabled == 1)
  {
    if (mqttClient != nullptr && mqttClient->connected())
    {
      mqttClient->loop();
    }
  }

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
      syslog->printf("[NTP] Sync timeout (60s). WiFi status=%d, IP=%s. Falling back to GPS time.\n",
                     WiFi.status(),
                     WiFi.localIP().toString().c_str());
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
    syslog->println("[NTP] Time synchronized successfully.");
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

void BoardCore::queueAbrpSdLog(const char *payload, size_t length, time_t currentTime, uint64_t operationTimeSec, bool timeSyncWithGps)
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
 * Called for any board that has contribute enabled in its settings.
 **/
bool BoardCore::netContributeData()
{
  int rc = 0;

// Only for core2

  return true;
}

void BoardCore::runSdV2BackgroundTasks(bool netReady)
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

bool BoardCore::ensureSdV2UploadFileSelected(const String &activeLogFilename)
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

bool BoardCore::processSdV2UploadChunk()
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

void BoardCore::resetSdV2UploadState()
{
  sdV2UploadFilePath = "";
  sdV2UploadFileName = "";
  sdV2UploadPart = 0;
  sdV2UploadOffset = 0;
}

bool BoardCore::postSdLogChunkToEvDash(const String &fileName, uint32_t part, const uint8_t *data, size_t length, String *responsePayload, int *responseCode, bool preferManualTimeouts)
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

bool BoardCore::cleanupUploadedSdV2Logs()
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
void BoardCore::initGPS()
{
  syslog->print("GPS initialization on hwUart: ");
  syslog->println(liveData->settings.gpsHwSerialPort);

  gpsHwUart = new HardwareSerial(liveData->settings.gpsHwSerialPort);
  auto beginGpsUart = [&](unsigned long baud)
  {
    // Only UART1 and UART2 can be routed to arbitrary pins, and even then the
    // pins are board-specific; a board that exposes none takes the defaults.
    const int rxPin = gpsUartRxPin();
    const int txPin = gpsUartTxPin();
    if ((liveData->settings.gpsHwSerialPort == 1 || liveData->settings.gpsHwSerialPort == 2) &&
        rxPin >= 0 && txPin >= 0)
    {
      gpsHwUart->begin(baud, SERIAL_8N1, rxPin, txPin);
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
void BoardCore::sendCasicGpsCommand(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t payloadLen)
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
void BoardCore::setGpsV21Pps(bool enabled)
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
void BoardCore::updateGpsV21PpsMode()
{
  if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
  {
    setGpsV21Pps(liveData->params.stopCommandQueue);
  }
}

/**
 * This function uploads log files from the SD card to the EvDash server.
 */
void BoardCore::uploadSdCardLogToEvDashServer(bool silent)
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
