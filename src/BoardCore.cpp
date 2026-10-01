#include "BoardCore.h"
#include <esp_heap_caps.h>
#include "BoardShared.h"

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
