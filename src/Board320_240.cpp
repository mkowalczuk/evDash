
// This file is display-only: it draws on a 320x240 panel and reads touch and
// buttons. A headless board has no screen, so it is not compiled at all there.
// Everything display-independent lives in BoardCore.cpp.
#if defined(BOARD_M5STACK_CORE2) || defined(BOARD_M5STACK_CORES3)

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
#include <PubSubClient.h>
#include "BoardShared.h"

/**
   Bring up the SD/TF card over SPI.
   The M5 boards expose the card on the SPI bus behind a chip-select pin.
   Core2's pin came from M5Core2's own utility/Config.h, which Board320_240
   included transitively; it is now stated here rather than relied upon.
*/
bool Board320_240::sdBegin()
{
#if defined(BOARD_M5STACK_CORES3) || defined(BOARD_M5STACK_CORE2)
  // Named "sTfCardCs" rather than TFCARD_CS_PIN because M5Core2's utility/Config.h
  // defines that as a macro, and a local of the same name does not compile.
  const uint8_t sTfCardCs = 4;
  return SD.begin(sTfCardCs, SPI, 40000000);
#else
  // No SPI SD slot on this board variant.
  return false;
#endif
}

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

void Board320_240::initBoard()
{
  liveData->params.booting = true;

  seedSystemClock();
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

  // Init comm device (BLE/BT) - MUST come before WiFi setup.
  // On ESP32-S3 with NimBLE, starting WiFi before the BT controller is initialized
  // causes coex_core_enable() to abort when BT later tries to join an already-WiFi-only
  // coexistence context. BT must register with the coex module first.
  showBootProgress("Adapter initialization...", "Starting OBD2/CAN comm", TFT_SILVER);
  BoardInterface::afterSetup();
  printHeapMemory();

  // WiFi - started AFTER BLE so both subsystems are registered with coex before either
  // starts using RF. (Previous comment "Starting WiFi after BLE prevents reboot loop"
  // was correct; this restores the intended order.)
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

/**
 * Button, touch and menu handling for a board with a display.
 * Debouncing, menu navigation, screen rotation and menu auto-hide.
 *
 * Extracted from mainLoop() so BoardCore can call one seam which a displayless
 * board overrides with a no-op.
 */
void Board320_240::handleUiInput()
{
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

}

/**
 * Display housekeeping for a board with a screen: automatic sleep after
 * inactivity, brightness, and the periodic redraw.
 *
 * Extracted from mainLoop() for the same reason as handleUiInput(). commLoop()
 * deliberately stays in mainLoop(): reading BLE/CAN data is not display work and
 * must keep running on a board with no screen.
 */
void Board320_240::updateScreen()
{
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

    // force redraw (min 1 sec update; slower while in Sentry)
    const time_t redrawIntervalSec = liveData->params.stopCommandQueue ? 2 : 1;
    if (!screenSwipePreviewActive &&
        (liveData->params.currentTime - lastRedrawTime >= redrawIntervalSec || liveData->redrawScreenRequested))
    {
      redrawScreen();
    }
}

/**
 * Main loop - primary thread.
 *
 * The loop body is display-independent and now lives in BoardCore::mainLoop().
 * All that remains here is the FPS counter, which feeds drawSceneDebug() on the
 * debug screen; a board with no display has nothing to measure.
 */
void Board320_240::mainLoop()
{
  BoardCore::mainLoop();
}

void Board320_240::updateDisplayFps()
{
  const uint32_t loopDurationMs = (millis() - mainLoopStart);
  displayFps = (loopDurationMs == 0 ? 0 : (1000.0f / loopDurationMs));
  mainLoopStart = millis();
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
  SdState = sdBegin();
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
 * Initializes and connects to WiFi using the stored SSID and password.
 *
 * Enables STA mode, starts the connection, and updates the last connected time.
 *
 * @return True if WiFi initialization and connection succeeded, false otherwise.
 */

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

// The M5 headers fix the external NMEA UART on the second serial port.
int Board320_240::gpsUartRxPin()
{
  return SERIAL2_RX;
}

int Board320_240::gpsUartTxPin()
{
  return SERIAL2_TX;
}

#endif // BOARD_M5STACK_CORE2 || BOARD_M5STACK_CORES3
