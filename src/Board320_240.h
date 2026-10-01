#pragma once

//
#include <TinyGPS++.h>
#include "BoardCore.h"
#include <SD.h>
#include <SPI.h>

#ifdef BOARD_M5STACK_CORE2
#include <M5Core2.h>
#define fontRobotoThin24 &Roboto_Thin_24
#define fontOrbitronLight24 &Orbitron_Light_24
#define fontOrbitronLight32 &Orbitron_Light_32
#define fontFont2 &FreeSans9pt7b
#define fontFont7 &FreeSansBold12pt7b

#endif // BOARD_M5STACK_CORE2
#ifdef BOARD_M5STACK_CORES3
#include <M5CoreS3.h>
#define fontRobotoThin24 &fonts::Roboto_Thin_24
#define fontOrbitronLight24 &fonts::Orbitron_Light_24
#define fontOrbitronLight32 &fonts::Orbitron_Light_32
#define fontFont2 &FreeSans9pt7b
#define fontFont7 &FreeSansBold12pt7b
#define fontFont2bmp &fonts::Font2
#define fontFont7bmp &fonts::Font7
#endif // BOARD_M5STACK_CORES3

class WebServer;
class PubSubClient;
class WiFiClient;
class WiFiClientSecure;

class Board320_240 : public BoardCore
{

protected:
// TFT, SD SPI
#if BOARD_M5STACK_CORE2
  M5Display tft;
  TFT_eSprite spr = TFT_eSprite(&tft);
  const GFXfont *lastFont;
  void sprSetFont(const GFXfont *f);
#endif // BOARD_M5STACK_CORE
#if BOARD_M5STACK_CORES3
  M5GFX &tft = CoreS3.Display;
  M5Canvas spr = M5Canvas(&CoreS3.Display);
  const lgfx::GFXfont *lastFont;
  void sprSetFont(const lgfx::GFXfont *font);
#endif // BOARD_M5STACK_CORES3
  static constexpr int16_t menuBackbufferOverscanPx = 20;
  uint8_t spriteColorDepth = 8;
  bool menuBackbufferActive = false;
  bool ensureMenuBackbuffer();
  void releaseMenuBackbuffer();
  void sprDrawString(const char *string, int32_t poX, int32_t poY);
  void tftDrawStringFont7(const char *string, int32_t poX, int32_t poY);
  char tmpStr1[64];
  char tmpStr2[20];
  char tmpStr3[20];
  char tmpStr4[20];
  float lastSpeedKmh = 0;
  int firstReload = 0;
  uint8_t menuVisibleCount = 5;
  uint8_t menuItemHeight = 20;
  uint8_t menuItemOffsetPx = 0;
  bool menuDragScrollActive = false;
  int16_t menuTouchHoverIndex = -1;
  uint16_t batteryCellsPage = 0;
  uint8_t debugInfoPage = 0;
  time_t lastRedrawTime = 0;

public:
  byte pinSpeaker = 0;
  byte pinBrightness = 0;
  byte pinSdcardCs = 0;
  byte pinSdcardMosi = 0;
  byte pinSdcardMiso = 0;
  byte pinSdcardSclk = 0;
  //
  void initBoard() override;
  void afterSetup() override;
  // RTC - both M5 boards have a battery-backed clock; see BoardCore for the
  // display-free defaults used by boards that do not.
  time_t rtcReadTime() override;
  void rtcWriteTime(time_t newTime) override;
  void commLoop() override;
  void boardLoop() override;
  void mainLoop() override;
  bool skipAdapterScan() override;
  void otaUpdate() override;
  // SD card
  bool sdcardMount() override;
  bool sdBegin() override;
  void sdcardToggleRecording() override;
  void sdcardEraseLogs();
  void enforceSdLogSpaceLimit();
  bool startSdcardConsoleLog();
  void stopSdcardConsoleLog();
  void runWebLogServer();
  void drawWebLogServerScreen(const String &ssid, const String &ip, bool isSta);
  void handleWebLogRoot(WebServer &server);
  void handleWebLogDownload(WebServer &server);
  void handleWebLogView(WebServer &server);
  void handleWebLogDelete(WebServer &server);
  // GPS
  bool netSendData(bool sendAbrp) override;
  void disconnectMqtt(bool sendOfflineStatus = false) override;
  bool buildContributePayloadV2(String &outJson, bool useReadableTsForSd = false) override;
  bool wifiScanToMenu();
  bool promptKeyboard(const char *title, String &value, bool mask, uint8_t maxLen = 63);
  bool promptWifiPassword(const char *ssid, String &outPassword, bool isOpenNetwork);
  bool canStatusMessageVisible();
  bool canStatusMessageHitTest(int16_t x, int16_t y);
  void dismissCanStatusMessage();
  inline bool isUpperLeftTouch(int16_t x, int16_t y) const { return x < 64 && y < 64; }
  void showBootProgress(const char *step, const char *detail, uint16_t bgColor = TFT_BLACK) override;
  // Basic GUI
  // Extracted from mainLoop(): display work a displayless board skips entirely.
  void handleUiInput() override;
  void updateScreen() override;
  void updateDisplayFps() override;
  void turnOffScreen() override;
  void setBrightness() override;
  void displayMessage(const char *row1, const char *row2) override;
  void displayMessage(const char *row1, const char *row2, const char *row3);
  bool confirmMessage(const char *row1, const char *row2) override;
  bool drawActiveScreenToSprite();
  void showScreenSwipePreview(int16_t deltaX);
  void redrawScreen() override;
  // Custom screens
  void drawBigCell(int32_t x, int32_t y, int32_t w, int32_t h, const char *text, const char *desc, uint16_t bgColor, uint16_t fgColor);
  void drawSmallCell(int32_t x, int32_t y, int32_t w, int32_t h, const char *text, const char *desc, int16_t bgColor, int16_t fgColor);
  void showTires(int32_t x, int32_t y, int32_t w, int32_t h, const char *topleft, const char *topright, const char *bottomleft, const char *bottomright, uint16_t color);
  void drawSceneMain();
  void drawSceneSpeed();
  void drawSceneHud();
  uint16_t batteryCellsRowsPerPage();
  uint16_t batteryCellsColumns();
  uint16_t batteryCellsCellsPerPage();
  uint16_t batteryCellsPageCount();
  void batteryCellsPageMove(bool forward);
  uint8_t debugInfoPageCount();
  void debugInfoPageMove(bool forward);
  void drawSceneBatteryCells();
  void drawPreDrawnChargingGraphs(int zeroX, int zeroY, int mulX, float mulY);
  void drawSceneChargingGraph();
  void drawSceneSoc10Table();
  void drawSceneDebug();
  void suppressTouchInputFor(uint16_t durationMs = 220);
  bool isTouchInputSuppressed() const;
  bool isKeyboardInputActive() const;
  bool isMessageDialogVisible() const;
  bool dismissMessageDialog();
  // Menu
  uint16_t menuItemsCountCurrent();
  void menuScrollByPixels(int16_t deltaTopPx);
  uint16_t menuItemFromTouchY(int16_t touchY);
  String menuItemText(int16_t menuItemId, String title);
  void showMenu() override;
  void hideMenu() override;
  void menuMove(bool forward, bool rotate = true);
  void menuItemClick();
  //
  void loadTestData();
  void printHeapMemory();
  //
};
