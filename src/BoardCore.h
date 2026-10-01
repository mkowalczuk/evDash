#pragma once

#include "BoardInterface.h"

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
  virtual void displayMessage(const char *row1, const char *row2) { (void)row1; (void)row2; }
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
};