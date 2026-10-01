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
};