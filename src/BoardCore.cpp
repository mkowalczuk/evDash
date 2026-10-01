#include "BoardCore.h"

/**
 * Default UI handling: nothing to poll on a board with no buttons or touch.
 */
void BoardCore::handleUiInput()
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
 * Default screen update: nothing to draw without a display.
 */
void BoardCore::updateScreen()
{
}

/**
 * Default RTC behaviour: no battery-backed clock on this board.
 *
 * Returning 0 means "no time available" - callers treat that as "fall back to
 * SNTP". rtcWriteTime() discards the write, which is correct for a board whose
 * clock is kept in sync from the network.
 */
time_t BoardCore::rtcReadTime()
{
  return 0;
}

void BoardCore::rtcWriteTime(time_t newTime)
{
  (void)newTime;
}