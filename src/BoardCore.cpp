#include "BoardCore.h"

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