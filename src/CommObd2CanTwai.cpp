#include "CommObd2CanTwai.h"

CommObd2CanTwai::CommObd2CanTwai(int txPin, int rxPin)
{
#if defined(ESP32) || defined(ESP32S3)
  canDriver.reset(new TwaiCanDriver(txPin, rxPin));
#endif
}
