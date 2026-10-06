#pragma once

#include "CommObd2Can.h"
#include "config.h"

#ifndef CAN_TX_PIN
#define CAN_TX_PIN 1
#endif
#ifndef CAN_RX_PIN
#define CAN_RX_PIN 2
#endif

class CommObd2CanTwai : public CommObd2Can
{
public:
  CommObd2CanTwai(int txPin = CAN_TX_PIN, int rxPin = CAN_RX_PIN);
};
