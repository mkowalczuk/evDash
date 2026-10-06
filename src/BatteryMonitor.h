#pragma once

#include <Arduino.h>
#include <cstdint>

/**
 * State of the 18650 battery cell
 */
struct BatteryPackState
{
  float voltage = 0.0f;
  float current = -999.0f; // -999 indicates current sense unavailable
  float soc = 0.0f;
  float temperature = -999.0f;
  bool isPresent = false;
  bool isCharging = false;
};

/**
 * 18650 single-cell (1S) battery monitor
 */
class BatteryMonitor
{
private:
  uint8_t packType = 0;     // 0 = Li-ion 18650 (3.7V nom), 1 = LiFePO4 (3.2V nom)
  bool usbChargeOnly = true;

  BatteryPackState state;
  bool gaugeReady = false;

public:
  BatteryMonitor() = default;

  void configure(uint8_t type, bool usbCharge);
  void updateSingleGauge(float rawCellVoltage, float rawSoc, bool usbPowered);

  const BatteryPackState &getState() const { return state; }
  uint8_t getPackType() const { return packType; }
  bool isUsbChargeOnly() const { return usbChargeOnly; }
  bool isGaugeReady() const { return gaugeReady; }

  float getVoltage() const { return state.voltage; }
  float getSoc() const { return state.soc; }
  const char *packTypeName() const;
};
