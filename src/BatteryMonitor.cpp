#include "BatteryMonitor.h"

void BatteryMonitor::configure(uint8_t type, bool usbCharge)
{
  packType = type;
  usbChargeOnly = usbCharge;
}

void BatteryMonitor::updateSingleGauge(float rawCellVoltage, float rawSoc, bool usbPowered)
{
  // A raw reading of <= 0.05 indicates gauge not yet initialized or absent
  if (rawCellVoltage <= 0.05f)
  {
    gaugeReady = false;
    return;
  }

  gaugeReady = true;
  state.voltage = rawCellVoltage;
  state.soc = (rawSoc < 0.0f) ? 0.0f : (rawSoc > 100.0f) ? 100.0f : rawSoc;
  state.isPresent = true;
  state.isCharging = usbPowered && (!usbChargeOnly || usbPowered);
}

const char *BatteryMonitor::packTypeName() const
{
  switch (packType)
  {
  case 1:
    return "LiFePO4 (3.2V nom)";
  case 0:
  default:
    return "Li-ion 18650 (3.7V nom)";
  }
}
