#include <Arduino.h>
#include <stdint.h>
#include <WString.h>
#include <string.h>
#include <sys/time.h>
#include "LiveData.h"
#include "CarXpeng.h"
#include <vector>

namespace
{
  const uint16_t kXpengCellCount = 212;
  const uint8_t kXpengModuleTempCount = 8;

  bool inRange(float value, float min, float max)
  {
    return value >= min && value <= max;
  }

  String didFromRequest(String request)
  {
    request.replace(" ", "");
    request.toUpperCase();
    if (request.startsWith("22") && request.length() >= 6)
      return request.substring(2, 6);
    return "";
  }

  double hexToDecPart(const String &response, uint16_t from, uint16_t to, uint8_t bytes, bool signedNum)
  {
    if (bytes < 1 || bytes > 4 || to > response.length() || from >= to)
      return -1;

    uint64_t value = strtoull(response.substring(from, to).c_str(), NULL, 16);
    if (signedNum)
    {
      const uint64_t range = ((uint64_t)1) << (bytes * 8);
      if (value > (range - 1) / 2)
        return (double)((int64_t)value - (int64_t)range);
    }
    return (double)value;
  }

  void updateAuxPercent(LiveData *liveData, float voltage)
  {
    // Resting Lead-Acid / LiFePO4 12V voltage range (11.6V = 0%, 12.8V = 100%).
    // When DC-DC converter is active (charging 12V at 13.0V+), clamp cleanly at 100% instead of jumping.
    const float minV = 11.6f;
    const float maxV = 12.8f;
    const float pct = (voltage - minV) * 100.0f / (maxV - minV);
    liveData->params.auxPerc = (pct > 100.0f) ? 100.0f : ((pct < 0.0f) ? 0.0f : pct);
  }

  void updateChargeGraph(LiveData *liveData)
  {
    if (liveData->params.speedKmh < 10 && liveData->params.batPowerKw >= 1.0f && inRange(liveData->params.socPerc, 0.0f, 100.0f))
    {
      int socIndex = static_cast<int>(liveData->params.socPerc);
      if (liveData->params.chargingGraphMinKw[socIndex] < 0 || liveData->params.batPowerKw < liveData->params.chargingGraphMinKw[socIndex])
        liveData->params.chargingGraphMinKw[socIndex] = liveData->params.batPowerKw;
      if (liveData->params.chargingGraphMaxKw[socIndex] < 0 || liveData->params.batPowerKw > liveData->params.chargingGraphMaxKw[socIndex])
        liveData->params.chargingGraphMaxKw[socIndex] = liveData->params.batPowerKw;
      liveData->params.chargingGraphBatMinTempC[socIndex] = liveData->params.batMinC;
      liveData->params.chargingGraphBatMaxTempC[socIndex] = liveData->params.batMaxC;
      liveData->params.chargingGraphWaterCoolantTempC[socIndex] = liveData->params.coolantTemp1C;
    }
  }
} // namespace

/**
   activateCommandQueue
*/
void CarXpeng::activateCommandQueue()
{
  std::vector<String> commandQueueXpeng = {
      "AT Z",    // Reset all
      "AT I",    // Print the version ID
      "AT S0",   // Printing of spaces off
      "AT E0",   // Echo off
      "AT L0",   // Linefeeds off
      "AT SP 6", // ISO 15765-4 CAN (11 bit ID, 500 kbit/s)
      "AT DP",
      "AT ST16",
      "ATCAF1",
      "ATAL",
      "ATFCSD300000",

      // --- Loop from here ---
      // BMS ECU (Header 704)
      "ATSH704",
      "ATCRA784",
      "ATFCSH704",
      "ATFCSM1",
      "221109", // State of Charge (SOC %)
      "221101", // HV Battery Voltage (V)
      "221103", // HV Battery Current (A)
      "221107", // Max Battery Temp (°C)
      "221108", // Min Battery Temp (°C)
      "221105", // Max Cell Voltage (V)
      "221106", // Min Cell Voltage (V)
      "22112D", // Charging Status
      "22110A", // State of Health (SOH %)
      "221118", // CLTC Range (km)
      "221130", // Charge Limit Setting (%)
      "221120", // Cumulative Charge
      "221121", // Cumulative Discharge
      "220101", // Odometer (km) [BMS-hosted 3-byte, avoids 7E0 echo bug]
      "220102", // 12V Aux Battery Voltage [BMS-hosted]
      "221122", // Cell Voltages (multi-frame)
      "221123", // Cell Temperatures (multi-frame)

      // VCU ECU (Header 7E0)
      "ATSH7E0",
      "ATCRA7E8",
      "ATFCSH7E0",
      "ATFCSM1",
      "220104", // Speed (km/h) & drive mode
      "220313", // Accelerator Pedal Position (%)
      "22031E", // VCU Display SoC (%)
      "220317", // Front Motor RPM
      "220318", // Rear Motor RPM
      "220319", // Front Motor Torque Request (Nm)
      "22031A", // Rear Motor Torque Request (Nm)
      "220321", // Brake Main Cylinder Pressure (bar)
      "220322", // DC Fast Charge Inlet Temp 1 (°C)
      "220323", // DC Fast Charge Inlet Temp 2 (°C)
      "220324", // AC Slow Charge Inlet Temp 1 (°C)
      "220325", // AC Slow Charge Inlet Temp 2 (°C)
      "220326", // AC Slow Charge Inlet Temp 3 (°C)
      "220327", // Motor Coolant Temp (°C)
      "220328", // Battery Coolant Temp (°C)
      "22031D", // Charger connected (HVIL)
  };

  float batteryKwh = 93.1; // default (Xpeng G9 93kWh)
  switch (liveData->settings.carType)
  {
  case CAR_XPENG_G6_66:
    batteryKwh = 66.0;
    break;
  case CAR_XPENG_G6_88:
    batteryKwh = 87.5;
    break;
  case CAR_XPENG_G9_78:
    batteryKwh = 78.2;
    break;
  case CAR_XPENG_G9_93:
    batteryKwh = 93.1;
    break;
  case CAR_XPENG_P7_60:
    batteryKwh = 60.2;
    break;
  case CAR_XPENG_P7_83:
    batteryKwh = 82.7;
    break;
  case CAR_XPENG_P7PLUS_75:
    batteryKwh = 74.9;
    break;
  case CAR_XPENG_P5_66:
    batteryKwh = 66.2;
    break;
  case CAR_XPENG_G3_66:
    batteryKwh = 66.5;
    break;
  case CAR_XPENG_X9_85:
    batteryKwh = 84.5;
    break;
  case CAR_XPENG_X9_101:
    batteryKwh = 101.5;
    break;
  default:
    batteryKwh = 93.1;
    break;
  }
  liveData->params.batteryTotalAvailableKWh = batteryKwh;
  liveData->params.batMaxEnergyContent = batteryKwh;
  liveData->params.cellCount = kXpengCellCount;
  liveData->params.batModuleTempCount = kXpengModuleTempCount;
  liveData->rxTimeoutMs = 1000;
  liveData->delayBetweenCommandsMs = 20;

  liveData->commandQueue.clear();
  for (auto cmd : commandQueueXpeng)
  {
    liveData->commandQueue.push_back({0, cmd});
  }

  liveData->commandQueueLoopFrom = 11;
  liveData->commandQueueCount = commandQueueXpeng.size();
}

/**
   parseRowMerged
*/
void CarXpeng::parseRowMerged()
{
  String response = liveData->responseRowMerged;
  response.trim();
  response.toUpperCase();
  if (response.length() < 6)
    return;

  String req = liveData->commandRequest;
  req.replace(" ", "");
  req.toUpperCase();
  String did = didFromRequest(req);
  if (did.length() == 0)
    return;

  // Expected positive response begins with 62 + DID
  int pos = response.indexOf("62" + did);
  if (pos < 0)
    return;

  int dataStart = pos + 6; // start of data bytes

  // ============================================
  // BMS PIDs (Header 704)
  // ============================================
  if (did == "1109") // SOC (%)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float soc = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 10.0f;
      if (inRange(soc, 0.0f, 100.0f))
      {
        liveData->params.socPercBms = soc;
        if (liveData->params.socPerc < 0)
          liveData->params.socPerc = soc;
      }
    }
  }
  else if (did == "110A") // SOH (%)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float soh = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 10.0f;
      if (inRange(soh, 0.0f, 100.0f))
      {
        liveData->params.sohPerc = soh;
      }
    }
  }
  else if (did == "1101") // HV Battery Voltage (V)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float v = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 10.0f;
      if (inRange(v, 200.0f, 1000.0f))
      {
        liveData->params.batVoltage = v;
        if (liveData->params.batPowerAmp != -1000)
        {
          liveData->params.batPowerKw = (liveData->params.batVoltage * liveData->params.batPowerAmp) / 1000.0f;
          updateChargeGraph(liveData);
        }
      }
    }
  }
  else if (did == "1103") // HV Battery Current (A)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      // Formula: [B4:B5] * 0.5 - 1600.0 (negative = charging, positive = discharging)
      float rawA = hexToDecPart(response, dataStart, dataStart + 4, 2, false) * 0.5f - 1600.0f;
      // In evDash: batPowerAmp > 0 is charging, < 0 is discharging
      liveData->params.batPowerAmp = -rawA;
      if (liveData->params.batVoltage > 0)
      {
        liveData->params.batPowerKw = (liveData->params.batVoltage * liveData->params.batPowerAmp) / 1000.0f;
        updateChargeGraph(liveData);
      }
    }
  }
  else if (did == "1105") // Max Cell Voltage (V)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float maxV = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 1000.0f;
      if (inRange(maxV, 2.0f, 5.0f))
      {
        liveData->params.batCellMaxV = maxV;
      }
    }
  }
  else if (did == "1106") // Min Cell Voltage (V)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float minV = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 1000.0f;
      if (inRange(minV, 2.0f, 5.0f))
      {
        liveData->params.batCellMinV = minV;
      }
    }
  }
  else if (did == "1107") // Max Battery Temp (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float maxT = hexToDecPart(response, dataStart, dataStart + 2, 1, false) - 40.0f;
      if (inRange(maxT, -40.0f, 100.0f))
      {
        liveData->params.batMaxC = maxT;
        if (liveData->params.batMinC > -90.0f)
          liveData->params.batTempC = (liveData->params.batMinC + liveData->params.batMaxC) / 2.0f;
        else
          liveData->params.batTempC = maxT;
      }
    }
  }
  else if (did == "1108") // Min Battery Temp (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float minT = hexToDecPart(response, dataStart, dataStart + 2, 1, false) - 40.0f;
      if (inRange(minT, -40.0f, 100.0f))
      {
        liveData->params.batMinC = minT;
        if (liveData->params.batMaxC > -90.0f)
          liveData->params.batTempC = (liveData->params.batMinC + liveData->params.batMaxC) / 2.0f;
        else
          liveData->params.batTempC = minT;
      }
    }
  }
  else if (did == "112D") // Charging Status (0/2/3/4)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      uint8_t chg = hexToDecPart(response, dataStart, dataStart + 2, 1, false);
      if (chg == 2 || chg == 4 || (liveData->params.batPowerAmp > 1.0f && liveData->params.speedKmh < 1.0f))
      {
        liveData->params.chargingOn = true;
        liveData->params.chargerDCconnected = (chg == 2 || chg == 4);
        liveData->params.chargerACconnected = (chg == 3);
        liveData->params.carMode = CAR_MODE_CHARGING;
      }
      else if (chg == 3)
      {
        liveData->params.chargingOn = true;
        liveData->params.chargerACconnected = true;
        liveData->params.chargerDCconnected = false;
        liveData->params.carMode = CAR_MODE_CHARGING;
      }
      else if (chg == 0 && (liveData->params.batPowerAmp <= 0.2f || liveData->params.speedKmh > 1.0f))
      {
        liveData->params.chargingOn = false;
        liveData->params.chargerDCconnected = false;
        liveData->params.chargerACconnected = false;
        liveData->params.chargerVoltage = 0;
        liveData->params.chargerCurrent = 0;
        liveData->params.carMode = (liveData->params.speedKmh > 1.0f) ? CAR_MODE_DRIVE : CAR_MODE_NONE;
      }
    }
  }
  else if (did == "1120") // Cumulative Charge
  {
    if (response.length() >= (uint16_t)(dataStart + 8))
    {
      double raw = hexToDecPart(response, dataStart, dataStart + 8, 4, false);
      if (raw > 0)
      {
        // If raw is Ah (typically < 100000), convert to kWh
        if (raw < 100000.0 && liveData->params.batVoltage > 200.0f)
          liveData->params.cumulativeEnergyChargedKWh = (raw * liveData->params.batVoltage) / 1000.0f;
        else
          liveData->params.cumulativeEnergyChargedKWh = raw / 10.0f;
      }
    }
  }
  else if (did == "1121") // Cumulative Discharge
  {
    if (response.length() >= (uint16_t)(dataStart + 8))
    {
      double raw = hexToDecPart(response, dataStart, dataStart + 8, 4, false);
      if (raw > 0)
      {
        if (raw < 100000.0 && liveData->params.batVoltage > 200.0f)
          liveData->params.cumulativeEnergyDischargedKWh = (raw * liveData->params.batVoltage) / 1000.0f;
        else
          liveData->params.cumulativeEnergyDischargedKWh = raw / 10.0f;
      }
    }
  }
  else if (did == "0101") // Odometer (km) - BMS 704 3-byte [B4:B6]
  {
    if (response.length() >= (uint16_t)(dataStart + 6))
    {
      double odo = hexToDecPart(response, dataStart, dataStart + 6, 3, false);
      if (odo > 0 && odo < 1500000)
      {
        liveData->params.odoKm = odo;
      }
    }
  }
  else if (did == "0102") // 12V Aux Battery Voltage - BMS 704
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float v = hexToDecPart(response, dataStart, dataStart + 2, 1, false) / 10.0f;
      if (response.length() >= (uint16_t)(dataStart + 4) && v > 50.0f)
      {
        v = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 100.0f;
      }
      if (inRange(v, 8.0f, 16.0f))
      {
        liveData->params.auxVoltage = v;
        updateAuxPercent(liveData, v);
      }
    }
  }
  else if (did == "1122") // Cell Voltages (multi-frame)
  {
    uint16_t cellIdx = 0;
    for (uint16_t i = dataStart; i + 3 < response.length() && cellIdx < sizeof(liveData->params.cellVoltage) / sizeof(liveData->params.cellVoltage[0]); i += 4)
    {
      float v = hexToDecPart(response, i, i + 4, 2, false) / 1000.0f;
      if (inRange(v, 2.0f, 5.0f))
      {
        liveData->params.cellVoltage[cellIdx++] = v;
      }
    }
    if (cellIdx > 0)
    {
      liveData->params.cellCount = cellIdx;
    }
  }
  else if (did == "1123") // Cell/Module Temperatures (multi-frame)
  {
    uint16_t modIdx = 0;
    for (uint16_t i = dataStart; i + 1 < response.length() && modIdx < sizeof(liveData->params.batModuleTempC) / sizeof(liveData->params.batModuleTempC[0]); i += 2)
    {
      float t = hexToDecPart(response, i, i + 2, 1, false) - 40.0f;
      if (inRange(t, -40.0f, 100.0f))
      {
        liveData->params.batModuleTempC[modIdx++] = t;
      }
    }
    if (modIdx > 0)
    {
      liveData->params.batModuleTempCount = modIdx;
    }
  }
  // ============================================
  // VCU PIDs (Header 7E0)
  // ============================================
  else if (did == "0104") // Speed (km/h)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float spd = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 100.0f;
      if (inRange(spd, 0.0f, 300.0f))
      {
        liveData->params.speedKmh = spd;
        if (spd > 1.0f)
        {
          liveData->params.forwardDriveMode = true;
          liveData->params.carMode = CAR_MODE_DRIVE;
          liveData->params.ignitionOn = true;
          liveData->params.lastIgnitionOnTime = liveData->params.currentTime;
          liveData->params.chargingOn = false;
          liveData->params.chargerDCconnected = false;
          liveData->params.chargerACconnected = false;
          liveData->params.chargerVoltage = 0;
          liveData->params.chargerCurrent = 0;
        }
      }
    }
  }
  else if (did == "031E") // VCU Display SoC (%)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float soc = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 10.0f;
      if (inRange(soc, 0.0f, 100.0f))
      {
        liveData->params.socPerc = soc;
      }
    }
  }
  else if (did == "0317") // Front Motor RPM
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float rpm = hexToDecPart(response, dataStart, dataStart + 4, 2, false) - 16000.0f;
      if (inRange(rpm, -20000.0f, 20000.0f))
        liveData->params.motor1Rpm = rpm;
    }
  }
  else if (did == "0318") // Rear Motor RPM
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float rpm = hexToDecPart(response, dataStart, dataStart + 4, 2, false) - 16000.0f;
      if (inRange(rpm, -20000.0f, 20000.0f))
      {
        liveData->params.motor2Rpm = rpm;
      }
    }
  }
  else if (did == "0319") // Front Motor Torque Request (Nm)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float tq = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 4.0f - 500.0f;
      if (inRange(tq, -1000.0f, 1000.0f))
      {
        liveData->params.motor1TorqueNm = tq;
      }
    }
  }
  else if (did == "031A") // Rear Motor Torque Request (Nm)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float tq = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 4.0f - 500.0f;
      if (inRange(tq, -1000.0f, 1000.0f))
      {
        liveData->params.motor2TorqueNm = tq;
      }
    }
  }
  else if (did == "0320") // DC Fast Charge Voltage (V)
  {
    if (response.length() >= (uint16_t)(dataStart + 4) && liveData->params.chargingOn && liveData->params.chargerDCconnected)
    {
      float v = hexToDecPart(response, dataStart, dataStart + 4, 2, false);
      if (inRange(v, 100.0f, 1000.0f))
      {
        liveData->params.chargerVoltage = v;
      }
    }
    else if (!liveData->params.chargingOn)
    {
      liveData->params.chargerVoltage = 0;
    }
  }
  else if (did == "031F") // DC Fast Charge Current (A)
  {
    if (response.length() >= (uint16_t)(dataStart + 4) && liveData->params.chargingOn && liveData->params.chargerDCconnected)
    {
      float a = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 10.0f - 1200.0f;
      if (inRange(a, 0.5f, 650.0f))
      {
        liveData->params.chargerCurrent = a;
      }
    }
    else if (!liveData->params.chargingOn)
    {
      liveData->params.chargerCurrent = 0;
    }
  }
  else if (did == "0322" || did == "0323") // Fast Charging Inlet Temp 1 & 2 (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float t = hexToDecPart(response, dataStart, dataStart + 2, 1, false) - 40.0f;
      if (inRange(t, -40.0f, 150.0f))
      {
        if (liveData->params.rapidChargePort < -90.0f || t > liveData->params.rapidChargePort)
          liveData->params.rapidChargePort = t;
        liveData->params.bmsUnknownTempA = liveData->params.rapidChargePort;
      }
    }
  }
  else if (did == "0324" || did == "0325") // Slow Charging Inlet Temp 1 & 2 (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float t = hexToDecPart(response, dataStart, dataStart + 2, 1, false) - 40.0f;
      if (inRange(t, -40.0f, 150.0f))
      {
        if (liveData->params.normalChargePort < -90.0f || t > liveData->params.normalChargePort)
          liveData->params.normalChargePort = t;
        liveData->params.bmsUnknownTempB = liveData->params.normalChargePort;
      }
    }
  }
  else if (did == "031D") // Charger Connected (HVIL)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      uint8_t conn = hexToDecPart(response, dataStart, dataStart + 2, 1, false);
      if (conn > 0)
      {
        if (liveData->params.batPowerAmp > 0.5f && liveData->params.speedKmh < 1.0f)
        {
          liveData->params.chargingOn = true;
          liveData->params.carMode = CAR_MODE_CHARGING;
        }
      }
      else if (conn == 0 && (liveData->params.batPowerAmp <= 0.2f || liveData->params.speedKmh > 1.0f))
      {
        liveData->params.chargingOn = false;
        liveData->params.chargerDCconnected = false;
        liveData->params.chargerACconnected = false;
        liveData->params.chargerVoltage = 0;
        liveData->params.chargerCurrent = 0;
      }
    }
  }
  else if (did == "0327") // Motor Temp (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float t = hexToDecPart(response, dataStart, dataStart + 2, 1, false) / 2.0f - 40.0f;
      if (inRange(t, -40.0f, 150.0f))
        liveData->params.motorTempC = t;
    }
  }
  else if (did == "0328") // Coolant Temp (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float t = hexToDecPart(response, dataStart, dataStart + 2, 1, false) / 2.0f - 40.0f;
      if (inRange(t, -40.0f, 100.0f))
      {
        liveData->params.coolantTemp1C = t;
        liveData->params.coolingWaterTempC = t;
      }
    }
  }
  else if (did == "1118") // CLTC Range (km)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float range = hexToDecPart(response, dataStart, dataStart + 4, 2, false);
      if (inRange(range, 0.0f, 1500.0f))
      {
        // Available for telemetry/display
      }
    }
  }
  else if (did == "1130") // Target Charge Limit Setting (%)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float limit = hexToDecPart(response, dataStart, dataStart + 4, 2, false) - 10.0f;
      if (inRange(limit, 50.0f, 100.0f))
      {
        // Target charge limit
      }
    }
  }
  else if (did == "0313") // Accelerator Pedal Position (%)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float posPct = hexToDecPart(response, dataStart, dataStart + 2, 1, false) / 2.0f;
      if (inRange(posPct, 0.0f, 100.0f))
      {
        // Accelerator position
      }
    }
  }
  else if (did == "0321") // Brake Main Cylinder Pressure (bar)
  {
    if (response.length() >= (uint16_t)(dataStart + 4))
    {
      float bar = hexToDecPart(response, dataStart, dataStart + 4, 2, false) / 5.0f;
      if (inRange(bar, 0.0f, 300.0f))
      {
        // Brake pressure
      }
    }
  }
  else if (did == "0326") // Slow Charging Inlet Temp 3 (°C)
  {
    if (response.length() >= (uint16_t)(dataStart + 2))
    {
      float t = hexToDecPart(response, dataStart, dataStart + 2, 1, false) - 40.0f;
      if (inRange(t, -40.0f, 150.0f))
      {
        if (liveData->params.normalChargePort < -90.0f || t > liveData->params.normalChargePort)
          liveData->params.normalChargePort = t;
      }
    }
  }
  else if (did == "F190") // VIN
  {
    char vin[18] = {0};
    uint8_t vinLen = 0;
    for (uint16_t i = dataStart; i + 1 < response.length() && vinLen < 17; i += 2)
    {
      char c = static_cast<char>(hexToDecPart(response, i, i + 2, 1, false));
      if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))
        vin[vinLen++] = c;
    }
    if (vinLen == 17)
    {
      strncpy(liveData->params.carVin, vin, sizeof(liveData->params.carVin) - 1);
      liveData->params.carVin[sizeof(liveData->params.carVin) - 1] = '\0';
    }
  }
}

/**
   loadTestData
*/
void CarXpeng::loadTestData()
{
  liveData->params.batteryTotalAvailableKWh = 93.0;
  liveData->params.cellCount = kXpengCellCount;
  liveData->params.batModuleTempCount = kXpengModuleTempCount;

  liveData->commandRequest = "221109";
  liveData->responseRowMerged = "6211090384"; // 90.0% SOC
  parseRowMerged();

  liveData->commandRequest = "22110A";
  liveData->responseRowMerged = "62110A03E8"; // 100.0% SOH
  parseRowMerged();

  liveData->commandRequest = "221101";
  liveData->responseRowMerged = "6211011964"; // 650.0V HV Voltage
  parseRowMerged();

  liveData->commandRequest = "221103";
  liveData->responseRowMerged = "6211030C80"; // 3200 * 0.5 - 1600 = 0A
  parseRowMerged();

  liveData->commandRequest = "221105";
  liveData->responseRowMerged = "6211050E74"; // 3.700V Max Cell
  parseRowMerged();

  liveData->commandRequest = "221106";
  liveData->responseRowMerged = "6211060E6A"; // 3.690V Min Cell
  parseRowMerged();

  liveData->commandRequest = "221107";
  liveData->responseRowMerged = "62110741"; // 65 - 40 = 25C Max Temp
  parseRowMerged();

  liveData->commandRequest = "221108";
  liveData->responseRowMerged = "6211083F"; // 63 - 40 = 23C Min Temp
  parseRowMerged();

  liveData->commandRequest = "220104";
  liveData->responseRowMerged = "6201041388"; // 50.00 km/h
  parseRowMerged();

  liveData->commandRequest = "22031E";
  liveData->responseRowMerged = "62031E0384"; // 90.0% Display SOC
  parseRowMerged();

  liveData->commandRequest = "220317";
  liveData->responseRowMerged = "6203174268"; // 17000 RPM (1000 net front)
  parseRowMerged();

  liveData->commandRequest = "220318";
  liveData->responseRowMerged = "6203184650"; // 18000 RPM (2000 net rear)
  parseRowMerged();

  liveData->commandRequest = "220319";
  liveData->responseRowMerged = "62031908FC"; // 2300/4 - 500 = 75 Nm front torque
  parseRowMerged();

  liveData->commandRequest = "22031A";
  liveData->responseRowMerged = "62031A0A28"; // 2600/4 - 500 = 150 Nm rear torque
  parseRowMerged();

  liveData->commandRequest = "220322";
  liveData->responseRowMerged = "6203224B"; // 75 - 40 = 35 C fast charge port
  parseRowMerged();

  liveData->commandRequest = "220324";
  liveData->responseRowMerged = "6203243C"; // 60 - 40 = 20 C slow charge port
  parseRowMerged();

  liveData->commandRequest = "220101";
  liveData->responseRowMerged = "620101002710"; // 10,000 km Odo
  parseRowMerged();

  liveData->commandRequest = "220102";
  liveData->responseRowMerged = "6201020087"; // 13.5V Aux
  parseRowMerged();

  liveData->params.tireFrontLeftPressureBar = 2.5;
  liveData->params.tireFrontRightPressureBar = 2.5;
  liveData->params.tireRearLeftPressureBar = 2.5;
  liveData->params.tireRearRightPressureBar = 2.5;
  liveData->params.tireFrontLeftTempC = 25;
  liveData->params.tireFrontRightTempC = 25;
  liveData->params.tireRearLeftTempC = 25;
  liveData->params.tireRearRightTempC = 25;

  for (int i = 0; i < kXpengCellCount; i++)
  {
    liveData->params.cellVoltage[i] = 3.69f + ((i % 10) * 0.001f);
  }

  for (int i = 0; i < kXpengModuleTempCount; i++)
  {
    liveData->params.batModuleTempC[i] = 24.0f + (i % 3);
  }

  for (int i = 10; i <= 80; i++)
  {
    float kw = (i < 40) ? (480.0f - (i * 2.0f)) : (400.0f - ((i - 40) * 6.0f));
    liveData->params.chargingGraphMinKw[i] = kw - 15.0f;
    liveData->params.chargingGraphMaxKw[i] = kw;
    liveData->params.chargingGraphBatMinTempC[i] = 23.0f + (i * 0.15f);
    liveData->params.chargingGraphBatMaxTempC[i] = 26.0f + (i * 0.18f);
    liveData->params.chargingGraphWaterCoolantTempC[i] = 22.0f + (i * 0.1f);
  }
}
