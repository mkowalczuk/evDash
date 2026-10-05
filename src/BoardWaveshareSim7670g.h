#pragma once

#include "BoardCore.h"
#include "Sim7670G.h"
#include "ModemTransport.h"
#include <SD_MMC.h>
#include <SPI.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>


/**
 * Waveshare ESP32-S3-SIM7670G-4G, run headless.
 *
 * This board has no screen fitted: the LCD is an external ST7789 wired to a
 * header and is left unpopulated, so there is nothing to draw on. It carries
 * an ESP32-S3, a SIM7670G LTE modem, a TF card slot and a MAX17048 fuel gauge.
 *
 * Everything display-independent - the main loop, WiFi, telemetry upload, GPS
 * and SD recording - comes from BoardCore unchanged. Each capability this board
 * does not implement keeps the no-op default declared there, so this class only
 * has to describe what is genuinely different about the hardware.
 *
 * The pin assignments live in config.h, taken from the V2 schematic and the
 * vendor's ESP-IDF demos, which agree on all of them:
 *   SIM7670G AT UART  RX GPIO17, TX GPIO18, UART1, 115200 8N1
 *   TF card (SDMMC)   CLK GPIO5, CMD GPIO4, DATA GPIO6, card detect GPIO46
 *   MAX17048 gauge    SDA GPIO15, SCL GPIO16
 *   WS2812B RGB       GPIO38 (unused for now)
 *
 * Each is overridable from platformio.ini, so a differently wired carrier needs
 * no code change.
 *
 * The modem's power rail is switched by a DIP switch on the carrier, not by an
 * ESP32 GPIO, so there is no enable pin to drive here.
 */
class BoardWaveshareSim7670g : public BoardCore
{
public:
  BoardWaveshareSim7670g() : modemTransport(&modem) {}

  const char *hardwareModelName() override { return "Sim7670G"; }
  uint8_t hardwareIdTag() const override { return 0x04U; }
  bool sdBegin() override;
  void initBoard() override;
  void commLoop() override;
  void boardLoop() override;
  void afterSetup() override;
  void initGPS() override;
  void showGps() override;
  void modemInfo() override;
  void modemReset() override;
  void modemTest() override;
  NetTransport *activeTransport() override;

private:
  // Opens the AT UART and hands it to the modem driver. The driver probes the
  // modem from the main loop, so boot does not wait for it to answer.
  void modemBegin();

  // The AT UART carries GNSS NMEA as well as command replies, so the driver
  // owns it and forwards the NMEA lines to the GPS parser. It is deliberately
  // not gpsHwUart.
  HardwareSerial *modemUart = nullptr;
  Sim7670G modem;
  ModemTransport modemTransport;
  bool gpsFed = false;
  SFE_MAX1704X max17048;
};