#ifdef BOARD_WAVESHARE_SIM7670G
#include "BoardWaveshareSim7670g.h"
#include "LogSerial.h"
#include "config.h"
#include <Wire.h>

// SIM7670G AT UART on UART1. The pins come from config.h, which takes them
// from platformio.ini or falls back to the carrier's documented wiring.
static constexpr int kModemRxPin = SIM7670G_RX_PIN;
static constexpr int kModemTxPin = SIM7670G_TX_PIN;
static constexpr int kModemBaud = 115200;
static constexpr uint8_t kModemUartNum = 1;

static constexpr uint32_t kModemProbeTimeoutMs = 4000;

void BoardWaveshareSim7670g::initBoard()
{
  liveData->params.booting = true;

  // No battery-backed clock on this board, so rtcReadTime() returns 0 and SNTP
  // establishes the time shortly after WiFi comes up.
  seedSystemClock();

  modemBegin();

  if (liveData->settings.voltmeterEnabled == 1)
  {
    Wire.begin(BAT_SDA_PIN, BAT_SCL_PIN);
    Wire.setTimeout(1000);
    if (max17048.begin(Wire))
    {
      syslog->println("MAX17048 initialized");
    }
    else
    {
      syslog->println("MAX17048 not found");
    }
  }


}

void BoardWaveshareSim7670g::afterSetup()
{
  BoardInterface::afterSetup();

  // WiFi - started AFTER BLE so both subsystems are registered with coex
  if (!liveData->params.wifiApMode && liveData->settings.wifiEnabled == 1)
  {
    wifiSetup();
  }
}

void BoardWaveshareSim7670g::commLoop()
{
  if (commInterface == nullptr || liveData->params.stopCommandQueue || commInterface->isSuspended())
  {
    return;
  }
  commInterface->mainLoop();
}

void BoardWaveshareSim7670g::boardLoop()
{
  if (modemUart == nullptr)
  {
    return;
  }

  const bool allowGps = !(liveData->params.stopCommandQueue && liveData->settings.voltmeterEnabled == 1);
  if (!allowGps)
  {
    return;
  }

  if (modemUart->available())
  {
    do
    {
      int ch = modemUart->read();
      if (ch != -1)
      {
        syslog->infoNolf(DEBUG_GPS, char(ch));
        gps.encode(ch);
      }
    } while (modemUart->available());
    syncGPS();
  }
}

void BoardWaveshareSim7670g::initGPS()
{
  if (!modemReady && modemUart == nullptr)
  {
    modemBegin();
    return;
  }

  syslog->println("SIM7670G: enabling GNSS...");
  modemSendCommand("AT+CGNSSPWR=1", 1000);
  delay(50);
  modemSendCommand("AT+CGNSSTST=1", 1000);
  delay(50);
  modemSendCommand("AT+CGNSSPORTSWITCH=0,1", 1000);
}

void BoardWaveshareSim7670g::showGps()
{
  BoardCore::showGps();
  syslog->printf("Modem Hardware:   SIM7670G on UART%u (GPIO%d RX, GPIO%d TX, %d baud)\n",
                 kModemUartNum, kModemRxPin, kModemTxPin, kModemBaud);
  syslog->printf("Modem Responding: %s\n", modemReady ? "YES" : "NO");
}

/**
 * Send one AT command and wait for the modem to answer.
 *
 * The reply is only checked for the "OK" terminator. Reading the response body
 * properly needs the AT parser, which arrives with the cellular work; until then
 * this is enough to tell an attached, powered modem from an unpowered carrier
 * or a wrong pin assignment.
 */
bool BoardWaveshareSim7670g::modemSendCommand(const char *command, uint32_t timeoutMs)
{
  modemUart->print(command);
  modemUart->print("\r\n");

  String reply;
  const uint32_t startMs = millis();
  while (millis() - startMs < timeoutMs)
  {
    while (modemUart->available())
    {
      const char ch = (char)modemUart->read();
      if (ch == '\r' || ch == '\n')
      {
        if (reply.endsWith("OK") || reply.indexOf("OK") >= 0)
        {
          return true;
        }
        if (reply.indexOf("ERROR") >= 0 || reply.indexOf("FAIL") >= 0)
        {
          return false;
        }
        if (reply.length() > 0)
        {
          reply.clear();
        }
        continue;
      }
      reply += ch;
      if (reply.length() > 256)
      {
        reply.remove(0, reply.length() - 256);
      }
    }
    delay(10);
  }
  return reply.indexOf("OK") >= 0;
}


bool BoardWaveshareSim7670g::modemBegin()
{
  modemUart = new HardwareSerial(kModemUartNum);
  modemUart->begin(kModemBaud, SERIAL_8N1, kModemRxPin, kModemTxPin);
  delay(300);
  while (modemUart->available())
  {
    modemUart->read();
  }

  // A bare "AT" is the cheapest way to see whether the modem is alive.
  modemReady = modemSendCommand("AT", kModemProbeTimeoutMs);

  if (modemReady)
  {
    syslog->println("SIM7670G: AT UART responding");
    initGPS();
  }
  else
  {
    syslog->println("SIM7670G: no response on the AT UART (GPIO17/18, 115200)");
  }
  return modemReady;
}
bool BoardWaveshareSim7670g::sdBegin()
{
  // SDMMC on ESP32-S3: CLK 5, CMD 4, DATA 6 (1-bit mode by default)
  // Be defensive: if pins can't be set or card absent, just fail gracefully
  if (!SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_DATA_PIN))
  {
    return false;
  }
  bool ok = SD_MMC.begin("/sdcard", false);
  if (!ok)
  {
    return false;
  }
  return true;
}

#endif // BOARD_WAVESHARE_SIM7670G
