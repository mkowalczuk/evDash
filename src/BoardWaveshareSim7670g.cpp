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

// NMEA bursts and command replies share the UART, and the main loop can be busy
// for a while between reads, so the default 256-byte receive buffer is too small.
static constexpr size_t kModemRxBufferSize = 2048;

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

  // The driver always runs, because the modem has to be brought up whether or
  // not GNSS is being consumed right now. It forwards NMEA through the sink.
  modem.loop();

  const bool allowGps = !(liveData->params.stopCommandQueue && liveData->settings.voltmeterEnabled == 1);
  if (gpsFed && allowGps)
  {
    gpsFed = false;
    syncGPS();
  }
}

void BoardWaveshareSim7670g::initGPS()
{
  if (modemUart == nullptr)
  {
    modemBegin();
    return;
  }

  // Queued rather than awaited. If the modem is not up yet these time out, and
  // the driver's ready callback runs this again once it answers.
  syslog->println("SIM7670G: enabling GNSS...");
  modem.send("AT+CGNSSPWR=1", 1000);
  modem.send("AT+CGNSSTST=1", 1000);
  modem.send("AT+CGNSSPORTSWITCH=0,1", 1000);
}

void BoardWaveshareSim7670g::showGps()
{
  BoardCore::showGps();
  syslog->printf("Modem Hardware:   SIM7670G on UART%u (GPIO%d RX, GPIO%d TX, %d baud)\n",
                 kModemUartNum, kModemRxPin, kModemTxPin, kModemBaud);
  syslog->printf("Modem Responding: %s\n", modem.responding() ? "YES" : "NO");
}

static const char *registrationText(int stat)
{
  switch (stat)
  {
  case 0: return "not registered, not searching";
  case 1: return "registered, home network";
  case 2: return "searching for a network";
  case 3: return "registration denied";
  case 4: return "unknown";
  case 5: return "registered, roaming";
  default: return "not queried yet";
  }
}

void BoardWaveshareSim7670g::modemInfo()
{
  const Sim7670G::Info &info = modem.info();
  syslog->println(".-[ Cellular Modem ]-_.");
  syslog->printf("Hardware:      SIM7670G on UART%u (GPIO%d RX, GPIO%d TX, %d baud)\n",
                 kModemUartNum, kModemRxPin, kModemTxPin, kModemBaud);
  syslog->printf("Responding:    %s\n", modem.responding() ? "YES" : "NO (check power DIP switch and antenna)");
  syslog->printf("State:         %s\n", modem.stateName());
  syslog->printf("Data enabled:  %s\n", (liveData->settings.modemEnabled == 1) ? "YES" : "NO (run 'modemEnabled=1')");
  syslog->printf("IMEI:          %s\n", info.imei.length() > 0 ? info.imei.c_str() : "unknown");
  if (liveData->settings.modemEnabled == 1)
  {
    syslog->printf("SIM:           %s\n", info.simReady ? "ready" : "not ready");
    syslog->printf("Registration:  %s\n", registrationText(info.registration));
  }
  else
  {
    syslog->println("SIM:           disabled (cellular off)");
    syslog->println("Registration:  disabled (cellular off)");
  }
  syslog->printf("Operator:      %s\n", info.operatorName.length() > 0 ? info.operatorName.c_str() : "unknown");
  if (info.rssiDbm != 0)
  {
    syslog->printf("Signal:        %d dBm\n", info.rssiDbm);
  }
  else
  {
    syslog->println("Signal:        unknown");
  }
  syslog->printf("APN:           %s\n", liveData->settings.modemApn[0] != '\0' ? liveData->settings.modemApn : "(not set, use 'modem=apn=<apn>')");
  if (strlen(liveData->settings.modemApnUser) > 0 || liveData->settings.modemApnAuth > 0)
  {
    const char *authStr = (liveData->settings.modemApnAuth == 1) ? "PAP" :
                          (liveData->settings.modemApnAuth == 2) ? "CHAP" :
                          (liveData->settings.modemApnAuth == 3) ? "PAP/CHAP" : "None";
    syslog->printf("APN User/Auth: %s (auth: %s)\n",
                   (strlen(liveData->settings.modemApnUser) > 0) ? liveData->settings.modemApnUser : "(none)",
                   authStr);
  }
  syslog->printf("Roaming:       %s\n", (liveData->settings.modemRoaming == 1) ? "allowed" : "disabled (home only)");
  const char *policyStr = "WiFi preferred";
  switch (liveData->settings.modemTransportPolicy)
  {
  case 1: policyStr = "Cellular preferred"; break;
  case 2: policyStr = "Cellular only"; break;
  case 3: policyStr = "WiFi only"; break;
  default: break;
  }
  syslog->printf("Policy:        %s\n", policyStr);
  syslog->printf("Data saver:    %s\n", (liveData->settings.modemDataSaver == 1) ? "ON (reduced cellular usage)" : "OFF");
  syslog->printf("TLS Insecure:  %s\n", (liveData->settings.modemTlsInsecure == 1) ? "YES (skip cert verification)" : "NO");
  syslog->printf("IP address:    %s\n", info.ipAddress.length() > 0 ? info.ipAddress.c_str() : "none");
  syslog->printf("AT commands:   %lu sent, %lu timed out\n",
                 static_cast<unsigned long>(modem.commandCount()), static_cast<unsigned long>(modem.timeoutCount()));
}

void BoardWaveshareSim7670g::modemReset()
{
  syslog->println("Restarting the modem state machine...");
  modem.reset();
}

void BoardWaveshareSim7670g::modemTest()
{
  syslog->println("Sending AT to the modem...");
  modem.send("AT", 2000, [](bool ok, const String &response)
             {
               syslog->printf("modem test: %s%s%s\n", ok ? "OK" : "FAILED", response.length() > 0 ? " - " : "", response.c_str());
             });
}

void BoardWaveshareSim7670g::modemBegin()
{
  modemUart = new HardwareSerial(kModemUartNum);
  modemUart->setRxBufferSize(kModemRxBufferSize);
  modemUart->begin(kModemBaud, SERIAL_8N1, kModemRxPin, kModemTxPin);

  modem.setNmeaSink([this](char ch)
                    {
                      syslog->infoNolf(DEBUG_GPS, ch);
                      if (gps.encode(ch))
                      {
                        gpsFed = true;
                      }
                    });
  modem.setReadyCallback([this]()
                         {
                           syslog->println("SIM7670G: AT UART responding");
                           initGPS();
                         });
  modem.begin(modemUart, liveData);
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

NetTransport *BoardWaveshareSim7670g::activeTransport()
{
  if (liveData == nullptr)
  {
    return &wifiTransport;
  }

  switch (liveData->settings.modemTransportPolicy)
  {
  case 1: // Cellular preferred
    if (modemTransport.ready())
    {
      return &modemTransport;
    }
    if (wifiTransport.ready())
    {
      return &wifiTransport;
    }
    return &modemTransport;

  case 2: // Cellular only
    return &modemTransport;

  case 3: // WiFi only
    return &wifiTransport;

  case 0: // WiFi preferred (default)
  default:
    if (wifiTransport.ready())
    {
      return &wifiTransport;
    }
    if (modemTransport.ready())
    {
      return &modemTransport;
    }
    if (liveData->settings.wifiEnabled == 1)
    {
      return &wifiTransport;
    }
    return &modemTransport;
  }
}

#endif // BOARD_WAVESHARE_SIM7670G
