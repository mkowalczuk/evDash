#include "BoardWaveshareSim7670g.h"
#include "LogSerial.h"
#include "config.h"
#include "BoardShared.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>

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

  if (liveData->settings.statusLedEnabled == 1)
  {
    statusLed.begin(WS2812B_PIN,
                    static_cast<uint8_t>((uint16_t)liveData->settings.statusLedBrightness * 255 / 100));
  }
  else
  {
    statusLed.setEnabled(false);
  }

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

  checkPinConflicts();
}

void BoardWaveshareSim7670g::afterSetup()
{
  BoardInterface::afterSetup();

  // SD card (TF slot over SD_MMC). Mount if enabled, then auto-start recording
  // and/or console logging per settings — the headless equivalent of the M5
  // boot path.
  if (liveData->settings.sdcardEnabled == 1)
  {
    if (sdcardMount() && liveData->settings.sdcardAutstartLog == 1)
    {
      syslog->println("Toggle recording on SD card");
      sdcardToggleRecording();
    }
  }

  // Apply the cellular data-connection gate based on policy + modemHoldData,
  // then set up preferred/fallback failover. The modem (and GNSS) stay powered
  // regardless; only the cellular DATA path may be withheld.
  applyModemDataGate();
  setupFailover();
}

/**
 * Decide whether the modem should open its data connection, based on transport
 * policy and the modemHoldData option. The modem and GNSS always stay on; this
 * only governs the cellular DATA path (AT+NETOPEN).
 *
 *   policy 2 (cellular only)        -> data always on
 *   policy 3 (WiFi only)            -> data held off (saves cellular), GNSS stays on
 *   policy 0/1 with modemHoldData=0 -> data on (default; fast failover)
 *   policy 0/1 with modemHoldData=1 -> data held off until needed as fallback
 */
void BoardWaveshareSim7670g::applyModemDataGate()
{
  const uint8_t policy = liveData->settings.modemTransportPolicy;
  bool dataEnabled;
  switch (policy)
  {
  case 2: dataEnabled = true; break;                                  // cellular only
  case 3: dataEnabled = false; break;                                 // WiFi only (GNSS still on)
  default: dataEnabled = (liveData->settings.modemHoldData == 0); break; // 0/1
  }
  modem.setDataConnectionEnabled(dataEnabled);
}

/**
 * Configure the reusable failover manager for policies 0 and 1. Policies 2/3
 * are single-link and leave the manager inactive (activeTransport falls back to
 * readiness checks).
 */
void BoardWaveshareSim7670g::setupFailover()
{
  const uint8_t policy = liveData->settings.modemTransportPolicy;
  const bool wifiUsable = (!liveData->params.wifiApMode && liveData->settings.wifiEnabled == 1);

  failoverActive = false;

  if (policy == 3)
  {
    // WiFi only: start WiFi, no cellular.
    if (wifiUsable) wifiSetup();
    return;
  }
  if (policy == 2)
  {
    // Cellular only: nothing to start for WiFi.
    return;
  }

  NetFailoverManager::Config cfg;
  cfg.probeIntervalMs = 60000;        // ping every 60s per spec
  cfg.fallbackRecheckStartMs = 60000;
  cfg.fallbackRecheckMaxMs = 300000;
  cfg.failureThreshold = 2;

  NetFailoverManager::Hooks hooks;
  hooks.log = [this](const String &s) { syslog->info(DEBUG_NET, s); };

  if (policy == 0)
  {
    // WiFi preferred, cellular fallback.
    if (wifiUsable) wifiSetup();
    hooks.preferredReady = [this]() { return WiFi.status() == WL_CONNECTED; };
    hooks.fallbackReady = [this]() { return modem.dataReady(); };
    hooks.probePreferred = [this]() { return wifiPing("1.1.1.1"); };
    hooks.probeFallback = [this]() { return modemReachable(); };
    hooks.bringUpFallback = [this]() { modem.setDataConnectionEnabled(true); };
    hooks.bringUpPreferred = [this]() { if (!liveData->params.wifiApMode) wifiSetup(); };
    hooks.advanceFallback = nullptr; // cellular has no alternates
    // Preferred is WiFi: rotate to the next configured SSID when it can't connect,
    // so a bad/absent primary AP doesn't pin us on cellular forever.
    hooks.onPreferredRetry = [this]() { return advanceWifiSsid(); };
    // On confirmed WiFi, optionally release cellular data (only if holding).
    hooks.onPreferredConfirmed = [this]() {
      if (liveData->settings.modemHoldData == 1)
        modem.setDataConnectionEnabled(false);
    };
  }
  else // policy == 1: cellular preferred, WiFi fallback
  {
    hooks.preferredReady = [this]() { return modem.dataReady(); };
    hooks.fallbackReady = [this]() { return WiFi.status() == WL_CONNECTED; };
    hooks.probePreferred = [this]() { return modemReachable(); };
    hooks.probeFallback = [this]() { return wifiPing("1.1.1.1"); };
    hooks.bringUpFallback = [this]() { if (!liveData->params.wifiApMode) wifiSetup(); };
    hooks.bringUpPreferred = [this]() { modem.setDataConnectionEnabled(true); };
    hooks.advanceFallback = [this]() { return advanceWifiSsid(); };
    hooks.onPreferredRetry = nullptr;    // cellular preferred: nothing to rotate
    hooks.onPreferredConfirmed = nullptr; // WiFi fallback left running (cheap); no teardown
  }

  failover.begin(cfg, hooks);
  failoverActive = true;
}

/**
 * Cycle to the next configured WiFi SSID (for the WiFi fallback). Returns false
 * when there is only one SSID or none configured.
 */
bool BoardWaveshareSim7670g::advanceWifiSsid()
{
  const bool have2 = liveData->settings.wifiSsid2[0] != '\0' && strcmp(liveData->settings.wifiSsid2, "empty") != 0;
  const bool have3 = liveData->settings.wifiSsid3[0] != '\0' && strcmp(liveData->settings.wifiSsid3, "empty") != 0;
  const bool have4 = liveData->settings.wifiSsid4[0] != '\0' && strcmp(liveData->settings.wifiSsid4, "empty") != 0;
  const uint8_t count = 1 + (have2 ? 1 : 0) + (have3 ? 1 : 0) + (have4 ? 1 : 0);
  if (count <= 1)
  {
    return false;
  }
  const uint8_t next = (liveData->params.wifiActiveIndex + 1) % count;
  wifiSwitchToIndex(next);
  return true;
}

/**
 * Lightweight WiFi reachability probe. ESP-IDF's esp_ping is callback/session
 * based and awkward to use synchronously; a short TCP connect to the anchor
 * host on 443 is a dependency-free, equally decisive "is the internet
 * reachable over WiFi" check. Returns true if the connection is established.
 */
bool BoardWaveshareSim7670g::wifiPing(const char *host, uint32_t timeoutMs)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    syslog->info(DEBUG_NET, "NET: wifi ping skipped (WiFi not connected)");
    return false;
  }
  WiFiClient c;
  c.setTimeout(timeoutMs / 1000 > 0 ? timeoutMs / 1000 : 1);
  const bool ok = c.connect(host, 443, timeoutMs);
  c.stop();
  syslog->info(DEBUG_NET, String("NET: wifi ping ") + host + (ok ? " OK (reply)" : " FAILED (no reply)"));
  return ok;
}

/**
 * Cellular reachability via a DNS resolve over the modem (AT+CDNSGIP). This is
 * fast, reliable on this firmware, and proves the data link is passing traffic.
 *
 * NOTE: AT+CPING is deliberately NOT used. Its minimum <wait_time> is 10000 ms
 * (per the SIM767XX manual), so a failed ping's +CPING result URC arrives up to
 * 10 s later and interleaves with unrelated AT traffic (e.g. CCHSEND prompts),
 * desynchronising the shared AT UART and cascading into command timeouts. A DNS
 * resolve gives the same up/down signal without that latency landmine.
 */
bool BoardWaveshareSim7670g::modemReachable()
{
  if (!modem.dataReady())
  {
    return false;
  }
  IPAddress ip;
  const bool dnsOk = modem.resolveHost("one.one.one.one", ip);
  syslog->info(DEBUG_NET, String("NET: modem reachability (DNS) ") + (dnsOk ? "OK" : "FAILED"));
  return dnsOk;
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

  if (failoverActive)
  {
    failover.loop();
  }

  const bool allowGps = !(liveData->params.stopCommandQueue && liveData->settings.voltmeterEnabled == 1);
  if (gpsFed && allowGps)
  {
    gpsFed = false;
    syncGPS();
  }

  // AGPS (AT+CGNSSAGPS) downloads assist data over the cellular PDP context to
  // shorten time-to-first-fix, but it only works once the data connection is
  // open. initGPS() issues it at GNSS power-up, which is usually before data is
  // up, so re-issue it the moment the data link becomes ready. Re-arm on a data
  // drop so a reconnect assists again. Costs one AT command per data (re)connect.
  const bool dataReady = modem.dataReady();
  if (dataReady && !agpsRequestedWithData)
  {
    syslog->info(DEBUG_GPS, "SIM7670G: data up, requesting AGPS assist");
    modem.send("AT+CGNSSAGPS=1", 1000);
    agpsRequestedWithData = true;
  }
  else if (!dataReady && agpsRequestedWithData)
  {
    agpsRequestedWithData = false; // re-arm for the next data reconnect
  }

  updateStatusLed();
}

void BoardWaveshareSim7670g::updateStatusLed()
{
  // Push live settings each frame so console changes to statusLedEnabled /
  // statusLedBrightness take effect immediately (no reboot needed).
  statusLed.setEnabled(liveData->settings.statusLedEnabled == 1);
  statusLed.setBrightness(
      static_cast<uint8_t>((uint16_t)liveData->settings.statusLedBrightness * 255 / 100));

  const bool wifiUp = (WiFi.status() == WL_CONNECTED);
  const bool modemDataUp = modem.dataReady();
  const bool anyNet = wifiUp || modemDataUp;

  // An upload error counts while it is recent and no later success has cleared
  // it (lastUploadErrorTime is reset to 0 on the next successful send).
  const bool uploadError =
      (liveData->params.lastUploadErrorTime != 0 &&
       liveData->params.currentTime - liveData->params.lastUploadErrorTime <= 30);

  StatusLed::DeviceStatus s;
  s.netConnected = anyNet;                           // green
  s.commConnecting = !liveData->commConnected;       // blue: car link not up / retrying
  s.error = (!anyNet) || uploadError;                // red: offline or upload/auth error
  s.noGpsFix = !isGpsFixUsable(liveData);            // yellow: no GPS fix

  statusLed.update(s);
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
  // Use all available constellations (GPS+GLONASS+BeiDou+Galileo). More birds in
  // view is the most reliable way to cut time-to-first-fix; the default may be
  // GPS-only. Value 15 = all four systems.
  modem.send("AT+CGNSSMODE=15", 1000);
  // Best-effort AGPS at power-up: downloads assist data over the PDP context to
  // shorten a cold fix. This early call usually happens before the data link is
  // open (so it may be a no-op here); boardLoop() re-issues it once the data
  // connection is actually ready, which is when the assist download can succeed.
  modem.send("AT+CGNSSAGPS=1", 1000);
  modem.send("AT+CGNSSTST=1", 1000);
  modem.send("AT+CGNSSPORTSWITCH=0,1", 1000);
}

String BoardWaveshareSim7670g::modemStatusText()
{
  if (!modem.responding())
  {
    return "not responding";
  }
  if (liveData->settings.modemEnabled != 1)
  {
    return "off (data disabled)";
  }
  if (modem.dataReady())
  {
    String s = "data ready";
    const int rssi = modem.info().rssiDbm;
    if (rssi != 0)
      s += String(" (") + rssi + " dBm)";
    return s;
  }
  // Responding but no data link yet — report the driver's state machine phase.
  return String(modem.stateName());
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
                          (liveData->settings.modemApnAuth == 2) ? "CHAP" : "None";
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

void BoardWaveshareSim7670g::modemFirmwareInfo()
{
  // AT+SIMCOMATI returns a multi-line identification block (manufacturer, model,
  // revision/firmware, IMEI, etc.). Use the blocking sendSync so the whole reply
  // is captured and printed verbatim to the console.
  syslog->println(".-[ Modem Firmware (AT+SIMCOMATI) ]-_.");
  String response;
  const bool ok = modem.sendSync("AT+SIMCOMATI", 3000, &response);
  if (!ok && response.length() == 0)
  {
    syslog->println("Modem did not respond (check power DIP switch / antenna).");
    return;
  }
  response.trim();
  syslog->println(response);
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
  // Keep latency-sensitive work alive while the driver blocks on a slow AT
  // command (TLS connect, DNS, COPS, MQTT I/O). The hook runs from inside the
  // driver's wait loops, so it must do only work that cannot re-enter the modem:
  //   - NOT the serial console: a console command (e.g. testMqtt) could call
  //     back into the modem synchronously and corrupt the in-flight AT state.
  //   - GPS folding is safe: syncGPS() only consumes bytes the NMEA sink already
  //     parsed locally and never touches the modem UART.
  // Then yield the CPU so FreeRTOS/ESP-IDF (WiFi/BLE) keep running. The console
  // is serviced normally on the next main-loop pass once the wait returns.
  modem.setYieldHook([this]()
                     {
                       if (gpsFed)
                       {
                         gpsFed = false;
                         syncGPS();
                       }
                       vTaskDelay(0); // relinquish to FreeRTOS/ESP-IDF (WiFi/BLE)
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
  // 1-bit mode: setPins() above wires only CLK/CMD/D0, so begin() must request
  // 1-bit (mode1bit=true). Passing false asks for 4-bit (D1-D3), which were
  // never assigned, and the mount fails.
  bool ok = SD_MMC.begin("/sdcard", true);
  if (!ok)
  {
    return false;
  }
  return true;
}

/**
 * Mount the TF card over SD_MMC and bring up SD-backed features. Mirrors the
 * M5 Board320_240::sdcardMount() but uses the SD_MMC backend this board wires
 * the card to, and points the console logger at the same filesystem.
 */
bool BoardWaveshareSim7670g::sdcardMount()
{
  if (liveData->params.sdcardInit)
  {
    return true;
  }

  syslog->print("Initializing SD card (SD_MMC)...");
  if (!sdBegin())
  {
    syslog->println(" failed");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    syslog->println(" no card attached");
    return false;
  }

  liveData->params.sdcardInit = true;
  const char *typeStr = (cardType == CARD_MMC) ? "MMC" : (cardType == CARD_SD) ? "SDSC" : (cardType == CARD_SDHC) ? "SDHC" : "UNKNOWN";
  syslog->printf(" found (%s, %lluMB)\n", typeStr, SD_MMC.cardSize() / (1024ULL * 1024ULL));

  // Route the console logger to this card, then start console logging if enabled.
  syslog->setSdFilesystem(SD_MMC);
  if (liveData->settings.sdcardConsoleLogEnabled == 1 && !syslog->isSdLogging())
  {
    startSdConsoleLog();
  }
  return true;
}

/**
 * Toggle SD telemetry recording on/off, flushing any buffered data on stop.
 * Mirrors the M5 Board320_240::sdcardToggleRecording() against SD_MMC.
 */
void BoardWaveshareSim7670g::sdcardToggleRecording()
{
  if (!liveData->params.sdcardInit)
  {
    return;
  }

  syslog->println("Toggle SD card recording...");
  liveData->params.sdcardRecording = !liveData->params.sdcardRecording;
  if (liveData->params.sdcardRecording)
  {
    liveData->params.sdcardCanNotify = true;
    liveData->params.sdcardLastFlushMs = millis();
    lastContributeSdRecordTime = 0;
  }
  else
  {
    if (sdcardRecordBuffer.length() > 0 && strlen(liveData->params.sdcardFilename) != 0)
    {
      if (rotateSdV2FileIfNeeded(boardFilesystem(), liveData->params.sdcardFilename,
                                 sizeof(liveData->params.sdcardFilename),
                                 sdcardRecordBuffer.length()))
      {
        syslog->print("SD v2 rollover file: ");
        syslog->println(liveData->params.sdcardFilename);
      }
      File file = boardFilesystem().open(liveData->params.sdcardFilename, FILE_APPEND);
      if (!file)
      {
        file = boardFilesystem().open(liveData->params.sdcardFilename, FILE_WRITE);
      }
      if (file)
      {
        file.print(sdcardRecordBuffer);
        file.close();
      }
      sdcardRecordBuffer = "";
    }
    liveData->params.sdcardFilename[0] = '\0';
  }
}

/**
 * Start console logging to /logs/<timestamp>.log on the SD_MMC card.
 * Headless equivalent of Board320_240::startSdcardConsoleLog().
 */
bool BoardWaveshareSim7670g::startSdConsoleLog()
{
  if (!liveData->params.sdcardInit && !sdcardMount())
  {
    return false;
  }
  if (syslog->isSdLogging())
  {
    return true;
  }
  if (!SD_MMC.exists("/logs"))
  {
    SD_MMC.mkdir("/logs");
  }

  char logPath[64];
  struct tm nowTm;
  if (!getLocalTime(&nowTm, 0))
  {
    memset(&nowTm, 0, sizeof(nowTm));
    nowTm.tm_year = 70; // 1970 until time is set
    nowTm.tm_mday = 1;
  }
  strftime(logPath, sizeof(logPath), "/logs/%Y-%m-%d_%H_%M_%S.log", &nowTm);
  if (SD_MMC.exists(logPath))
  {
    for (int i = 1; i < 100; i++)
    {
      char candidate[72];
      snprintf(candidate, sizeof(candidate), "/logs/%04d-%02d-%02d_%02d_%02d_%02d_%d.log",
               nowTm.tm_year + 1900, nowTm.tm_mon + 1, nowTm.tm_mday,
               nowTm.tm_hour, nowTm.tm_min, nowTm.tm_sec, i);
      if (!SD_MMC.exists(candidate))
      {
        strncpy(logPath, candidate, sizeof(logPath) - 1);
        logPath[sizeof(logPath) - 1] = '\0';
        break;
      }
    }
  }

  if (syslog->startSdLogging(logPath))
  {
    syslog->printf("Started SD console logging to %s\n", logPath);
    return true;
  }
  syslog->printf("Failed to start SD console logging to %s\n", logPath);
  return false;
}

void BoardWaveshareSim7670g::ntpSync()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    BoardCore::ntpSync();
    return;
  }
  if (!modem.dataReady())
  {
    syslog->printf("[NTP] Cannot sync: no WiFi and modem not online (state=%s, SIM %s).\n",
                   modem.stateName(), modem.info().simReady ? "ready" : "NOT ready");
    return;
  }

  // Modem clock may already be valid from NITZ or an earlier AT+CNTP.
  time_t utc = 0;
  if (!modem.readClockUtc(utc))
  {
    syslog->println("[NTP] Requesting time via modem (AT+CNTP), will read clock on next retry.");
    modem.requestNtp("pool.ntp.org");
    return;
  }

  setSystemTimeUtc(utc, "NTP/GSM");
  liveData->params.currTimeSyncWithGps = false;
  liveData->params.ntpTimeSet = true;
  syslog->println("[NTP] Time synchronized via cellular modem.");
  showTime();
}

NetTransport *BoardWaveshareSim7670g::activeTransport()
{
  if (liveData == nullptr)
  {
    return &wifiTransport;
  }

  switch (liveData->settings.modemTransportPolicy)
  {
  case 2: // Cellular only
    return &modemTransport;

  case 3: // WiFi only
    return &wifiTransport;

  case 0: // WiFi preferred, cellular fallback
  case 1: // Cellular preferred, WiFi fallback
  default:
  {
    // The failover manager owns the active-link decision for 0/1. Map its
    // preferred/fallback to the concrete transport for this policy.
    //   policy 0: preferred = WiFi,     fallback = cellular
    //   policy 1: preferred = cellular, fallback = WiFi
    const bool wifiIsPreferred = (liveData->settings.modemTransportPolicy == 0);
    if (failoverActive)
    {
      const bool usePreferred = (failover.activeLink() == NetFailoverManager::Link::Preferred);
      const bool useWifi = (usePreferred == wifiIsPreferred);
      return useWifi ? static_cast<NetTransport *>(&wifiTransport)
                     : static_cast<NetTransport *>(&modemTransport);
    }
    // Manager not started yet (very early boot): fall back to readiness, biased
    // to the policy's preferred link.
    if (wifiIsPreferred)
    {
      if (wifiTransport.ready()) return &wifiTransport;
      if (modemTransport.ready()) return &modemTransport;
      return &wifiTransport;
    }
    if (modemTransport.ready()) return &modemTransport;
    if (wifiTransport.ready()) return &wifiTransport;
    return &modemTransport;
  }
  }
}

void BoardWaveshareSim7670g::checkPinConflicts()
{
  struct PinAssignment
  {
    int pin;
    const char *name;
  };

  const PinAssignment pins[] = {
      {SIM7670G_RX_PIN, "SIM7670G RX"},
      {SIM7670G_TX_PIN, "SIM7670G TX"},
      {LOG_SERIAL_RX_PIN, "LOG UART RX"},
      {LOG_SERIAL_TX_PIN, "LOG UART TX"},
      {BAT_SDA_PIN, "Battery I2C SDA"},
      {BAT_SCL_PIN, "Battery I2C SCL"},
      {SDMMC_CLK_PIN, "SDMMC CLK"},
      {SDMMC_CMD_PIN, "SDMMC CMD"},
      {SDMMC_DATA_PIN, "SDMMC DATA"},
      {SD_CARDDETECT_PIN, "SD Card Detect"},
      {WS2812B_PIN, "WS2812B RGB"},
#ifdef CAN_ENABLE
      {CAN_TX_PIN, "CAN TX"},
      {CAN_RX_PIN, "CAN RX"},
#endif
  };

  bool conflict = false;
  const size_t count = sizeof(pins) / sizeof(pins[0]);
  for (size_t i = 0; i < count; i++)
  {
    for (size_t j = i + 1; j < count; j++)
    {
      if (pins[i].pin == pins[j].pin && pins[i].pin >= 0)
      {
        syslog->printf("PIN CONFLICT ERROR: GPIO %d claimed by both '%s' and '%s'!\n",
                       pins[i].pin, pins[i].name, pins[j].name);
        conflict = true;
      }
    }
  }

  if (!conflict)
  {
    syslog->println("Pin check: OK (no peripheral pin conflicts detected)");
  }
}

void BoardWaveshareSim7670g::updateBatteryState()
{
  if (liveData->settings.voltmeterEnabled != 1)
  {
    return;
  }
  if (liveData->params.currentTime - liveData->params.lastVoltageReadTime <= 5)
  {
    return;
  }

  liveData->params.lastVoltageReadTime = liveData->params.currentTime;

  batteryMonitor.configure(liveData->settings.batteryPackType,
                           liveData->settings.batteryUsbChargeEnabled != 0);

  float rawVoltage = 0.0f;
  float rawSoc = 0.0f;
  if (max17048.begin(Wire))
  {
    rawVoltage = max17048.getVoltage();
    rawSoc = max17048.getSOC();
  }

  batteryMonitor.updateSingleGauge(rawVoltage, rawSoc, false);

  if (batteryMonitor.isGaugeReady())
  {
    liveData->params.auxVoltage = batteryMonitor.getVoltage();
    liveData->params.auxPerc = batteryMonitor.getSoc();

    if (liveData->params.auxVoltage > liveData->settings.voltmeterSleep)
    {
      liveData->params.lastVoltageOkTime = liveData->params.currentTime;
    }

    // Protect 1S 18650 battery: require voltage > 2.5V so unread gauge / noise cannot trigger shutdown
    if (liveData->settings.sleepModeLevel == SLEEP_MODE_SCREEN_ONLY &&
        liveData->params.auxVoltage > 2.5f &&
        liveData->params.auxVoltage < liveData->settings.voltmeterCutOff)
    {
      syslog->print("AUX voltage under cut-off voltage: ");
      syslog->println(liveData->settings.voltmeterCutOff);
      shutdownDevice();
    }
  }
}

void BoardWaveshareSim7670g::batteryInfo()
{
  syslog->println(".-[ Battery & Power Management ]-_.");
  syslog->printf("Gauge Status:   %s (MAX17048 on I2C SDA %d, SCL %d)\n",
                 batteryMonitor.isGaugeReady() ? "READY" : "NOT READY",
                 BAT_SDA_PIN, BAT_SCL_PIN);
  syslog->printf("Battery:        1S 18650 (%s)\n",
                 batteryMonitor.packTypeName());
  syslog->printf("USB Charge:     %s\n",
                 batteryMonitor.isUsbChargeOnly() ? "Enabled (USB power only)" : "Disabled");
  syslog->printf("Voltage / SoC:  %.2f V  |  %.1f %%\n",
                 liveData->params.auxVoltage,
                 liveData->params.auxPerc);
}
