#ifdef BOARD_WAVESHARE_SIM7670G

#include "Sim7670G.h"
#include "LogSerial.h"
#include "config.h"
#include "evdash_certs.h"

static constexpr uint32_t kNetOpenTimeoutMs = 30000;
static constexpr uint32_t kStatusPollMs = 3000;
static constexpr uint32_t kStatusPollConnectedMs = 10000;

// Filename under which the ISRG Root X1 CA (HiveMQ Cloud / Let's Encrypt chain)
// is stored in the modem's certificate store for server authentication.
static const char *const kCaCertName = "isrgrootx1.pem";

// Unsolicited result codes. They can arrive in the middle of a command's reply,
// so they have to be recognised by prefix and kept out of the response buffer.
static const char *const kUrcPrefixes[] = {
    "+CIPRX", "+CMQTTRX", "+CGNSSINFO", "+CGEV", "+NETOPEN", "+CIPEVENT", "+CMQTTCONNLOST",
    "+CIPOPEN", "+CCHOPEN", "+CCHCLOSE", "+CIPCLOSE", "+CDNSGIP", "+CCHRECV", "RDY", "PB DONE"};

void Sim7670G::begin(HardwareSerial *serial, LiveData *data)
{
  uart = serial;
  liveData = data;
  reset();
}

const char *Sim7670G::stateName() const
{
  switch (currentState)
  {
  case STATE_OFF: return "OFF";
  case STATE_INIT: return "INIT";
  case STATE_READY: return "READY";
  case STATE_REGISTERED: return "REGISTERED";
  case STATE_NET_OPEN: return "NET_OPEN";
  }
  return "?";
}

void Sim7670G::reset()
{
  queue.clear();
  commandInFlight = false;
  responseBuffer = "";
  lineBuffer = "";
  lineIsNmea = false;
  atLineStart = true;
  stepBusy = false;
  initStep = 0;
  pollStep = 0;
  copsPollCounter = 0;
  nextActionMs = 0;
  netOpenPending = false;
  apnWarned = false;
  wasDataEnabled = (liveData != nullptr) && dataEnabled();
  readyCallbackCalled = false;
  modemInfo = Info();
  if (uart != nullptr)
  {
    while (uart->available())
    {
      uart->read();
    }
  }
  currentState = (uart != nullptr) ? STATE_INIT : STATE_OFF;
  pinAttempted = false;
  for (uint8_t i = 0; i < kMaxSockets; i++)
  {
    socketConnected[i] = false;
    socketSecure[i] = false;
    socketDataPending[i] = false;
    socketOpenDone[i] = false;
    socketOpenResult[i] = -1;
    socketRxHead[i] = 0;
    socketRxTail[i] = 0;
  }
  captureMode = false;
  captureLinkId = 0;
  captureRemaining = 0;
  syncWaiting = false;
  syncSuccess = false;
  promptWaiting = false;
  promptError = false;
  rawRxMode = false;
  rawRxExpected = 0;
  rawRxReceived = 0;
  dnsResolved = false;
  caCertUploaded = false;
}

bool Sim7670G::send(const String &command, uint32_t timeoutMs, ResponseCallback callback)
{
  if (queue.size() >= kMaxQueue)
  {
    return false;
  }
  queue.push_back({command, timeoutMs, callback});
  return true;
}

void Sim7670G::loop()
{
  if (uart == nullptr)
  {
    return;
  }
  readUart();
  checkTimeout();
  startNextCommand();
  tick();
  startNextCommand();
}

/**
 * Split the UART byte stream into lines and route each one.
 *
 * NMEA sentences are forwarded character by character rather than collected, so
 * the GNSS parser sees them exactly as it would on a dedicated UART.
 */
void Sim7670G::readUart()
{
  while (uart->available())
  {
    const int raw = uart->read();
    if (raw < 0)
    {
      break;
    }
    const char ch = static_cast<char>(raw);

    if (promptWaiting && ch == '>')
    {
      promptWaiting = false;
      continue;
    }

    if (rawRxMode && rawRxExpected > 0)
    {
      if (rawRxReceived < rawRxTargetMax && rawRxTarget != nullptr)
      {
        rawRxTarget[rawRxReceived] = static_cast<uint8_t>(raw);
      }
      rawRxReceived++;
      if (rawRxReceived >= rawRxExpected)
      {
        rawRxExpected = 0;
      }
      continue;
    }

    // Consuming the raw payload that follows a "+CCHRECV: DATA,<mux>,<len>" (or
    // "+CIPRXGET") push. These bytes are not AT lines; they go straight into the
    // socket's ring buffer.
    if (captureMode && captureRemaining > 0)
    {
      socketRxPush(captureLinkId, static_cast<uint8_t>(raw));
      captureRemaining--;
      if (captureRemaining == 0)
      {
        captureMode = false;
      }
      continue;
    }

    if (atLineStart && ch == '$')
    {
      lineIsNmea = true;
    }

    if (lineIsNmea)
    {
      if (nmeaSink)
      {
        nmeaSink(ch);
      }
      if (ch == '\n')
      {
        lineIsNmea = false;
        atLineStart = true;
      }
      else
      {
        atLineStart = false;
      }
      continue;
    }

    if (ch == '\r')
    {
      continue;
    }
    if (ch == '\n')
    {
      atLineStart = true;
      lineBuffer.trim();
      if (lineBuffer.length() > 0)
      {
        String line = lineBuffer;
        lineBuffer = "";
        handleLine(line);
      }
      continue;
    }

    atLineStart = false;
    if (lineBuffer.length() < kMaxLine)
    {
      lineBuffer += ch;
    }
  }
}

/**
 * One iteration of a blocking wait. Always services the modem UART, then hands
 * control to the yield hook (if set) so GPS/console/other work keeps running
 * and the CPU is released to FreeRTOS during a slow AT exchange. The hook must
 * not touch the modem UART (that stays owned by this driver).
 */
void Sim7670G::pumpWait()
{
  readUart();
  if (yieldHook)
  {
    yieldHook();
  }
}

bool Sim7670G::isFinalResult(const String &line, bool &ok) const
{
  if (line == "OK")
  {
    ok = true;
    return true;
  }
  if (line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR") || line.startsWith("+IP ERROR"))
  {
    ok = false;
    return true;
  }
  return false;
}

bool Sim7670G::isUrc(const String &line) const
{
  for (const char *prefix : kUrcPrefixes)
  {
    if (line.startsWith(prefix))
    {
      return true;
    }
  }
  return false;
}

void Sim7670G::handleLine(const String &line)
{
  syslog->info(DEBUG_MODEM, String("MODEM < ") + line);

  // While waiting for a '>' send prompt (AT+CCHSEND / AT+CIPSEND), the modem may
  // instead answer with ERROR if the send was rejected (wrong state, closed
  // link, bad length). Capture that so sendSocketData can abort immediately
  // rather than spinning until its timeout.
  if (promptWaiting && !syncWaiting && !commandInFlight)
  {
    bool ok = false;
    if (isFinalResult(line, ok) && !ok)
    {
      promptError = true;
      promptWaiting = false;
      return;
    }
  }

  if (commandInFlight)
  {
    // With echo still on, the modem repeats the command before answering it.
    if (line == inFlight.text)
    {
      return;
    }
    bool ok = false;
    if (isFinalResult(line, ok))
    {
      if (!ok)
      {
        if (responseBuffer.length() > 0)
        {
          responseBuffer += '\n';
        }
        responseBuffer += line;
      }
      completeCommand(ok, true);
      return;
    }
    if (isUrc(line))
    {
      handleUrc(line);
      return;
    }
    if (responseBuffer.length() + line.length() + 1 < kMaxResponse)
    {
      if (responseBuffer.length() > 0)
      {
        responseBuffer += '\n';
      }
      responseBuffer += line;
    }
    return;
  }

  if (syncWaiting)
  {
    bool ok = false;
    if (isFinalResult(line, ok))
    {
      syncSuccess = ok;
      syncWaiting = false;
      return;
    }
    if (isUrc(line))
    {
      handleUrc(line);
      return;
    }
    if (rawRxMode && rawRxExpected == 0)
    {
      if (line.startsWith("+CIPRXGET: 2,"))
      {
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        int thirdComma = line.indexOf(',', secondComma + 1);
        if (firstComma > 0 && secondComma > 0)
        {
          int len = (thirdComma > 0) ? line.substring(secondComma + 1, thirdComma).toInt()
                                     : line.substring(secondComma + 1).toInt();
          rawRxExpected = (len > 0) ? static_cast<size_t>(len) : 0;
          if (rawRxExpected == 0)
          {
            rawRxMode = false;
          }
          return;
        }
      }
      else if (line.startsWith("+CCHRECV:"))
      {
        int lastComma = line.lastIndexOf(',');
        if (lastComma > 0)
        {
          int len = line.substring(lastComma + 1).toInt();
          rawRxExpected = (len > 0) ? static_cast<size_t>(len) : 0;
          if (rawRxExpected == 0)
          {
            rawRxMode = false;
          }
          return;
        }
      }
    }
    if (syncResponseBuffer.length() + line.length() + 1 < kMaxResponse)
    {
      if (syncResponseBuffer.length() > 0)
      {
        syncResponseBuffer += '\n';
      }
      syncResponseBuffer += line;
    }
    return;
  }

  handleUrc(line);
}

void Sim7670G::handleUrc(const String &line)
{
  if (line.startsWith("+NETOPEN:"))
  {
    const int code = line.substring(line.indexOf(':') + 1).toInt();
    if (netOpenPending)
    {
      netOpenPending = false;
      if (code == 0)
      {
        setState(STATE_NET_OPEN);
      }
      else
      {
        syslog->info(DEBUG_MODEM, String("MODEM: data connection refused, code ") + code);
        backoff();
      }
    }
  }
  else if ((line.startsWith("+CGEV:") && line.indexOf("DEACT") >= 0) ||
           (line.startsWith("+CIPEVENT:") && line.indexOf("NETWORK CLOSED") >= 0))
  {
    if (currentState == STATE_NET_OPEN)
    {
      setState(STATE_REGISTERED);
    }
  }
  else if (line == "RDY" || line == "PB DONE")
  {
    // The modem restarted underneath us, so whatever was set up is gone.
    if (currentState != STATE_INIT)
    {
      setState(STATE_INIT);
    }
  }
  else if (line.startsWith("+CIPOPEN:"))
  {
    int comma = line.indexOf(',');
    if (comma > 0)
    {
      int linkId = line.substring(9, comma).toInt();
      int err = line.substring(comma + 1).toInt();
      if (linkId >= 0 && linkId < kMaxSockets)
      {
        socketOpenDone[linkId] = true;
        socketOpenResult[linkId] = err;
        if (err == 0)
        {
          socketConnected[linkId] = true;
        }
      }
    }
  }
  else if (line.startsWith("+CCHOPEN:"))
  {
    int comma = line.indexOf(',');
    if (comma > 0)
    {
      int linkId = line.substring(9, comma).toInt();
      int err = line.substring(comma + 1).toInt();
      if (linkId >= 0 && linkId < kMaxSockets)
      {
        socketOpenDone[linkId] = true;
        socketOpenResult[linkId] = err;
        if (err == 0)
        {
          socketConnected[linkId] = true;
        }
      }
    }
  }
  else if (line.startsWith("+CIPCLOSE:"))
  {
    int linkId = line.substring(10).toInt();
    if (linkId >= 0 && linkId < kMaxSockets)
    {
      socketConnected[linkId] = false;
    }
  }
  else if (line.startsWith("+CCHCLOSE:"))
  {
    int linkId = line.substring(10).toInt();
    if (linkId >= 0 && linkId < kMaxSockets)
    {
      socketConnected[linkId] = false;
    }
  }
  else if (line.startsWith("+CCHRECV:"))
  {
    // Two shapes:
    //   +CCHRECV: DATA,<mux>,<len>  -> <len> raw bytes follow immediately (push)
    //   +CCHRECV: <mux>,<len>       -> length-only notification
    const int dataPos = line.indexOf("DATA,");
    if (dataPos >= 0)
    {
      String rest = line.substring(dataPos + 5); // after "DATA,"
      const int comma = rest.indexOf(',');
      if (comma > 0)
      {
        const int mux = rest.substring(0, comma).toInt();
        const int len = rest.substring(comma + 1).toInt();
        if (mux >= 0 && mux < kMaxSockets && len > 0)
        {
          captureLinkId = static_cast<uint8_t>(mux);
          captureRemaining = static_cast<size_t>(len);
          captureMode = true;
          socketDataPending[mux] = true;
        }
      }
    }
    else
    {
      const int colon = line.indexOf(':');
      String rest = line.substring(colon + 1);
      rest.trim();
      const int comma = rest.indexOf(',');
      if (comma > 0)
      {
        const int mux = rest.substring(0, comma).toInt();
        if (mux >= 0 && mux < kMaxSockets)
        {
          socketDataPending[mux] = true;
        }
      }
    }
  }
  else if (line.startsWith("+CIPRXGET: 1,"))
  {
    int linkId = line.substring(13).toInt();
    if (linkId >= 0 && linkId < kMaxSockets)
    {
      socketDataPending[linkId] = true;
    }
  }
  else if (line.startsWith("+CDNSGIP: 1,"))
  {
    int lastQuote = line.lastIndexOf('"');
    int secondLastQuote = line.lastIndexOf('"', lastQuote - 1);
    if (lastQuote > secondLastQuote && secondLastQuote >= 0)
    {
      String ipStr = line.substring(secondLastQuote + 1, lastQuote);
      if (dnsResolvedIp.fromString(ipStr))
      {
        dnsResolved = true;
      }
    }
  }

  if (urcHandler)
  {
    urcHandler(line);
  }
}

void Sim7670G::startNextCommand()
{
  if (commandInFlight || queue.empty())
  {
    return;
  }
  inFlight = queue.front();
  queue.pop_front();
  responseBuffer = "";
  commandInFlight = true;
  inFlightStartMs = millis();
  totalCommands++;
  syslog->info(DEBUG_MODEM, String("MODEM > ") + inFlight.text);
  uart->print(inFlight.text);
  uart->print("\r\n");
}

void Sim7670G::completeCommand(bool ok, bool responded)
{
  commandInFlight = false;
  if (responded)
  {
    everResponded = true;
  }
  ResponseCallback callback = inFlight.callback;
  const String response = responseBuffer;
  inFlight.callback = nullptr;
  responseBuffer = "";
  if (callback)
  {
    callback(ok, response);
  }
}

void Sim7670G::checkTimeout()
{
  if (!commandInFlight || (millis() - inFlightStartMs) < inFlight.timeoutMs)
  {
    return;
  }
  totalTimeouts++;
  syslog->info(DEBUG_MODEM, String("MODEM ! timeout: ") + inFlight.text);
  responseBuffer = "timeout";
  completeCommand(false, false);
}

uint32_t Sim7670G::retryBackoffMs() const
{
  const uint32_t sec = liveData->settings.modemRetryBackoffSec;
  return (sec == 0 ? 5U : sec) * 1000U;
}

bool Sim7670G::dataEnabled() const
{
  return liveData->settings.modemEnabled == 1;
}

bool Sim7670G::apnConfigured() const
{
  const char *apn = liveData->settings.modemApn;
  return apn[0] != '\0' && strcmp(apn, "not_set") != 0;
}

bool Sim7670G::registered() const
{
  if (modemInfo.registration == 1)
  {
    return true;
  }
  if (liveData != nullptr && liveData->settings.modemRoaming == 1 && modemInfo.registration == 5)
  {
    return true;
  }
  return false;
}

uint32_t Sim7670G::pollIntervalMs() const
{
  return currentState == STATE_NET_OPEN ? kStatusPollConnectedMs : kStatusPollMs;
}

void Sim7670G::backoff()
{
  nextActionMs = millis() + retryBackoffMs();
}

void Sim7670G::setState(State next)
{
  if (next == currentState)
  {
    return;
  }
  syslog->info(DEBUG_MODEM, String("MODEM state: ") + stateName() + " -> " +
                              (next == STATE_OFF ? "OFF" : next == STATE_INIT ? "INIT" : next == STATE_READY ? "READY" : next == STATE_REGISTERED ? "REGISTERED" : "NET_OPEN"));
  currentState = next;
  stepBusy = false;
  nextActionMs = 0;
  if (next < STATE_NET_OPEN)
  {
    modemInfo.ipAddress = "";
  }
  if (next != STATE_REGISTERED)
  {
    netOpenPending = false;
  }
  if (next == STATE_INIT)
  {
    initStep = 0;
    modemInfo.simReady = false;
  }
}

/**
 * Advance the state machine by at most one command. Nothing is issued while any
 * command is queued or in flight, so the steps stay strictly sequential.
 */
void Sim7670G::tick()
{
  if (liveData != nullptr)
  {
    const bool currentDataEnabled = dataEnabled();
    if (currentDataEnabled != wasDataEnabled)
    {
      wasDataEnabled = currentDataEnabled;
      reset();
      return;
    }
  }

  if (stepBusy || commandInFlight || !queue.empty())
  {
    return;
  }
  if (static_cast<int32_t>(millis() - nextActionMs) < 0)
  {
    return;
  }

  switch (currentState)
  {
  case STATE_INIT: stepInit(); break;
  case STATE_READY: stepReady(); break;
  case STATE_REGISTERED: stepRegistered(); break;
  case STATE_NET_OPEN: stepNetOpen(); break;
  case STATE_OFF: break;
  }
}

void Sim7670G::stepInit()
{
  if (initStep >= 7)
  {
    setState(STATE_READY);
    if (readyCallback && !readyCallbackCalled)
    {
      readyCallbackCalled = true;
      readyCallback();
    }
    return;
  }

  stepBusy = true;
  switch (initStep)
  {
  case 0:
    // Echo off, so every line read back is either a reply or an unsolicited code.
    send("ATE0", 1000, [this](bool ok, const String &)
         {
           stepBusy = false;
           if (ok)
           {
             if (readyCallback && !readyCallbackCalled)
             {
               readyCallbackCalled = true;
               readyCallback();
             }
             // If cellular data is disabled, skip SIM/APN setup and query hardware info.
             initStep = dataEnabled() ? 1 : 2;
           }
           else
           {
             backoff();
           }
         });
    break;
  case 1:
    send("AT+CPIN?", 3000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           if (ok && response.indexOf("READY") >= 0)
           {
             modemInfo.simReady = true;
             initStep = 2;
             return;
           }
           if (response.indexOf("SIM PIN") >= 0 && liveData->settings.modemPin[0] != '\0' && !pinAttempted)
           {
             // Only one attempt per reset: a wrong PIN retried in a loop would PUK-lock the SIM.
             pinAttempted = true;
             send(String("AT+CPIN=\"") + liveData->settings.modemPin + "\"", 5000);
             nextActionMs = millis() + 2000;
             return;
           }
           if (response.indexOf("SIM PIN") >= 0)
           {
             syslog->info(DEBUG_MODEM, pinAttempted
                                         ? "MODEM: SIM PIN rejected, check 'modem=pin=XXXX' then 'modem=reset'"
                                         : "MODEM: SIM needs PIN, set it with 'modem=pin=XXXX' then 'modem=reset'");
           }
           syslog->info(DEBUG_MODEM, String("MODEM: SIM not ready: ") + response);
           backoff();
         });
    break;
  case 2:
    send("AT+CGSN", 2000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           if (ok)
           {
             modemInfo.imei = firstLine(response);
             initStep = 3;
           }
           else
           {
             backoff();
           }
         });
    break;
  case 3:
    send("AT+CGMR", 2000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           if (ok)
           {
             String line = firstLine(response);
             if (line.startsWith("+"))
             {
               const int colon = line.indexOf(':');
               if (colon >= 0)
               {
                 line = line.substring(colon + 1);
                 line.trim();
               }
             }
             modemInfo.firmware = line;
             initStep = dataEnabled() ? 4 : 7;
           }
           else
           {
             backoff();
           }
         });
    break;
  case 4:
  {
    // 0 auto, 1 LTE only, 2 GSM only. A refused mode is not fatal: the modem keeps
    // its previous setting and still registers.
    const uint8_t mode = liveData->settings.modemNetworkMode;
    const int cnmp = (mode == 1) ? 38 : (mode == 2) ? 13 : 2;
    send(String("AT+CNMP=") + cnmp, 3000, [this](bool, const String &)
         {
           stepBusy = false;
           initStep = 5;
         });
    break;
  }
  case 5:
    if (apnConfigured())
    {
      send(String("AT+CGDCONT=1,\"IP\",\"") + liveData->settings.modemApn + "\"", 3000, [this](bool, const String &)
           {
             stepBusy = false;
             initStep = 6;
           });
    }
    else
    {
      stepBusy = false;
      initStep = 6;
    }
    break;
  case 6:
    if (apnConfigured() && (liveData->settings.modemApnAuth > 0 || liveData->settings.modemApnUser[0] != '\0'))
    {
      uint8_t auth = liveData->settings.modemApnAuth;
      if (auth == 0)
      {
        auth = 1; // Default to PAP if credentials given without explicit auth type
      }
      // SIM7670G's AT+CGAUTH only accepts auth_type 0 (none), 1 (PAP) or 2 (CHAP).
      // The UI no longer offers anything else, but guard against a stale value from
      // older firmware/EEPROM (e.g. the previously-offered "PAP or CHAP"=3) which
      // the modem rejects with "+CME ERROR: Incorrect parameters". Fall back to PAP.
      if (auth > 2)
      {
        auth = 1;
      }
      String cmd = String("AT+CGAUTH=1,") + auth;
      if (liveData->settings.modemApnPass[0] != '\0' || liveData->settings.modemApnUser[0] != '\0')
      {
        cmd += ",\"";
        cmd += liveData->settings.modemApnPass;
        cmd += "\",\"";
        cmd += liveData->settings.modemApnUser;
        cmd += "\"";
      }
      send(cmd, 3000, [this](bool, const String &)
           {
             stepBusy = false;
             initStep = 7;
           });
    }
    else
    {
      stepBusy = false;
      initStep = 7;
    }
    break;
  }
}

void Sim7670G::stepReady()
{
  if (!dataEnabled())
  {
    nextActionMs = millis() + kStatusPollMs;
    return;
  }
  if (registered())
  {
    setState(STATE_REGISTERED);
    return;
  }
  pollStatus();
}

void Sim7670G::stepRegistered()
{
  if (!dataEnabled() || !registered())
  {
    setState(STATE_READY);
    return;
  }
  if (!apnConfigured())
  {
    if (!apnWarned)
    {
      apnWarned = true;
      syslog->println("MODEM: registered, but no APN is set (modem=apn=<apn>), data connection not opened");
    }
    nextActionMs = millis() + kStatusPollMs;
    return;
  }
  if (netOpenPending)
  {
    if ((millis() - netOpenSentMs) > kNetOpenTimeoutMs)
    {
      netOpenPending = false;
      syslog->println("MODEM: data connection did not open in time");
      backoff();
    }
    else
    {
      nextActionMs = millis() + 500;
    }
    return;
  }

  stepBusy = true;
  send("AT+NETOPEN", 10000, [this](bool ok, const String &response)
       {
         stepBusy = false;
         if (ok)
         {
           // The result arrives afterwards as a +NETOPEN: URC.
           netOpenPending = true;
           netOpenSentMs = millis();
         }
         else if (response.indexOf("already") >= 0)
         {
           setState(STATE_NET_OPEN);
         }
         else
         {
           syslog->info(DEBUG_MODEM, String("MODEM: AT+NETOPEN failed: ") + response);
           backoff();
         }
       });
}

void Sim7670G::stepNetOpen()
{
  if (!dataEnabled() || !registered())
  {
    setState(STATE_READY);
    return;
  }
  if (modemInfo.ipAddress.length() == 0)
  {
    stepBusy = true;
    send("AT+CGPADDR=1", 3000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           // Format: +CGPADDR: 1,"10.x.x.x" or +CGPADDR: 1,10.x.x.x or +CGPADDR: 1,"10.x.x.x","2001:..."
           if (ok)
           {
             String ip = "";
             int firstQuote = response.indexOf('"');
             int secondQuote = (firstQuote >= 0) ? response.indexOf('"', firstQuote + 1) : -1;
             if (firstQuote >= 0 && secondQuote > firstQuote)
             {
               ip = response.substring(firstQuote + 1, secondQuote);
             }
             else
             {
               int comma = response.indexOf(',');
               if (comma >= 0)
               {
                 ip = response.substring(comma + 1);
                 int nextComma = ip.indexOf(',');
                 if (nextComma >= 0)
                 {
                   ip = ip.substring(0, nextComma);
                 }
               }
             }
             ip.trim();
             if (ip.length() > 0 && ip != "0.0.0.0")
             {
               modemInfo.ipAddress = ip;
               return;
             }
           }
           nextActionMs = millis() + kStatusPollMs;
         });
    return;
  }
  pollStatus();
}

/**
 * Refresh registration, signal strength and operator, one query per call.
 */
void Sim7670G::pollStatus()
{
  stepBusy = true;
  switch (pollStep)
  {
  case 0:
    send("AT+CEREG?", 2000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           const int stat = ok ? parseRegistration(response) : -1;
           if (stat >= 0)
           {
             modemInfo.registration = stat;
           }
           pollStep = registered() ? 2 : 1;
           nextActionMs = millis() + 200;
         });
    break;
  case 1:
    // Circuit-switched fallback: on a GSM-only network CEREG never reports registered.
    send("AT+CGREG?", 2000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           const int stat = ok ? parseRegistration(response) : -1;
           if (stat == 1 || stat == 5)
           {
             modemInfo.registration = stat;
           }
           pollStep = 2;
           nextActionMs = millis() + 200;
         });
    break;
  case 2:
    send("AT+CSQ", 2000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           if (ok)
           {
             modemInfo.rssiDbm = parseSignalDbm(response);
           }
           if (!registered())
           {
             pollStep = 0;
             nextActionMs = millis() + pollIntervalMs();
             return;
           }
           // Only spend a (slow) AT+COPS? when we still need the operator name or
           // once every kCopsPollEvery cycles; otherwise end the cycle here so the
           // UART is free for MQTT between polls.
           const bool needOperator = (modemInfo.operatorName.length() == 0);
           const bool periodicRefresh = (++copsPollCounter >= kCopsPollEvery);
           if (needOperator || periodicRefresh)
           {
             copsPollCounter = 0;
             pollStep = 3;
             nextActionMs = millis() + 200;
           }
           else
           {
             pollStep = 0;
             nextActionMs = millis() + pollIntervalMs();
           }
         });
    break;
  default:
    send("AT+COPS?", 3000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           if (ok)
           {
             modemInfo.operatorName = firstQuoted(response);
           }
           pollStep = 0;
           nextActionMs = millis() + pollIntervalMs();
         });
    break;
  }
}

/**
 * Pull <stat> out of "+CEREG: <n>,<stat>[,...]". A reply with a single field is
 * the unsolicited form, where that field is the stat itself.
 */
int Sim7670G::parseRegistration(const String &response)
{
  const int colon = response.indexOf(':');
  if (colon < 0)
  {
    return -1;
  }
  String fields = response.substring(colon + 1);
  fields.trim();
  const int firstComma = fields.indexOf(',');
  if (firstComma < 0)
  {
    return fields.toInt();
  }
  String rest = fields.substring(firstComma + 1);
  const int secondComma = rest.indexOf(',');
  if (secondComma >= 0)
  {
    rest = rest.substring(0, secondComma);
  }
  rest.trim();
  return rest.toInt();
}

/**
 * "+CSQ: <rssi>,<ber>" maps 0..31 to -113..-51 dBm in 2 dB steps; 99 is unknown.
 */
int Sim7670G::parseSignalDbm(const String &response)
{
  const int colon = response.indexOf(':');
  if (colon < 0)
  {
    return 0;
  }
  const int index = response.substring(colon + 1).toInt();
  if (index < 0 || index > 31)
  {
    return 0;
  }
  return -113 + 2 * index;
}

String Sim7670G::firstQuoted(const String &text)
{
  const int open = text.indexOf('"');
  if (open < 0)
  {
    return "";
  }
  const int close = text.indexOf('"', open + 1);
  if (close < 0)
  {
    return "";
  }
  return text.substring(open + 1, close);
}

String Sim7670G::firstLine(const String &text)
{
  String line = text;
  const int newline = line.indexOf('\n');
  if (newline >= 0)
  {
    line = line.substring(0, newline);
  }
  line.trim();
  return line;
}

bool Sim7670G::sendSync(const String &command, uint32_t timeoutMs, String *outResponse)
{
  if (uart == nullptr)
  {
    return false;
  }

  // Drain any asynchronous command that was already queued/in-flight
  uint32_t drainStartMs = millis();
  while (commandInFlight && (millis() - drainStartMs < 2000))
  {
    pumpWait();
    checkTimeout();
    tick();
  }

  syncResponseBuffer = "";
  syncSuccess = false;
  syncWaiting = true;
  totalCommands++;

  syslog->info(DEBUG_MODEM, String("MODEM [sync] > ") + command);
  uart->print(command);
  uart->print("\r\n");

  uint32_t startMs = millis();
  while (syncWaiting && (millis() - startMs < timeoutMs))
  {
    pumpWait();
  }

  bool ok = false;
  if (syncWaiting)
  {
    syncWaiting = false;
    totalTimeouts++;
    syslog->info(DEBUG_MODEM, String("MODEM [sync] timeout: ") + command);
  }
  else
  {
    ok = syncSuccess;
    everResponded = true;
  }

  if (outResponse != nullptr)
  {
    *outResponse = syncResponseBuffer;
  }

  return ok;
}

bool Sim7670G::requestNtp(const char *server)
{
  if (currentState != STATE_NET_OPEN || server == nullptr || server[0] == '\0')
  {
    return false;
  }
  // Let the network (NITZ) keep the modem clock updated as well.
  sendSync("AT+CTZU=1", 1000);
  if (!sendSync(String("AT+CNTP=\"") + server + "\",0", 2000))
  {
    return false;
  }
  // Result arrives later as URC "+CNTP: <err>"; clock is read via readClockUtc().
  return sendSync("AT+CNTP", 2000);
}

bool Sim7670G::readClockUtc(time_t &utcOut)
{
  String resp;
  if (!sendSync("AT+CCLK?", 2000, &resp))
  {
    return false;
  }
  // +CCLK: "yy/MM/dd,hh:mm:ss±zz"  (local time, zz = quarter hours)
  const int q = resp.indexOf("+CCLK: \"");
  if (q < 0)
  {
    return false;
  }
  int yy, MM, dd, hh, mm, ss, tz = 0;
  char sign = '+';
  const char *p = resp.c_str() + q + 8;
  const int n = sscanf(p, "%d/%d/%d,%d:%d:%d%c%d", &yy, &MM, &dd, &hh, &mm, &ss, &sign, &tz);
  if (n < 6 || yy < 24 || yy > 99 || MM < 1 || MM > 12 || dd < 1 || dd > 31 ||
      hh > 23 || mm > 59 || ss > 60)
  {
    syslog->info(DEBUG_MODEM, String("MODEM: clock not set yet: ") + firstLine(resp));
    return false;
  }
  // Days from civil (Howard Hinnant), avoids TZ-dependent mktime().
  int y = 2000 + yy - (MM <= 2 ? 1 : 0);
  const int era = y / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (MM + (MM > 2 ? -3 : 9)) + 2) / 5 + dd - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = static_cast<int64_t>(era) * 146097 + doe - 719468;
  // On SIMCom SIM7670 (synced via AT+CNTP="...",0 / AT+CTZU=1), the RTC time
  // in AT+CCLK? is already UTC time. The trailing ±zz is the carrier's NITZ
  // local timezone offset and must not be subtracted from the UTC clock.
  int64_t epoch = days * 86400 + hh * 3600 + mm * 60 + ss;
  if (epoch < 1704067200) // 2024-01-01
  {
    return false;
  }
  utcOut = static_cast<time_t>(epoch);
  return true;
}

bool Sim7670G::ensureCaCert()
{
  if (caCertUploaded)
  {
    return true;
  }

  // The cert persists in modem flash across reboots. To avoid needless flash
  // writes (wear), query the store first and only upload when the cert is
  // *confirmed* absent. If AT+CCERTLIST itself fails (transient error, parse
  // quirk), treat the state as unknown and do NOT upload: a failed query must
  // not trigger a rewrite on every reset. We assume it is present and proceed;
  // if it is genuinely missing the handshake fails and the next attempt retries.
  String listResp;
  const bool listOk = sendSync("AT+CCERTLIST", 3000, &listResp);
  if (!listOk)
  {
    syslog->info(DEBUG_MODEM, "MODEM [cert] CCERTLIST query failed; assuming CA present, not re-uploading");
    caCertUploaded = true; // avoid repeated probing/writes this session
    return true;
  }
  if (listResp.indexOf(kCaCertName) >= 0)
  {
    caCertUploaded = true;
    return true;
  }

  const size_t len = strlen(ISRG_ROOT_X1);
  if (len == 0)
  {
    return false;
  }

  // AT+CCERTDOWN="<name>",<len> replies with a '>' prompt, then expects exactly
  // <len> raw bytes of PEM, then OK. Reuse the same prompt handshake as sendSocketData.
  String cmd = String("AT+CCERTDOWN=\"") + kCaCertName + "\"," + len;
  syslog->info(DEBUG_MODEM, String("MODEM [cert] > ") + cmd);
  promptWaiting = true;
  promptError = false;
  uart->print(cmd);
  uart->print("\r\n");

  uint32_t startMs = millis();
  bool gotPrompt = false;
  while (millis() - startMs < 5000)
  {
    pumpWait();
    if (!promptWaiting)
    {
      gotPrompt = !promptError;
      break;
    }
  }
  if (!gotPrompt)
  {
    promptWaiting = false;
    syslog->info(DEBUG_MODEM, "MODEM [cert] no '>' prompt for CCERTDOWN; CA not installed");
    return false;
  }

  uart->write(reinterpret_cast<const uint8_t *>(ISRG_ROOT_X1), len);

  // Await final OK/ERROR for the download.
  syncWaiting = true;
  syncSuccess = false;
  syncResponseBuffer = "";
  startMs = millis();
  bool ok = false;
  while (millis() - startMs < 5000)
  {
    pumpWait();
    if (!syncWaiting)
    {
      ok = syncSuccess;
      break;
    }
  }
  syncWaiting = false;

  if (ok)
  {
    caCertUploaded = true;
    syslog->info(DEBUG_MODEM, String("MODEM [cert] uploaded ") + kCaCertName + " (" + len + " bytes)");
  }
  else
  {
    syslog->info(DEBUG_MODEM, "MODEM [cert] CCERTDOWN failed; server auth will not work");
  }
  return ok;
}

bool Sim7670G::openSocket(uint8_t linkId, const char *host, uint16_t port, bool secure, uint32_t timeoutMs)
{
  if (currentState != STATE_NET_OPEN || host == nullptr || port == 0 || linkId >= kMaxSockets)
  {
    return false;
  }

  if (socketConnected[linkId])
  {
    closeSocket(linkId, secure, 1000);
  }

  socketOpenDone[linkId] = false;
  socketOpenResult[linkId] = -1;
  // Discard anything left in the ring buffer from a previous session.
  socketRxHead[linkId] = 0;
  socketRxTail[linkId] = 0;
  if (captureMode && captureLinkId == linkId)
  {
    captureMode = false;
    captureRemaining = 0;
  }

  if (secure)
  {
    // SIM7670G (A76XX family) TLS-over-CCH setup. Order matters:
    //   1. configure the SSL context (sslversion / authmode / SNI) on context 0
    //   2. AT+CCHSTART to bring up the SSL/CCH service (activates the PDP context)
    //   3. AT+CCHSSLCFG to bind SSL context 0 to this channel (must be AFTER CCHSTART)
    //   4. AT+CCHOPEN to open the connection
    // The previous order issued CCHSSLCFG and CCHOPEN before CCHSTART, which made
    // CCHSTART return ERROR and left the stack unusable (connect timed out, state -4).
    sendSync("AT+CSSLCFG=\"sslversion\",0,4", 1000);

    // Server authentication (authmode 1) requires a CA cert in the modem store.
    // When the user asks for verification (modemTlsInsecure == 0), upload the
    // bundled ISRG Root X1 (HiveMQ Cloud / Let's Encrypt chain) and bind it to
    // SSL context 0. If provisioning fails, fall back to authmode 0 (still
    // encrypted, just unverified) so telemetry keeps flowing, with a warning.
    const bool wantVerify = (liveData && liveData->settings.modemTlsInsecure == 0);
    bool verify = false;
    if (wantVerify)
    {
      if (ensureCaCert())
      {
        sendSync(String("AT+CSSLCFG=\"cacert\",0,\"") + kCaCertName + "\"", 1000);
        verify = true;
      }
      else
      {
        syslog->info(DEBUG_MODEM, "MODEM: CA cert unavailable, falling back to unverified TLS (set modemTlsInsecure=1 to silence)");
      }
    }
    sendSync(String("AT+CSSLCFG=\"authmode\",0,") + (verify ? "1" : "0"), 1000);
    // enableSNI is enough; the SNI host name is taken from the CCHOPEN host.
    // The explicit AT+CSSLCFG="sni",... sub-command is not supported here and returns ERROR.
    sendSync("AT+CSSLCFG=\"enableSNI\",0,1", 1000);

    // Start the SSL/CCH service. ERROR here usually means it is already started
    // from a previous attempt, which is fine, so the result is intentionally not
    // treated as fatal. CCHSSLCFG below only works once the service is up.
    sendSync("AT+CCHSTART", 2000);

    sendSync(String("AT+CCHSSLCFG=") + linkId + ",0", 1000);

    String cmd = String("AT+CCHOPEN=") + linkId + ",\"" + host + "\"," + port + ",2";
    if (!sendSync(cmd, timeoutMs))
    {
      return false;
    }
  }
  else
  {
    sendSync("AT+CIPRXGET=1", 1000);
    String cmd = String("AT+CIPOPEN=") + linkId + ",\"TCP\",\"" + host + "\"," + port;
    if (!sendSync(cmd, timeoutMs))
    {
      return false;
    }
  }

  uint32_t startMs = millis();
  while (!socketOpenDone[linkId] && (millis() - startMs < timeoutMs))
  {
    pumpWait();
  }

  if (socketOpenDone[linkId] && socketOpenResult[linkId] == 0)
  {
    socketConnected[linkId] = true;
    socketSecure[linkId] = secure;
    socketDataPending[linkId] = false;
    return true;
  }

  socketConnected[linkId] = false;
  return false;
}

bool Sim7670G::closeSocket(uint8_t linkId, bool secure, uint32_t timeoutMs)
{
  if (linkId >= kMaxSockets)
  {
    return false;
  }
  const bool wasConnected = socketConnected[linkId];
  const bool useSecure = wasConnected ? socketSecure[linkId] : secure;
  socketConnected[linkId] = false;
  socketDataPending[linkId] = false;
  if (!wasConnected)
  {
    return true;
  }
  if (useSecure)
  {
    return sendSync(String("AT+CCHCLOSE=") + linkId, timeoutMs);
  }
  else
  {
    return sendSync(String("AT+CIPCLOSE=") + linkId, timeoutMs);
  }
}

size_t Sim7670G::sendSocketData(uint8_t linkId, const uint8_t *data, size_t len, bool secure, uint32_t timeoutMs)
{
  if (!socketConnected[linkId] || data == nullptr || len == 0 || linkId >= kMaxSockets)
  {
    return 0;
  }

  size_t sentTotal = 0;
  const size_t kMaxChunk = 1024;

  while (sentTotal < len)
  {
    const size_t chunkSize = (len - sentTotal > kMaxChunk) ? kMaxChunk : (len - sentTotal);
    String cmd = secure ? (String("AT+CCHSEND=") + linkId + "," + chunkSize)
                        : (String("AT+CIPSEND=") + linkId + "," + chunkSize);

    syslog->info(DEBUG_MODEM, String("MODEM [send] > ") + cmd);
    promptWaiting = true;
    promptError = false;
    uart->print(cmd);
    uart->print("\r\n");

    uint32_t startMs = millis();
    bool gotPrompt = false;
    while (millis() - startMs < timeoutMs)
    {
      pumpWait();
      if (!promptWaiting)
      {
        gotPrompt = !promptError;
        break;
      }
    }

    if (!gotPrompt)
    {
      promptWaiting = false;
      syslog->info(DEBUG_MODEM, promptError
                                  ? String("MODEM [send] rejected (no prompt, ERROR): ") + cmd
                                  : String("MODEM [send] timeout waiting for '>' prompt: ") + cmd);
      break;
    }

    uart->write(data + sentTotal, chunkSize);

    bool ok = false;
    startMs = millis();
    syncWaiting = true;
    syncResponseBuffer = "";
    while (millis() - startMs < timeoutMs)
    {
      pumpWait();
      if (!syncWaiting)
      {
        ok = syncSuccess;
        break;
      }
    }
    syncWaiting = false;

    if (!ok)
    {
      syslog->info(DEBUG_MODEM, String("MODEM [send] no OK after ") + chunkSize + " bytes on link " + linkId);
      break;
    }

    sentTotal += chunkSize;
  }

  if (sentTotal > 0)
  {
    syslog->info(DEBUG_MODEM, String("MODEM [send] wrote ") + sentTotal + "/" + len + " bytes on link " + linkId);
  }
  return sentTotal;
}

int Sim7670G::readSocketData(uint8_t linkId, uint8_t *buf, size_t maxLen, bool secure, uint32_t timeoutMs)
{
  if (!socketConnected[linkId] || buf == nullptr || maxLen == 0 || linkId >= kMaxSockets)
  {
    return 0;
  }

  // Pump the UART so any already-arrived or in-flight pushed data
  // (+CCHRECV: DATA,<mux>,<len> / +CIPRXGET) is captured into the ring buffer.
  readUart();

  // For the plain-TCP path, nudge the modem to report/deliver buffered data.
  // The secure (CCH) path on this firmware pushes data unsolicited and rejects
  // the manual AT+CCHRECV= poll with ERROR, so we do not issue it there.
  if (!secure && socketRxAvailable(linkId) == 0)
  {
    size_t fetchLen = (maxLen > 1024) ? 1024 : maxLen;
    String cmd = String("AT+CIPRXGET=2,") + linkId + "," + fetchLen;
    syslog->info(DEBUG_MODEM, String("MODEM [recv] > ") + cmd);
    uart->print(cmd);
    uart->print("\r\n");
  }

  // Wait briefly for data to land in the ring buffer, then drain it.
  uint32_t startMs = millis();
  while (socketRxAvailable(linkId) == 0 && captureMode == false &&
         (millis() - startMs < timeoutMs))
  {
    pumpWait();
    if (socketRxAvailable(linkId) > 0)
    {
      break;
    }
  }
  // If a capture is mid-flight, let it finish filling the buffer.
  while (captureMode && captureLinkId == linkId && (millis() - startMs < timeoutMs))
  {
    pumpWait();
  }

  size_t n = socketRxDrain(linkId, buf, maxLen);
  if (socketRxAvailable(linkId) == 0)
  {
    socketDataPending[linkId] = false;
  }
  if (n > 0)
  {
    syslog->info(DEBUG_MODEM, String("MODEM [recv] got ") + n + " bytes on link " + linkId);
  }
  return static_cast<int>(n);
}

size_t Sim7670G::socketRxAvailable(uint8_t linkId) const
{
  if (linkId >= kMaxSockets)
  {
    return 0;
  }
  const size_t head = socketRxHead[linkId];
  const size_t tail = socketRxTail[linkId];
  if (tail >= head)
  {
    return tail - head;
  }
  return kSocketRxBufferSize - head + tail;
}

void Sim7670G::socketRxPush(uint8_t linkId, uint8_t byte)
{
  if (linkId >= kMaxSockets)
  {
    return;
  }
  const size_t nextTail = (socketRxTail[linkId] + 1) % kSocketRxBufferSize;
  if (nextTail == socketRxHead[linkId])
  {
    // Buffer full: drop the oldest byte to make room (keep newest data flowing).
    socketRxHead[linkId] = (socketRxHead[linkId] + 1) % kSocketRxBufferSize;
  }
  socketRxBuffer[linkId][socketRxTail[linkId]] = byte;
  socketRxTail[linkId] = nextTail;
}

size_t Sim7670G::socketRxDrain(uint8_t linkId, uint8_t *buf, size_t maxLen)
{
  if (linkId >= kMaxSockets || buf == nullptr)
  {
    return 0;
  }
  size_t n = 0;
  while (n < maxLen && socketRxHead[linkId] != socketRxTail[linkId])
  {
    buf[n++] = socketRxBuffer[linkId][socketRxHead[linkId]];
    socketRxHead[linkId] = (socketRxHead[linkId] + 1) % kSocketRxBufferSize;
  }
  return n;
}

bool Sim7670G::resolveHost(const char *host, IPAddress &out)
{
  if (host == nullptr || host[0] == '\0')
  {
    return false;
  }
  if (out.fromString(host))
  {
    return true;
  }
  if (currentState != STATE_NET_OPEN)
  {
    return false;
  }

  dnsResolved = false;
  dnsResolvedIp = IPAddress(0, 0, 0, 0);

  String cmd = String("AT+CDNSGIP=\"") + host + "\"";
  if (!sendSync(cmd, 5000))
  {
    return false;
  }

  uint32_t startMs = millis();
  while (!dnsResolved && (millis() - startMs < 5000))
  {
    pumpWait();
  }

  if (dnsResolved)
  {
    out = dnsResolvedIp;
    return true;
  }
  return false;
}

#endif // BOARD_WAVESHARE_SIM7670G

