#ifdef BOARD_WAVESHARE_SIM7670G

#include "Sim7670G.h"
#include "LogSerial.h"
#include "config.h"

static constexpr uint32_t kNetOpenTimeoutMs = 30000;
static constexpr uint32_t kStatusPollMs = 3000;
static constexpr uint32_t kStatusPollConnectedMs = 10000;

// Unsolicited result codes. They can arrive in the middle of a command's reply,
// so they have to be recognised by prefix and kept out of the response buffer.
static const char *const kUrcPrefixes[] = {
    "+CIPRX", "+CMQTTRX", "+CGNSSINFO", "+CGEV", "+NETOPEN", "+CIPEVENT", "+CMQTTCONNLOST", "RDY", "PB DONE"};

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
  nextActionMs = 0;
  netOpenPending = false;
  apnWarned = false;
  modemInfo = Info();
  if (uart != nullptr)
  {
    while (uart->available())
    {
      uart->read();
    }
  }
  currentState = (uart != nullptr) ? STATE_INIT : STATE_OFF;
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
  syslog->info(DEBUG_NET, String("MODEM < ") + line);

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
        syslog->info(DEBUG_NET, String("MODEM: data connection refused, code ") + code);
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
  syslog->info(DEBUG_NET, String("MODEM > ") + inFlight.text);
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
  syslog->info(DEBUG_NET, String("MODEM ! timeout: ") + inFlight.text);
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
  syslog->info(DEBUG_NET, String("MODEM state: ") + stateName() + " -> " +
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
    if (readyCallback)
    {
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
             initStep = 1;
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
           if (response.indexOf("SIM PIN") >= 0 && liveData->settings.modemPin[0] != '\0')
           {
             send(String("AT+CPIN=\"") + liveData->settings.modemPin + "\"", 5000);
             nextActionMs = millis() + 2000;
             return;
           }
           syslog->info(DEBUG_NET, String("MODEM: SIM not ready: ") + response);
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
             initStep = 4;
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
  if (dataEnabled() && registered())
  {
    setState(STATE_REGISTERED);
    return;
  }
  pollStatus();
}

void Sim7670G::stepRegistered()
{
  if (!registered())
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
           syslog->info(DEBUG_NET, String("MODEM: AT+NETOPEN failed: ") + response);
           backoff();
         }
       });
}

void Sim7670G::stepNetOpen()
{
  if (!registered())
  {
    setState(STATE_READY);
    return;
  }
  if (modemInfo.ipAddress.length() == 0)
  {
    stepBusy = true;
    send("AT+IPADDRESS", 3000, [this](bool ok, const String &response)
         {
           stepBusy = false;
           const int colon = response.indexOf(':');
           if (ok && colon >= 0)
           {
             String ip = response.substring(colon + 1);
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
           pollStep = registered() ? 3 : 0;
           nextActionMs = millis() + (registered() ? 200 : pollIntervalMs());
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

#endif // BOARD_WAVESHARE_SIM7670G
