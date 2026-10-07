#pragma once

#ifdef BOARD_WAVESHARE_SIM7670G

#include <Arduino.h>
#include <HardwareSerial.h>
#include <deque>
#include <functional>
#include "LiveData.h"

/**
 * Non-blocking driver for the SIM7670G LTE Cat-1 modem on an AT UART.
 *
 * evDash runs everything from one cooperative main loop, so a modem driver that
 * waits for a reply with delay() would stall logging, CAN handling and the web
 * server for as long as the modem takes to answer. This driver never waits: a
 * command is queued, written when it reaches the head of the queue, and its
 * completion callback runs from loop() once a final result code (or a timeout)
 * arrives. Only one command is ever in flight.
 *
 * The same UART also carries the GNSS NMEA stream (the TF card owns the pins the
 * module's dedicated GNSS UART would need), so every received line is
 * demultiplexed: '$' lines go to the NMEA sink, unsolicited result codes go to
 * the URC handler, and everything else is the reply to the command in flight.
 *
 * On top of the command engine sits a small state machine that brings the modem
 * from power-up to an open data connection:
 *
 *   OFF -> INIT -> READY -> REGISTERED -> NET_OPEN
 *
 * INIT checks the SIM and reads the identity, READY waits for the network to
 * accept the SIM, REGISTERED opens the packet data connection and NET_OPEN means
 * the modem holds an IP address. Losing the network walks the state back down.
 */
class Sim7670G
{
public:
  enum State : uint8_t
  {
    STATE_OFF = 0,
    STATE_INIT,
    STATE_READY,
    STATE_REGISTERED,
    STATE_NET_OPEN
  };

  // Called with the result of a command. 'response' holds the lines received
  // before the final result code, joined with '\n', without the result code.
  using ResponseCallback = std::function<void(bool ok, const String &response)>;
  // Receives every character of a '$' (NMEA) line, including its line ending.
  using NmeaSink = std::function<void(char)>;
  // Receives unsolicited result codes the driver does not consume itself.
  using UrcHandler = std::function<void(const String &line)>;
  // Called once each time INIT completes, i.e. when the modem is known to be alive.
  using ReadyCallback = std::function<void()>;
  // Invoked repeatedly from inside the driver's blocking wait loops (sendSync,
  // openSocket, sendSocketData, readSocketData). Lets the caller keep latency-
  // sensitive work alive (GPS NMEA, serial console) and yield the CPU to
  // FreeRTOS/ESP-IDF while a slow AT command is in flight, instead of freezing
  // the whole cooperative main loop. Must NOT touch the modem UART itself.
  using YieldHook = std::function<void()>;

  struct Info
  {
    String imei;
    String firmware;
    String operatorName;
    String ipAddress;
    int rssiDbm = 0;        // 0 when unknown
    int registration = -1;  // 3GPP <stat>: 0 none, 1 home, 2 searching, 3 denied, 5 roaming
    bool simReady = false;
  };

  void begin(HardwareSerial *uart, LiveData *data);
  void loop();

  // Queue one AT command. Returns false when the queue is full.
  bool send(const String &command, uint32_t timeoutMs, ResponseCallback callback = nullptr);
  // Drop everything queued and in flight, and restart the state machine at INIT.
  void reset();

  void setNmeaSink(NmeaSink sink) { nmeaSink = sink; }
  void setUrcHandler(UrcHandler handler) { urcHandler = handler; }
  void setReadyCallback(ReadyCallback callback) { readyCallback = callback; }
  void setYieldHook(YieldHook hook) { yieldHook = hook; }

  static constexpr uint8_t kMaxSockets = 2; // link 0: HTTP/API, link 1: MQTT

  bool openSocket(uint8_t linkId, const char *host, uint16_t port, bool secure, uint32_t timeoutMs);
  bool closeSocket(uint8_t linkId, bool secure, uint32_t timeoutMs);
  size_t sendSocketData(uint8_t linkId, const uint8_t *data, size_t len, bool secure, uint32_t timeoutMs);
  int readSocketData(uint8_t linkId, uint8_t *buf, size_t maxLen, bool secure, uint32_t timeoutMs);
  bool isSocketConnected(uint8_t linkId) const { return (linkId < kMaxSockets) && socketConnected[linkId]; }
  bool isSocketDataPending(uint8_t linkId) const { return (linkId < kMaxSockets) && socketDataPending[linkId]; }
  bool resolveHost(const char *host, IPAddress &out);
  bool sendSync(const String &command, uint32_t timeoutMs, String *outResponse = nullptr);
  // Time over cellular. ESP32 SNTP runs on the WiFi lwIP stack and cannot use
  // the modem, so the modem's own NTP client (AT+CNTP) / NITZ clock is used.
  // requestNtp() triggers an async modem NTP update; readClockUtc() reads
  // AT+CCLK? and returns true with UTC epoch when the clock looks valid.
  bool requestNtp(const char *server);
  bool readClockUtc(time_t &utcOut);

  State state() const { return currentState; }
  const char *stateName() const;
  bool responding() const { return everResponded; }
  bool dataReady() const { return currentState == STATE_NET_OPEN; }
  const Info &info() const { return modemInfo; }
  uint32_t timeoutCount() const { return totalTimeouts; }
  uint32_t commandCount() const { return totalCommands; }

private:
  struct Command
  {
    String text;
    uint32_t timeoutMs;
    ResponseCallback callback;
  };

  static constexpr size_t kMaxQueue = 8;
  static constexpr size_t kMaxLine = 256;
  static constexpr size_t kMaxResponse = 512;

  HardwareSerial *uart = nullptr;
  LiveData *liveData = nullptr;
  NmeaSink nmeaSink;
  UrcHandler urcHandler;
  ReadyCallback readyCallback;
  YieldHook yieldHook;

  std::deque<Command> queue;
  bool commandInFlight = false;
  Command inFlight;
  uint32_t inFlightStartMs = 0;
  String responseBuffer;
  String lineBuffer;
  bool lineIsNmea = false;
  bool atLineStart = true;

  State currentState = STATE_OFF;
  Info modemInfo;
  bool everResponded = false;
  uint32_t totalTimeouts = 0;
  uint32_t totalCommands = 0;

  // State machine bookkeeping. 'stepBusy' is true from the moment a step queues
  // its command until that command's callback runs, so a step is never issued twice.
  bool stepBusy = false;
  uint8_t initStep = 0;
  uint8_t pollStep = 0;
  // AT+COPS? is slow (the modem queries the network for the operator name, often
  // 1-3 s) and holds the shared UART, delaying MQTT sends. The operator name is
  // effectively static, so it is only refreshed every kCopsPollEvery cycles.
  uint16_t copsPollCounter = 0;
  static constexpr uint16_t kCopsPollEvery = 30;
  uint32_t nextActionMs = 0;
  bool netOpenPending = false;
  uint32_t netOpenSentMs = 0;
  bool apnWarned = false;
  bool wasDataEnabled = false;
  bool readyCallbackCalled = false;
  bool pinAttempted = false;

  // Socket state
  bool socketConnected[kMaxSockets] = {false, false};
  bool socketSecure[kMaxSockets] = {false, false};
  bool socketDataPending[kMaxSockets] = {false, false};
  bool socketOpenDone[kMaxSockets] = {false, false};
  int socketOpenResult[kMaxSockets] = {-1, -1};

  // Per-socket receive ring buffers.
  //
  // The SIM7670G firmware here pushes received socket data unsolicited as
  //   +CCHRECV: DATA,<mux>,<len>\r\n<len raw bytes>
  // and rejects the manual AT+CCHRECV=<mux>,<len> poll with ERROR. So the driver
  // captures the pushed bytes into these buffers whenever readUart() runs, and
  // readSocketData() drains from them instead of polling the modem.
  static constexpr size_t kSocketRxBufferSize = 2048;
  uint8_t socketRxBuffer[kMaxSockets][kSocketRxBufferSize];
  size_t socketRxHead[kMaxSockets] = {0, 0};
  size_t socketRxTail[kMaxSockets] = {0, 0};
  // Active while consuming the raw byte payload of a +CCHRECV: DATA / +CIPRXGET
  // push into socketRxBuffer[captureLinkId].
  bool captureMode = false;
  uint8_t captureLinkId = 0;
  size_t captureRemaining = 0;

  size_t socketRxAvailable(uint8_t linkId) const;
  void socketRxPush(uint8_t linkId, uint8_t byte);
  size_t socketRxDrain(uint8_t linkId, uint8_t *buf, size_t maxLen);

  // Synchronous command execution support
  bool syncWaiting = false;
  bool syncSuccess = false;
  String syncResponseBuffer;
  bool promptWaiting = false;
  // Set when an ERROR/CME ERROR line arrives while waiting for a '>' send prompt,
  // so sendSocketData can fail fast instead of blocking until timeout.
  bool promptError = false;

  // Raw socket receive state
  bool rawRxMode = false;
  uint8_t *rawRxTarget = nullptr;
  size_t rawRxTargetMax = 0;
  size_t rawRxExpected = 0;
  size_t rawRxReceived = 0;

  // DNS resolution state
  bool dnsResolved = false;
  IPAddress dnsResolvedIp;

  // TLS server-authentication (authmode 1) requires a CA certificate in the
  // modem's store. Uploaded lazily on first secure connect; the modem keeps it
  // in flash, so this just tracks whether we have verified/uploaded it this run.
  bool caCertUploaded = false;
  bool ensureCaCert();

  void readUart();
  // One iteration of a blocking wait: pumps the UART and invokes the yield hook
  // so the rest of the system keeps running while an AT command is in flight.
  void pumpWait();
  void handleLine(const String &line);
  bool isFinalResult(const String &line, bool &ok) const;
  bool isUrc(const String &line) const;
  void handleUrc(const String &line);
  void startNextCommand();
  void completeCommand(bool ok, bool responded);
  void checkTimeout();

  void tick();
  void stepInit();
  void stepReady();
  void stepRegistered();
  void stepNetOpen();
  void pollStatus();
  void setState(State next);
  void backoff();
  uint32_t retryBackoffMs() const;
  bool dataEnabled() const;
  bool apnConfigured() const;
  bool registered() const;
  uint32_t pollIntervalMs() const;

  static int parseRegistration(const String &response);
  static int parseSignalDbm(const String &response);
  static String firstQuoted(const String &text);
  static String firstLine(const String &text);
};

#endif // BOARD_WAVESHARE_SIM7670G
