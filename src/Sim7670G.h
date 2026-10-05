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
  uint32_t nextActionMs = 0;
  bool netOpenPending = false;
  uint32_t netOpenSentMs = 0;
  bool apnWarned = false;

  void readUart();
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
