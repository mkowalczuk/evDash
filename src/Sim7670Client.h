#pragma once

#ifdef BOARD_WAVESHARE_SIM7670G

#include <Arduino.h>
#include <WiFiClient.h>

class Sim7670G;

/**
 * Arduino WiFiClient implementation over SIM7670G sockets.
 *
 * Implements plain TCP (via AT+CIP...) or TLS/SSL (via AT+CCH... / AT+CSSLCFG).
 * Connect, write, read and stop operate through the modem driver while preserving
 * background NMEA parsing on the shared AT UART.
 */
class Sim7670Client : public WiFiClient
{
public:
  Sim7670Client(Sim7670G *modem, uint8_t linkId = 0, bool secure = false);
  virtual ~Sim7670Client();

  int connect(IPAddress ip, uint16_t port) override;
  int connect(IPAddress ip, uint16_t port, int32_t timeout_ms);
  int connect(const char *host, uint16_t port) override;
  int connect(const char *host, uint16_t port, int32_t timeout_ms);
  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buf, size_t size) override;
  int available() override;
  int read() override;
  int read(uint8_t *buf, size_t size) override;
  int peek() override;
  void flush() override;
  void stop() override;
  uint8_t connected() override;
  operator bool() override;

  int setTimeout(uint32_t seconds);

protected:
  Sim7670G *modem = nullptr;
  uint8_t linkId = 0;
  bool isSecure = false;
  uint32_t socketTimeoutMs = 5000;

  static constexpr size_t kRxBufferSize = 1024;
  uint8_t rxBuffer[kRxBufferSize];
  size_t rxHead = 0;
  size_t rxTail = 0;

  size_t bufferAvailable() const;
  int bufferRead();
  void bufferClear();
  void fillBuffer();
};

/**
 * TLS/SSL variant of Sim7670Client.
 *
 * TLS terminates inside the SIM7670G modem, saving ~40-60 KB of ESP32 heap.
 * Provides setInsecure() and setHandshakeTimeout() for compatibility with WiFiClientSecure.
 */
class Sim7670ClientSecure : public Sim7670Client
{
public:
  Sim7670ClientSecure(Sim7670G *modem, uint8_t linkId = 0)
    : Sim7670Client(modem, linkId, true)
  {
  }

  void setInsecure() { (void)isSecure; }
  void setHandshakeTimeout(uint32_t sec) { (void)sec; }
};

#endif // BOARD_WAVESHARE_SIM7670G
