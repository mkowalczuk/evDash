#pragma once

#include <Arduino.h>
#include <cstdint>
#include <memory>
#include <vector>

// Forward declarations
class MCP_CAN;

/**
 * Generic CAN frame representation
 */
struct CanMessage
{
  uint32_t id = 0;
  bool isExtended = false;
  bool isRtr = false;
  uint8_t len = 0;
  uint8_t data[8] = {0};
};

/**
 * Abstract CAN driver interface across MCP2515 and ESP32 internal TWAI
 */
class CanDriver
{
public:
  virtual ~CanDriver() = default;

  virtual bool begin(uint32_t baudRate, uint16_t carType) = 0;
  virtual void end() = 0;
  virtual uint8_t send(uint32_t id, bool isExtended, uint8_t len, const uint8_t *data) = 0;
  virtual bool available() = 0;
  virtual bool read(uint32_t &id, bool &isExtended, uint8_t &len, uint8_t *data) = 0;
  virtual bool sleep() = 0;
  virtual bool wake() = 0;
  virtual const char *name() const = 0;
};

/**
 * MCP2515 SPI CAN controller driver (used on M5Stack Core2 / CoreS3)
 */
class Mcp2515CanDriver : public CanDriver
{
private:
  uint8_t csPin;
  uint8_t intPin;
  std::unique_ptr<MCP_CAN> can;

public:
  Mcp2515CanDriver(uint8_t cs, uint8_t intr);
  ~Mcp2515CanDriver() override;

  bool begin(uint32_t baudRate, uint16_t carType) override;
  void end() override;
  uint8_t send(uint32_t id, bool isExtended, uint8_t len, const uint8_t *data) override;
  bool available() override;
  bool read(uint32_t &id, bool &isExtended, uint8_t &len, uint8_t *data) override;
  bool sleep() override;
  bool wake() override;
  const char *name() const override { return "MCP2515"; }
};

#if defined(ESP32) || defined(ESP32S3)
/**
 * ESP32 / ESP32-S3 internal TWAI (Two-Wire Automotive Interface) controller driver
 */
class TwaiCanDriver : public CanDriver
{
private:
  int txPin;
  int rxPin;
  bool installed = false;
  bool running = false;

public:
  TwaiCanDriver(int tx, int rx);
  ~TwaiCanDriver() override;

  bool begin(uint32_t baudRate, uint16_t carType) override;
  void end() override;
  uint8_t send(uint32_t id, bool isExtended, uint8_t len, const uint8_t *data) override;
  bool available() override;
  bool read(uint32_t &id, bool &isExtended, uint8_t &len, uint8_t *data) override;
  bool sleep() override;
  bool wake() override;
  const char *name() const override { return "TWAI"; }
};
#endif
