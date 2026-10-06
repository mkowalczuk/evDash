#include "CanProtocol.h"
#include "LiveData.h"
#include <SPI.h>
#include <mcp_can.h>

#if defined(ESP32) || defined(ESP32S3)
#include <driver/twai.h>
#endif

// ============================================================================
// Mcp2515CanDriver
// ============================================================================

Mcp2515CanDriver::Mcp2515CanDriver(uint8_t cs, uint8_t intr)
    : csPin(cs), intPin(intr)
{
}

Mcp2515CanDriver::~Mcp2515CanDriver()
{
  end();
}

bool Mcp2515CanDriver::begin(uint32_t baudRate, uint16_t carType)
{
  can.reset(new MCP_CAN(&SPI, csPin));
  if (!can)
  {
    return false;
  }

  // MCP_STDEXT sets both RX buffers to filter mode
  if (can->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
  {
    return false;
  }

  if (carType == CAR_BMW_I3_2014)
  {
    // Filter to allow only receipt of 0x7xx CAN IDs
    can->init_Mask(0, 0, 0x07000000);
    can->init_Mask(1, 0, 0x07000000);
    for (uint8_t i = 0; i < 6; ++i)
    {
      can->init_Filt(i, 0, 0x06000000);
    }
  }
  else if (carType == CAR_PEUGEOT_E208)
  {
    // PSA e-CMP: accept only diagnostic reply IDs
    can->init_Mask(0, 0, 0x07FF0000);
    can->init_Mask(1, 0, 0x07FF0000);
    can->init_Filt(0, 0, 0x07E80000); // 0x7E8 VIN (mode 09)
    can->init_Filt(1, 0, 0x058F0000); // 0x58F charger / DC-DC
    can->init_Filt(2, 0, 0x06820000); // 0x682 VCU
    can->init_Filt(3, 0, 0x06940000); // 0x694 BMS / TBMU
    can->init_Filt(4, 0, 0x06940000); // 0x694
    can->init_Filt(5, 0, 0x06940000); // 0x694
  }

  if (can->setMode(MCP_NORMAL) != MCP2515_OK)
  {
    return false;
  }

  pinMode(intPin, INPUT);
  return true;
}

void Mcp2515CanDriver::end()
{
  if (can)
  {
    can->setMode(MCP_SLEEP);
    can.reset();
  }
}

uint8_t Mcp2515CanDriver::send(uint32_t id, bool isExtended, uint8_t len, const uint8_t *data)
{
  if (!can)
  {
    return 1;
  }
  return can->sendMsgBuf(id, isExtended ? 1 : 0, len, const_cast<uint8_t *>(data));
}

bool Mcp2515CanDriver::available()
{
  if (!can)
  {
    return false;
  }
  return !digitalRead(intPin);
}

bool Mcp2515CanDriver::read(uint32_t &id, bool &isExtended, uint8_t &len, uint8_t *data)
{
  if (!can)
  {
    return false;
  }
  unsigned long rxId = 0;
  unsigned char rxLen = 0;
  byte stat = can->readMsgBuf(&rxId, &rxLen, data);
  if (stat != CAN_OK)
  {
    return false;
  }
  id = static_cast<uint32_t>(rxId);
  isExtended = ((rxId & 0x80000000) != 0);
  len = rxLen;
  return true;
}

bool Mcp2515CanDriver::sleep()
{
  if (!can)
  {
    return false;
  }
  return (can->setMode(MCP_SLEEP) == MCP2515_OK);
}

bool Mcp2515CanDriver::wake()
{
  if (!can)
  {
    return false;
  }
  return (can->setMode(MCP_NORMAL) == MCP2515_OK);
}

// ============================================================================
// TwaiCanDriver
// ============================================================================

#if defined(ESP32) || defined(ESP32S3)

TwaiCanDriver::TwaiCanDriver(int tx, int rx)
    : txPin(tx), rxPin(rx)
{
}

TwaiCanDriver::~TwaiCanDriver()
{
  end();
}

bool TwaiCanDriver::begin(uint32_t baudRate, uint16_t carType)
{
  if (running)
  {
    return true;
  }

  // TWAI_MODE_NO_ACK: Transmits without requiring ACKs and does not ACK background frames
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(txPin), static_cast<gpio_num_t>(rxPin), TWAI_MODE_NO_ACK);
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 16;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (carType == CAR_BMW_I3_2014)
  {
    // Filter for 0x7xx standard IDs: code 0x700, mask 0x0FF
    f_config.acceptance_code = (0x700 << 21);
    f_config.acceptance_mask = ~(0x700 << 21);
    f_config.single_filter = true;
  }

  if (installed)
  {
    twai_driver_uninstall();
    installed = false;
  }

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK)
  {
    return false;
  }
  installed = true;

  err = twai_start();
  if (err != ESP_OK)
  {
    twai_driver_uninstall();
    installed = false;
    return false;
  }

  running = true;
  return true;
}

void TwaiCanDriver::end()
{
  if (running)
  {
    twai_stop();
    running = false;
  }
  if (installed)
  {
    twai_driver_uninstall();
    installed = false;
  }
}

uint8_t TwaiCanDriver::send(uint32_t id, bool isExtended, uint8_t len, const uint8_t *data)
{
  if (!running)
  {
    return 1;
  }

  twai_message_t msg = {};
  msg.identifier = id & (isExtended ? 0x1FFFFFFF : 0x7FF);
  msg.extd = isExtended ? 1 : 0;
  msg.rtr = 0;
  msg.data_length_code = (len > 8) ? 8 : len;
  memcpy(msg.data, data, msg.data_length_code);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(20));
  return (err == ESP_OK) ? 0 : 1;
}

bool TwaiCanDriver::available()
{
  if (!running)
  {
    return false;
  }

  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK)
  {
    if (status.state == TWAI_STATE_BUS_OFF)
    {
      twai_initiate_recovery();
    }
    return status.msgs_to_rx > 0;
  }
  return false;
}

bool TwaiCanDriver::read(uint32_t &id, bool &isExtended, uint8_t &len, uint8_t *data)
{
  if (!running)
  {
    return false;
  }

  twai_message_t msg;
  esp_err_t err = twai_receive(&msg, 0);
  if (err != ESP_OK)
  {
    return false;
  }

  id = msg.identifier;
  if (msg.extd)
  {
    id |= 0x80000000;
  }
  isExtended = (msg.extd != 0);
  len = msg.data_length_code;
  memcpy(data, msg.data, len);
  return true;
}

bool TwaiCanDriver::sleep()
{
  if (!running)
  {
    return false;
  }
  esp_err_t err = twai_stop();
  if (err == ESP_OK)
  {
    running = false;
    return true;
  }
  return false;
}

bool TwaiCanDriver::wake()
{
  if (running)
  {
    return true;
  }
  if (!installed)
  {
    return false;
  }
  esp_err_t err = twai_start();
  if (err == ESP_OK)
  {
    running = true;
    return true;
  }
  return false;
}

#endif // ESP32 || ESP32S3
