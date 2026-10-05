#ifdef BOARD_WAVESHARE_SIM7670G

#include "Sim7670Client.h"
#include "Sim7670G.h"

Sim7670Client::Sim7670Client(Sim7670G *modem, uint8_t linkId, bool secure)
  : modem(modem), linkId(linkId), isSecure(secure)
{
  bufferClear();
}

Sim7670Client::~Sim7670Client()
{
  stop();
}

int Sim7670Client::setTimeout(uint32_t seconds)
{
  if (seconds > 100)
  {
    socketTimeoutMs = seconds;
  }
  else
  {
    socketTimeoutMs = seconds * 1000;
  }
  WiFiClient::setTimeout(seconds);
  return 0;
}

int Sim7670Client::connect(IPAddress ip, uint16_t port)
{
  return connect(ip.toString().c_str(), port);
}

int Sim7670Client::connect(IPAddress ip, uint16_t port, int32_t timeout_ms)
{
  uint32_t prevTimeout = socketTimeoutMs;
  if (timeout_ms > 0)
  {
    socketTimeoutMs = static_cast<uint32_t>(timeout_ms);
  }
  int res = connect(ip, port);
  socketTimeoutMs = prevTimeout;
  return res;
}

int Sim7670Client::connect(const char *host, uint16_t port)
{
  if (modem == nullptr)
  {
    return 0;
  }
  bufferClear();
  return modem->openSocket(linkId, host, port, isSecure, socketTimeoutMs) ? 1 : 0;
}

int Sim7670Client::connect(const char *host, uint16_t port, int32_t timeout_ms)
{
  uint32_t prevTimeout = socketTimeoutMs;
  if (timeout_ms > 0)
  {
    socketTimeoutMs = static_cast<uint32_t>(timeout_ms);
  }
  int res = connect(host, port);
  socketTimeoutMs = prevTimeout;
  return res;
}

size_t Sim7670Client::write(uint8_t b)
{
  return write(&b, 1);
}

size_t Sim7670Client::write(const uint8_t *buf, size_t size)
{
  if (modem == nullptr || buf == nullptr || size == 0)
  {
    return 0;
  }
  return modem->sendSocketData(linkId, buf, size, isSecure, socketTimeoutMs);
}

int Sim7670Client::available()
{
  if (bufferAvailable() > 0)
  {
    return bufferAvailable();
  }
  if (!connected())
  {
    return 0;
  }
  fillBuffer();
  return bufferAvailable();
}

int Sim7670Client::read()
{
  if (available() <= 0)
  {
    return -1;
  }
  return bufferRead();
}

int Sim7670Client::read(uint8_t *buf, size_t size)
{
  if (buf == nullptr || size == 0)
  {
    return 0;
  }
  size_t bytesRead = 0;
  while (bytesRead < size)
  {
    if (bufferAvailable() == 0)
    {
      fillBuffer();
      if (bufferAvailable() == 0)
      {
        break;
      }
    }
    buf[bytesRead++] = static_cast<uint8_t>(bufferRead());
  }
  return bytesRead;
}

int Sim7670Client::peek()
{
  if (available() <= 0)
  {
    return -1;
  }
  return rxBuffer[rxHead];
}

void Sim7670Client::flush()
{
}

void Sim7670Client::stop()
{
  if (modem != nullptr)
  {
    modem->closeSocket(linkId, isSecure, 1000);
  }
  bufferClear();
}

uint8_t Sim7670Client::connected()
{
  if (bufferAvailable() > 0)
  {
    return 1;
  }
  return (modem != nullptr && modem->isSocketConnected(linkId)) ? 1 : 0;
}

Sim7670Client::operator bool()
{
  return connected() != 0;
}

size_t Sim7670Client::bufferAvailable() const
{
  if (rxTail >= rxHead)
  {
    return rxTail - rxHead;
  }
  return kRxBufferSize - rxHead + rxTail;
}

int Sim7670Client::bufferRead()
{
  if (rxHead == rxTail)
  {
    return -1;
  }
  uint8_t b = rxBuffer[rxHead];
  rxHead = (rxHead + 1) % kRxBufferSize;
  return b;
}

void Sim7670Client::bufferClear()
{
  rxHead = 0;
  rxTail = 0;
}

void Sim7670Client::fillBuffer()
{
  if (modem == nullptr || !modem->isSocketConnected(linkId))
  {
    return;
  }
  uint8_t temp[256];
  int n = modem->readSocketData(linkId, temp, sizeof(temp), isSecure, 1000);
  for (int i = 0; i < n; i++)
  {
    size_t nextTail = (rxTail + 1) % kRxBufferSize;
    if (nextTail != rxHead)
    {
      rxBuffer[rxTail] = temp[i];
      rxTail = nextTail;
    }
  }
}

#endif // BOARD_WAVESHARE_SIM7670G
