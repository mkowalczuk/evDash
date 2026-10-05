#pragma once

#ifdef BOARD_WAVESHARE_SIM7670G

#include "NetTransport.h"
#include "Sim7670Client.h"
#include "Sim7670G.h"

/**
 * Cellular network transport for SIM7670G.
 *
 * Implements NetTransport over the SIM7670G modem, providing socket clients
 * with modem-side TLS.
 */
class ModemTransport : public NetTransport
{
public:
  explicit ModemTransport(Sim7670G *modem)
    : modem(modem),
      httpClient(modem, 0, false),
      httpSecureClient(modem, 0),
      mqttPlainClient(modem, 1, false),
      mqttSecureClient(modem, 1)
  {
  }

  bool ready() override
  {
    return (modem != nullptr && modem->dataReady());
  }

  WiFiClient *plainClient() override
  {
    return &httpClient;
  }

  WiFiClient *secureClient() override
  {
    httpSecureClient.setInsecure();
    return &httpSecureClient;
  }

  WiFiClient *mqttClient(bool secure) override
  {
    if (secure)
    {
      mqttSecureClient.setInsecure();
      return &mqttSecureClient;
    }
    return &mqttPlainClient;
  }

  bool resolve(const char *host, IPAddress &out) override
  {
    if (modem == nullptr || !modem->dataReady() || host == nullptr)
    {
      return false;
    }
    return modem->resolveHost(host, out);
  }

  int rssi() override
  {
    return (modem != nullptr) ? modem->info().rssiDbm : 0;
  }

  const char *name() override
  {
    return "Cellular";
  }

  String ipAddress() override
  {
    return (modem != nullptr) ? modem->info().ipAddress : "";
  }

private:
  Sim7670G *modem = nullptr;
  Sim7670Client httpClient;
  Sim7670ClientSecure httpSecureClient;
  Sim7670Client mqttPlainClient;
  Sim7670ClientSecure mqttSecureClient;
};

#endif // BOARD_WAVESHARE_SIM7670G
