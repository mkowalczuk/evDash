#pragma once

#include "NetTransport.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>

/**
 * WiFi network transport.
 *
 * Implements NetTransport over the ESP32 WiFi stack with zero semantic change
 * from the baseline WiFi upload paths.
 */
class WifiTransport : public NetTransport
{
public:
  WifiTransport() = default;

  bool ready() override
  {
    return (WiFi.status() == WL_CONNECTED);
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
    return (WiFi.hostByName(host, out) == 1);
  }

  int rssi() override
  {
    return WiFi.RSSI();
  }

  const char *name() override
  {
    return "WiFi";
  }

  String ipAddress() override
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      return WiFi.localIP().toString();
    }
    return "";
  }

private:
  WiFiClient httpClient;
  WiFiClientSecure httpSecureClient;
  WiFiClient mqttPlainClient;
  WiFiClientSecure mqttSecureClient;
};
