#pragma once

#include <Arduino.h>

class WiFiClient;

namespace Traccar
{
  bool sendPosition(const char *serverHost,
                    uint16_t serverPort,
                    const String &deviceId,
                    time_t timestamp,
                    double latitude,
                    double longitude,
                    float speedKmh,
                    float altitudeMeters,
                    float headingDeg,
                    float socPercent,
                    bool charging,
                    int &outHttpCode,
                    WiFiClient *client = nullptr);
}
