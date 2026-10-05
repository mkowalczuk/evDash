#pragma once

#include <Arduino.h>
#include <WiFiClient.h>
#include <IPAddress.h>

/**
 * Abstract network transport interface.
 *
 * Decouples upload channels (MQTT, ABRP, Traccar, contribute, pairing)
 * from the underlying physical link (WiFi vs Cellular).
 */
class NetTransport
{
public:
  virtual ~NetTransport() = default;

  // True if the transport has an active, usable data connection.
  virtual bool ready() = 0;

  // Return a client for plain TCP connections.
  virtual WiFiClient *plainClient() = 0;

  // Return a client for TLS/SSL connections.
  virtual WiFiClient *secureClient() = 0;

  // Return a client dedicated to MQTT (allows concurrent MQTT + HTTP).
  virtual WiFiClient *mqttClient(bool secure) { return secure ? secureClient() : plainClient(); }

  // Resolve host name to IP address.
  virtual bool resolve(const char *host, IPAddress &out) = 0;

  // Signal strength in dBm (negative value, or 0 if unknown).
  virtual int rssi() = 0;

  // Transport name for diagnostics (e.g. "WiFi", "Cellular").
  virtual const char *name() = 0;

  // Current IP address as string (or empty if not connected).
  virtual String ipAddress() { return ""; }
};
