#pragma once

#include "LiveData.h"

class CommInterface; // Forward declaration

class CarInterface
{

protected:
  LiveData *liveData;
  CommInterface *commInterface;

public:
  void setLiveData(LiveData *pLiveData);
  void setCommInterface(CommInterface *pCommInterface);
  virtual void activateCommandQueue();
  virtual bool commandAllowed() { return true; }
  virtual void parseRowMerged();
  virtual void loadTestData();
  // Whether this car ever provides a value for the given HA MQTT telemetry
  // object_id (e.g. "inverter_temp", "charger_power"). The base reports every
  // field as supported, so a car that does not override this keeps the full
  // entity set. A car whose parser never populates a field overrides this to
  // return false for that object_id, and the common publisher then neither
  // registers nor publishes it — keeping car-specific knowledge in the car.
  virtual bool supportsHaTelemetry(const char *objectId) { return true; }
  //
  virtual void testHandler(const String &cmd);
};
