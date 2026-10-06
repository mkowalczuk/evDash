#pragma once

#include "LiveData.h"
#include "CommInterface.h"
#include "CanProtocol.h"

#include <memory>
#include <vector>
#include <unordered_map>

class CommObd2Can : public CommInterface
{
protected:
#ifdef COMMU_INT_PIN
  const uint8_t pinCanInt = COMMU_INT_PIN;
  const uint8_t pinCanCs = COMMU_CS_PIN;
#else
  const uint8_t pinCanInt = 0;
  const uint8_t pinCanCs = 0;
#endif
  std::unique_ptr<CanDriver> canDriver;
  long unsigned int rxId = 0;
  unsigned char rxLen = 0;
  uint8_t rxBuf[32] = {0};
  bool sentCanData = false;
  int16_t rxRemaining = 0; // Remaining bytes to complete message, signed is ok
  uint8_t requestFramesCount = 0;
  uint16_t rxSequenceRow = 0; // receive-order row key for dataRows (ISO-TP 4-bit index wraps)
  char msgString[128] = {0};  // Array to store serial string
  uint32_t lastPid = 0;
  unsigned long lastDataSent = 0;
  long errorsComm = 0;
  std::vector<uint8_t> mergedData;
  std::unordered_map<uint16_t, std::vector<uint8_t>> dataRows;
  bool bResponseProcessed = false;
  static constexpr uint32_t kCanReconnectGraceMs = 20000;
  uint32_t canReconnectAllowedAtMs = 0;

  enum class enFrame_t
  {
    single = 0,
    first = 1,
    consecutive = 2,
    unknown = 9
  };

public:
  virtual ~CommObd2Can() = default;

  void connectDevice() override;
  void disconnectDevice() override;
  void scanDevices() override;
  void mainLoop() override;
  void executeCommand(String cmd) override;

protected:
  void sendPID(const uint32_t pid, const String &cmd) override;
  void sendFlowControlFrame();
  uint8_t receivePID() override;
  enFrame_t getFrameType(const uint8_t firstByte);
  bool processFrameBytes();
  bool processFrame();
  void processMergedResponse();
  void suspendDevice() override;
  void resumeDevice() override;
};
