#include "ble_compat.h"
#include "CommObd2Ble4.h"
#include "BoardInterface.h"
#include "LiveData.h"

CommObd2Ble4 *commObj;
BoardInterface *boardObj;
LiveData *liveDataObj;

namespace
{
  constexpr uint32_t kBleConnectRetryBaseMs = 3000;
  constexpr uint32_t kBleConnectRetryMaxMs = 15000;
  constexpr uint32_t kBleCmdTimeoutMs = 4000;
  constexpr uint8_t kBleMaxStallRetries = 3;
  uint8_t bleStallRetryCount = 0;

  // BLE connect/disconnect events are signalled from the BLE callback task and the
  // user-facing message is rendered later from the loop task (TFT/sprite work must
  // not run in BLE host/controller context). 0 = none, 1 = connected, 2 = disconnected.
  volatile int8_t bleConnEventPending = 0;

  bool hasConfiguredBleMac(const char *value)
  {
    return value != nullptr &&
           value[0] != '\0' &&
           strcmp(value, "00:00:00:00:00:00") != 0 &&
           strcmp(value, "not_set") != 0 &&
           strcmp(value, "empty") != 0;
  }

  uint32_t calcConnectRetryDelayMs(uint8_t failCount)
  {
    uint32_t backoffStep = failCount;
    if (backoffStep > 4)
      backoffStep = 4;

    uint32_t delayMs = kBleConnectRetryBaseMs * (backoffStep + 1);
    if (delayMs > kBleConnectRetryMaxMs)
      delayMs = kBleConnectRetryMaxMs;
    return delayMs;
  }
} // namespace

/**
  BLE callbacks
*/
class MyClientCallback : public BLEClientCallbacks
{

  // On BLE connect. Runs in BLE callback task context — only touch flags here,
  // the display message is rendered later from the loop task (see mainLoop).
  void onConnect(BLEClient *pclient)
  {
    syslog->println("onConnect");
    liveDataObj->commConnected = true;
    bleConnEventPending = 1;
  }

  // On BLE disconnect. Clear the stale characteristic pointers so executeCommand
  // can't dereference a freed handle, and re-arm obd2ready so mainLoop reconnects
  // (mainLoop early-returns while suspended, so this won't fight sleep/shutdown).
  void onDisconnect(BLEClient *pclient)
  {
    syslog->println("onDisconnect");
    liveDataObj->commConnected = false;
    liveDataObj->pRemoteCharacteristic = nullptr;
    liveDataObj->pRemoteCharacteristicWrite = nullptr;
    liveDataObj->obd2ready = true;
    bleConnEventPending = 2;
  }
};

/**
   Scan for BLE servers and find the selected adapter by MAC address.
*/
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{

  // Called for each advertising BLE server.
  // NimBLE passes the device by pointer, Bluedroid by value; normalize to a pointer.
#ifdef EVDASH_USE_NIMBLE
  void onResult(BLEAdvertisedDevice *advertisedDevice)
  {
#else
  void onResult(BLEAdvertisedDevice advertisedDeviceVal)
  {
    BLEAdvertisedDevice *advertisedDevice = &advertisedDeviceVal;
#endif

    syslog->print("BLE advertised device found: ");
    syslog->println(advertisedDevice->toString().c_str());
    syslog->println(advertisedDevice->getAddress().toString().c_str());

    // Add to the device list (maximum of 9 devices allowed for now)
    String tmpStr;

    if (liveDataObj->scanningDeviceIndex < 10)
    { // Check if the device name and manufacturer data are empty, skip adding to the list
      for (uint16_t i = 0; i < liveDataObj->menuItemsCount; ++i)
      {
        if (liveDataObj->menuItems[i].id == 10001 + liveDataObj->scanningDeviceIndex)
        {
          String deviceName = advertisedDevice->getName().c_str();
          String manufacturerData = advertisedDevice->getManufacturerData().c_str();

          if (deviceName.isEmpty() && manufacturerData.isEmpty())
          {
            syslog->println("Skipping device with empty name and manufacturer data.");
            continue;
          }

          // Populate the string with name, manufacturer, and MAC address
          tmpStr = deviceName;
          if (!manufacturerData.isEmpty())
          {
            tmpStr += ", ";
            tmpStr += manufacturerData;
          }
          tmpStr += ", ";
          tmpStr += advertisedDevice->getAddress().toString().c_str();

          // Save to menuItems
          tmpStr.toCharArray(liveDataObj->menuItems[i].title, sizeof(liveDataObj->menuItems[i].title));
          tmpStr = advertisedDevice->getAddress().toString().c_str();
          tmpStr.toCharArray(liveDataObj->menuItems[i].obdMacAddress, sizeof(liveDataObj->menuItems[i].obdMacAddress));
        }
      }
      liveDataObj->scanningDeviceIndex++;
    }

    if (strcmp(advertisedDevice->getAddress().toString().c_str(), liveDataObj->settings.obdMacAddress) == 0)
    {
      if (advertisedDevice->haveServiceUUID() &&
          advertisedDevice->isAdvertisingService(BLEUUID(liveDataObj->settings.serviceUUID)))
      {
        syslog->println("Stop scanning. Found target MAC + matching service UUID.");
      }
      else
      {
        // Some adapters do not advertise a service UUID during scan or use a different one.
        syslog->println("Stop scanning. Found target MAC (service UUID differs/missing, using auto-detect).");
      }
      BLEDevice::getScan()->stop();
      liveDataObj->foundMyBleDevice = new BLEAdvertisedDevice(*advertisedDevice);
    }
  }
};

uint32_t PIN = 1234;

/**
  BLE Security
*/
class MySecurity : public BLESecurityCallbacks
{

  uint32_t onPassKeyRequest()
  {
    syslog->printf("Pairing password: %d \r\n", PIN);
    return PIN;
  }

  void onPassKeyNotify(uint32_t pass_key)
  {
    syslog->printf("onPassKeyNotify\r\n");
  }

  bool onConfirmPIN(uint32_t pass_key)
  {
    syslog->printf("onConfirmPIN\r\n");
    return true;
  }

  bool onSecurityRequest()
  {
    syslog->printf("onSecurityRequest\r\n");
    return true;
  }

#ifdef EVDASH_USE_NIMBLE
  void onAuthenticationComplete(ble_gap_conn_desc *desc)
  {
    if (desc->sec_state.encrypted || desc->sec_state.bonded)
#else
  void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)
  {
    if (auth_cmpl.success)
#endif
    {
      syslog->printf("onAuthenticationComplete\r\n");
    }
    else
    {
      syslog->println("Auth failure. Incorrect PIN?");
      liveDataObj->obd2ready = false;
    }
  }
};

/**
   Ble notification callback
*/
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{

  char ch;

  // Parse multiframes to single response. responseRow deliberately persists across
  // notifications: an ELM327 line split over two BLE packets is completed by the
  // next packet instead of being dropped (frozen 620101/power values, issue #107).
  // It is cleared in executeCommand() before each new command.
  for (size_t i = 0; i < length; i++)
  {
    ch = pData[i];
    if (ch == '\r' || ch == '\n' || ch == '\0')
    {
      if (liveDataObj->responseRow != "")
        commObj->parseResponse();
      liveDataObj->responseRow = "";
    }
    else
    {
      liveDataObj->responseRow += ch;
      if (liveDataObj->responseRow == ">")
      {
        bleStallRetryCount = 0;
        if (liveDataObj->responseRowMerged != "")
        {
          syslog->infoNolf(DEBUG_COMM, "merged: ");
          syslog->info(DEBUG_COMM, liveDataObj->responseRowMerged);
          commObj->parseRowMerged();
        }
        liveDataObj->responseRowMerged = "";
        liveDataObj->canSendNextAtCommand = true;
      }
    }
  }
}

/**
   Connect ble4 adapter
*/
void CommObd2Ble4::connectDevice()
{

  commObj = this;
  liveDataObj = liveData;
  boardObj = board;

  syslog->println("BLE4 connectDevice");
  connectFailCount = 0;
  nextConnectRetryMs = 0;
  liveData->commConnected = false;
  liveData->obd2ready = true;
  liveData->pRemoteCharacteristic = nullptr;
  liveData->pRemoteCharacteristicWrite = nullptr;
  liveData->foundMyBleDevice = nullptr;

  // Start BLE connection. NimBLE releases classic-BT controller memory itself;
  // Bluedroid needs the explicit release to reclaim heap.
#ifndef EVDASH_USE_NIMBLE
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we have detected a new device.
  // Specify that we want active scanning and start the scan to run for 10 seconds.
  syslog->println("Setup BLE scan");
  liveData->pBLEScan = BLEDevice::getScan();
  liveData->pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  liveData->pBLEScan->setInterval(1349);
  liveData->pBLEScan->setWindow(449);
  liveData->pBLEScan->setActiveScan(true);

  // Avoid 10-second blocking scan during boot.
  // Connection is attempted directly by configured MAC in mainLoop().
  if (hasConfiguredBleMac(liveData->settings.obdMacAddress) && !board->skipAdapterScan())
  {
    syslog->println("Boot BLE scan skipped (non-blocking startup connect by MAC).");
  }
}

/**
   Disconnect device
*/
void CommObd2Ble4::disconnectDevice()
{

  syslog->println("COMM disconnectDevice");
#ifdef EVDASH_USE_NIMBLE
  BLEDevice::deinit(false);
#else
  btStop();
#endif
}

/**
   Scan device list, from menu
*/
void CommObd2Ble4::scanDevices()
{

  syslog->println("COMM scanDevices");
  startBleScan();
}

///////////////////////////////////

/**
   Start ble scan
*/
void CommObd2Ble4::startBleScan()
{

  liveData->foundMyBleDevice = NULL;
  liveData->scanningDeviceIndex = 0;
  board->displayMessage(" > Scanning BLE4 devices", "10sec.or hold middle&RST");

  syslog->printf("Total/free heap: %i/%i-%i\n", ESP.getHeapSize(), ESP.getFreeHeap(), heap_caps_get_free_size(MALLOC_CAP_8BIT));

  // Start scanning
  syslog->println("Scanning BLE devices...");
  syslog->print("Looking for ");
  syslog->println(liveData->settings.obdMacAddress);
  BLEScanResults foundDevices = liveData->pBLEScan->start(10, false);
  syslog->print("Devices found: ");
  syslog->println(foundDevices.getCount());
  syslog->println("Scan done!");
  liveData->pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

  char tmpStr1[20];
  sprintf(tmpStr1, "Found %d devices", foundDevices.getCount());
  board->displayMessage(" > Scanning BLE4 devices", tmpStr1);

  // Scan devices from menu, show list of devices
  if (liveData->menuCurrent == 9999)
  {
    syslog->println("Display menu with devices");
    liveData->menuVisible = true;
    // liveData->menuCurrent = 9999;
    liveData->menuItemSelected = 0;
    board->showMenu();
  }
  else
  {
    // Redraw screen
    if (liveData->foundMyBleDevice == NULL)
    {
      board->displayMessage("Device not found", "Middle button - menu");
    }
    else
    {
      board->redrawScreen();
    }
  }
}

namespace
{
  bool isKnownObdService(const String &sUuid, const String &configuredUuid)
  {
    if (configuredUuid.length() > 0 && sUuid.indexOf(configuredUuid) != -1)
      return true;
    if (sUuid.indexOf("fff0") != -1 ||      // OBDLink CX, Veepeak, Viecar, generic ELM327
        sUuid.indexOf("18f0") != -1 ||      // Vgate iCar Pro, vLinker MC/FD
        sUuid.indexOf("ffe0") != -1 ||      // LELink, HM-10 / CC2541 / Carista / Viecar
        sUuid.indexOf("ffe5") != -1 ||      // Generic ELM327 BLE clones
        sUuid.indexOf("a001") != -1 ||      // Generic OBD BLE dongles
        sUuid.indexOf("6e400001") != -1 ||  // Nordic UART Service (nRF NUS)
        sUuid.indexOf("e7810a71") != -1)    // OBDLink proprietary
      return true;
    return false;
  }

  bool isNonObdOrOtaService(const String &sUuid)
  {
    return (sUuid.indexOf("1800") != -1 ||      // Generic Access
            sUuid.indexOf("1801") != -1 ||      // Generic Attribute
            sUuid.indexOf("180a") != -1 ||      // Device Information
            sUuid.indexOf("1804") != -1 ||      // Tx Power
            sUuid.indexOf("180f") != -1 ||      // Battery Service
            sUuid.indexOf("fef5") != -1 ||      // Dialog SUOTA OTA
            sUuid.indexOf("fe59") != -1 ||      // Nordic DFU OTA
            sUuid.indexOf("8e400001") != -1 ||  // Nordic DFU OTA (128-bit)
            sUuid.indexOf("f000ffc0") != -1);   // TI OAD OTA
  }
} // namespace

/**
   Connect to BLE device and automatically detect service and characteristics
*/
bool CommObd2Ble4::connectToServer(BLEAddress pAddress)
{
  String devName = strlen(liveData->settings.obd2Name) > 0 ? liveData->settings.obd2Name : "OBD BLE Adapter";
  String devMac = String("(") + pAddress.toString().c_str() + ")";
  connectStatus = "Connecting...";
  board->displayMessage(" > Connecting device", devName.c_str(), devMac.c_str());

  syslog->print("Connecting to device: ");
  syslog->println(pAddress.toString().c_str());

  // Set BLE encryption and security
#ifndef EVDASH_USE_NIMBLE
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
#endif
  BLEDevice::setSecurityCallbacks(new MySecurity());

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
  pSecurity->setCapability(ESP_IO_CAP_KBDISP);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  // Create BLE client and set callbacks
  if (liveData->pClient == nullptr)
  {
    liveData->pClient = BLEDevice::createClient();
    if (liveData->pClient == nullptr)
    {
      syslog->println("Failed to create BLE client.");
      return false;
    }
    liveData->pClient->setClientCallbacks(new MyClientCallback());
  }
  else if (liveData->pClient->isConnected())
  {
    liveData->pClient->disconnect();
  }

#ifdef EVDASH_USE_NIMBLE
  liveData->pClient->setConnectTimeout(3); // 3 seconds timeout
#endif

  // Poll hardware buttons/touch input before initiating connection
  board->boardLoop();
  if (liveData->params.stopCommandQueue || !liveData->obd2ready)
  {
    syslog->println("Connection attempt cancelled by user input.");
    return false;
  }

  // Attempt to connect to the BLE device (async = false, non-blocking call in NimBLE)
#ifdef EVDASH_USE_NIMBLE
  bool connected = liveData->pClient->connect(BLEAddress(pAddress.toString(), BLE_ADDR_RANDOM), false);
  const uint32_t connStartMs = millis();
  while (!connected && !liveData->pClient->isConnected() && (millis() - connStartMs < 3000))
  {
    board->boardLoop();
    if (liveData->params.stopCommandQueue || !liveData->obd2ready)
    {
      syslog->println("Connection attempt cancelled by user input during async connect.");
      liveData->pClient->disconnect();
      return false;
    }
    delay(20);
  }
  connected = liveData->pClient->isConnected();

  if (!connected && liveData->obd2ready && !liveData->params.stopCommandQueue)
  {
    syslog->println("Connect with RANDOM address type failed. Trying PUBLIC...");
    connected = liveData->pClient->connect(BLEAddress(pAddress.toString(), BLE_ADDR_PUBLIC), false);
    const uint32_t connStartMs2 = millis();
    while (!connected && !liveData->pClient->isConnected() && (millis() - connStartMs2 < 3000))
    {
      board->boardLoop();
      if (liveData->params.stopCommandQueue || !liveData->obd2ready)
      {
        syslog->println("Connection attempt cancelled by user input during async connect.");
        liveData->pClient->disconnect();
        return false;
      }
      delay(20);
    }
    connected = liveData->pClient->isConnected();
  }
#else
  bool connected = liveData->pClient->connect(pAddress, BLE_ADDR_TYPE_RANDOM);
  if (!connected && liveData->obd2ready && !liveData->params.stopCommandQueue)
  {
    board->boardLoop();
    if (!liveData->params.stopCommandQueue && liveData->obd2ready)
    {
      syslog->println("Connect with RANDOM address type failed. Trying PUBLIC...");
      connected = liveData->pClient->connect(pAddress, BLE_ADDR_TYPE_PUBLIC);
    }
  }
#endif

  if (liveData->params.stopCommandQueue || !liveData->obd2ready)
  {
    syslog->println("Connection cancelled during BLE connect.");
    if (connected && liveData->pClient && liveData->pClient->isConnected())
    {
      liveData->pClient->disconnect();
    }
    return false;
  }

  if (!connected)
  {
    syslog->println("Failed to connect to BLE device.");
    return false;
  }
  syslog->println("Successfully connected to BLE device.");

  // Discover all services. NimBLE returns a vector, Bluedroid a map.
  liveData->pRemoteCharacteristic = nullptr;
  liveData->pRemoteCharacteristicWrite = nullptr;

#ifdef EVDASH_USE_NIMBLE
  std::vector<BLERemoteService *> *services = liveData->pClient->getServices(true);
  if (services == nullptr || services->empty())
  {
    syslog->println("No services found.");
    liveData->pClient->disconnect();
    return false;
  }

  String configuredUuid = String(liveData->settings.serviceUUID);
  configuredUuid.toLowerCase();
  configuredUuid.trim();

  // Pass 1: Prioritize known OBD-II Serial GATT Services & configured serviceUUID
  for (auto *pRemoteService : *services)
  {
    String sUuid = pRemoteService->getUUID().toString().c_str();
    sUuid.toLowerCase();
    syslog->print("Detected service UUID: ");
    syslog->println(sUuid.c_str());

    if (!isKnownObdService(sUuid, configuredUuid))
      continue;

    syslog->println("Matched known OBD serial service.");
    std::vector<BLERemoteCharacteristic *> *characteristics = pRemoteService->getCharacteristics(true);
    for (auto *pCharacteristic : *characteristics)
    {
      if ((pCharacteristic->canNotify() || pCharacteristic->canIndicate()) && liveData->pRemoteCharacteristic == nullptr)
      {
        liveData->pRemoteCharacteristic = pCharacteristic;
        syslog->print("Detected Tx characteristic UUID: ");
        syslog->println(pCharacteristic->getUUID().toString().c_str());
      }
      if ((pCharacteristic->canWrite() || pCharacteristic->canWriteNoResponse()) && liveData->pRemoteCharacteristicWrite == nullptr)
      {
        liveData->pRemoteCharacteristicWrite = pCharacteristic;
        syslog->print("Detected Rx characteristic UUID: ");
        syslog->println(pCharacteristic->getUUID().toString().c_str());
      }
    }
    if (liveData->pRemoteCharacteristic && liveData->pRemoteCharacteristicWrite)
      break;
  }

  // Pass 2: Fallback scan for unknown adapters (skipping non-OBD/OTA services)
  if (liveData->pRemoteCharacteristic == nullptr || liveData->pRemoteCharacteristicWrite == nullptr)
  {
    for (auto *pRemoteService : *services)
    {
      String sUuid = pRemoteService->getUUID().toString().c_str();
      sUuid.toLowerCase();

      if (isNonObdOrOtaService(sUuid))
        continue;

      std::vector<BLERemoteCharacteristic *> *characteristics = pRemoteService->getCharacteristics(true);
      for (auto *pCharacteristic : *characteristics)
      {
        if ((pCharacteristic->canNotify() || pCharacteristic->canIndicate()) && liveData->pRemoteCharacteristic == nullptr)
        {
          liveData->pRemoteCharacteristic = pCharacteristic;
          syslog->print("Detected Tx characteristic UUID (fallback): ");
          syslog->println(pCharacteristic->getUUID().toString().c_str());
        }
        if ((pCharacteristic->canWrite() || pCharacteristic->canWriteNoResponse()) && liveData->pRemoteCharacteristicWrite == nullptr)
        {
          liveData->pRemoteCharacteristicWrite = pCharacteristic;
          syslog->print("Detected Rx characteristic UUID (fallback): ");
          syslog->println(pCharacteristic->getUUID().toString().c_str());
        }
      }
      if (liveData->pRemoteCharacteristic && liveData->pRemoteCharacteristicWrite)
        break;
    }
  }
#else
  std::map<std::string, BLERemoteService *> *services = liveData->pClient->getServices();
  if (services == nullptr || services->empty())
  {
    syslog->println("No services found.");
    liveData->pClient->disconnect();
    return false;
  }

  String configuredUuid = String(liveData->settings.serviceUUID);
  configuredUuid.toLowerCase();
  configuredUuid.trim();

  // Pass 1: Prioritize known OBD-II Serial GATT Services & configured serviceUUID
  for (auto const &entry : *services)
  {
    BLERemoteService *pRemoteService = entry.second;
    String sUuid = String(entry.first.c_str());
    sUuid.toLowerCase();
    syslog->print("Detected service UUID: ");
    syslog->println(sUuid.c_str());

    if (!isKnownObdService(sUuid, configuredUuid))
      continue;

    syslog->println("Matched known OBD serial service.");
    std::map<std::string, BLERemoteCharacteristic *> *characteristics = pRemoteService->getCharacteristics();
    for (auto const &charEntry : *characteristics)
    {
      BLERemoteCharacteristic *pCharacteristic = charEntry.second;
      if ((pCharacteristic->canNotify() || pCharacteristic->canIndicate()) && liveData->pRemoteCharacteristic == nullptr)
      {
        liveData->pRemoteCharacteristic = pCharacteristic;
        syslog->print("Detected Tx characteristic UUID: ");
        syslog->println(charEntry.first.c_str());
      }
      if ((pCharacteristic->canWrite() || pCharacteristic->canWriteNoResponse()) && liveData->pRemoteCharacteristicWrite == nullptr)
      {
        liveData->pRemoteCharacteristicWrite = pCharacteristic;
        syslog->print("Detected Rx characteristic UUID: ");
        syslog->println(charEntry.first.c_str());
      }
    }
    if (liveData->pRemoteCharacteristic && liveData->pRemoteCharacteristicWrite)
      break;
  }

  // Pass 2: Fallback scan for unknown adapters (skipping non-OBD/OTA services)
  if (liveData->pRemoteCharacteristic == nullptr || liveData->pRemoteCharacteristicWrite == nullptr)
  {
    for (auto const &entry : *services)
    {
      BLERemoteService *pRemoteService = entry.second;
      String sUuid = String(entry.first.c_str());
      sUuid.toLowerCase();

      if (isNonObdOrOtaService(sUuid))
        continue;

      std::map<std::string, BLERemoteCharacteristic *> *characteristics = pRemoteService->getCharacteristics();
      for (auto const &charEntry : *characteristics)
      {
        BLERemoteCharacteristic *pCharacteristic = charEntry.second;
        if ((pCharacteristic->canNotify() || pCharacteristic->canIndicate()) && liveData->pRemoteCharacteristic == nullptr)
        {
          liveData->pRemoteCharacteristic = pCharacteristic;
          syslog->print("Detected Tx characteristic UUID (fallback): ");
          syslog->println(charEntry.first.c_str());
        }
        if ((pCharacteristic->canWrite() || pCharacteristic->canWriteNoResponse()) && liveData->pRemoteCharacteristicWrite == nullptr)
        {
          liveData->pRemoteCharacteristicWrite = pCharacteristic;
          syslog->print("Detected Rx characteristic UUID (fallback): ");
          syslog->println(charEntry.first.c_str());
        }
      }
      if (liveData->pRemoteCharacteristic && liveData->pRemoteCharacteristicWrite)
        break;
    }
  }
#endif

  // Check if Tx and Rx characteristics were found
  if (liveData->pRemoteCharacteristic == nullptr || liveData->pRemoteCharacteristicWrite == nullptr)
  {
    syslog->println("Failed to detect Tx/Rx characteristics.");
    liveData->pClient->disconnect();
    return false;
  }

  syslog->println("Successfully detected service and characteristics.");

  // Enable indications on the Tx characteristic (CCCD = {0x02,0x00}).
#ifdef EVDASH_USE_NIMBLE
  if (liveData->pRemoteCharacteristic->canNotify())
  {
    liveData->pRemoteCharacteristic->subscribe(true, notifyCallback, true);
    delay(200);
  }
  else if (liveData->pRemoteCharacteristic->canIndicate())
  {
    liveData->pRemoteCharacteristic->subscribe(false, notifyCallback, true);
    delay(200);
  }
#else
  if (liveData->pRemoteCharacteristic->canNotify())
  {
    const uint8_t indicationOn[] = {0x2, 0x0};
    BLERemoteDescriptor *notifyDescriptor = liveData->pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902));
    if (notifyDescriptor != nullptr)
    {
      notifyDescriptor->writeValue((uint8_t *)indicationOn, 2, true);
    }
    else
    {
      syslog->println("Notify descriptor 0x2902 not found. Registering callback only.");
    }
    liveData->pRemoteCharacteristic->registerForNotify(notifyCallback, false);
    delay(200);
  }
#endif

  syslog->println("BLE device is ready for communication.");
  return true;
}

/**
   Main loop
*/
void CommObd2Ble4::mainLoop()
{
  if (liveData->params.stopCommandQueue || suspendedDevice)
  {
    return;
  }

  // Render deferred BLE connect/disconnect message here (loop task), not in the callback.
  const int8_t connEvent = bleConnEventPending;
  if (connEvent != 0)
  {
    bleConnEventPending = 0;
    board->displayMessage(connEvent == 1 ? "BLE connected" : "BLE disconnected", "");
  }

  // Prompt watchdog: if a command was sent but we didn't get the '>' prompt back,
  // do NOT advance to new commands. Retry sending carriage return to prompt the adapter.
  // If the adapter remains unresponsive after retries, disconnect and reconnect cleanly.
  if (liveData->commConnected && !liveData->canSendNextAtCommand &&
      lastBleCmdSentMs != 0 && (uint32_t)(millis() - lastBleCmdSentMs) > kBleCmdTimeoutMs)
  {
    if (bleStallRetryCount < kBleMaxStallRetries)
    {
      bleStallRetryCount++;
      syslog->print("BLE prompt missing for [");
      syslog->print(liveData->commandRequest);
      syslog->print("]. Sending prompt retry (");
      syslog->print(bleStallRetryCount);
      syslog->print("/");
      syslog->print(kBleMaxStallRetries);
      syslog->println(")...");
      // Do NOT set canSendNextAtCommand - do not advance to new commands without prompt!
      executeCommand("");
      lastBleCmdSentMs = millis();
    }
    else
    {
      syslog->println("BLE adapter unresponsive: no '>' prompt after retries. Disconnecting to recover.");
      bleStallRetryCount = 0;
      lastBleCmdSentMs = 0;
      liveData->responseRow = "";
      liveData->responseRowMerged = "";
      liveData->canSendNextAtCommand = false;
      liveData->commandQueueIndex = 0;
      if (liveData->pClient != nullptr && liveData->pClient->isConnected())
      {
        liveData->pClient->disconnect();
      }
      liveData->commConnected = false;
      liveData->obd2ready = true;
    }
  }

  // Connect BLE device
  if (!liveData->commConnected && liveData->obd2ready == true && hasConfiguredBleMac(liveData->settings.obdMacAddress))
  {
    if (liveData->menuVisible)
    {
      // Avoid blocking connect attempts while user is interacting with menu.
      connectStatus = "BLE connect paused (menu)";
    }
    else
    {
      const uint32_t nowMs = millis();
      const bool retryWindowOpen =
          (nextConnectRetryMs == 0) || (static_cast<int32_t>(nowMs - nextConnectRetryMs) >= 0);
      if (retryWindowOpen)
      {
        BLEAddress serverAddress(liveData->settings.obdMacAddress);
        if (connectToServer(serverAddress))
        {

          liveData->commConnected = true;
          connectFailCount = 0;
          nextConnectRetryMs = 0;
          bleStallRetryCount = 0;

          syslog->println("We are now connected to the BLE device.");
          connectStatus = "Connected";

          // Clear popup dialog so dashboard is accessible and modal window disappears
          board->dismissMessageDialog();

          // Serve first command (ATZ)
          doNextQueueCommand();
        }
        else
        {
          syslog->println("We have failed to connect to the server; scheduling retry.");
          if (!suspendedDevice && !liveData->params.stopCommandQueue)
          {
            if (connectFailCount < 0xFF)
              connectFailCount++;
            const uint32_t retryDelayMs = calcConnectRetryDelayMs(connectFailCount);
            nextConnectRetryMs = nowMs + retryDelayMs;
            connectStatus = String("Retry in ") + String(retryDelayMs / 1000) + " s";
          }
          else
          {
            connectFailCount = 0;
            nextConnectRetryMs = 0;
            connectStatus = "Cancelled";
            board->dismissMessageDialog();
          }
        }
      }
    }
  }

  // Parent declaration
  CommInterface::mainLoop();

  if (board->scanDevices)
  {
    board->scanDevices = false;
    startBleScan();
  }
}

/**
 * Send command
 */
void CommObd2Ble4::executeCommand(String cmd)
{

  String tmpStr = cmd + "\r";
  // Require the write handle too: commConnected can still be true for a moment after
  // a disconnect callback nulls the characteristic pointers.
  if (liveData->commConnected && liveData->pRemoteCharacteristicWrite != nullptr)
  {
    // Drop any unfinished line fragment from the previous response so it cannot
    // prepend itself to this command's response (responseRow persists across
    // BLE notifications, see notifyCallback).
    liveData->responseRow = "";
    liveData->pRemoteCharacteristicWrite->writeValue(tmpStr.c_str(), tmpStr.length());
    lastBleCmdSentMs = millis(); // arm the queue-stall watchdog
  }
}

/**
 * Suspends the CAN device by setting it to sleep mode.
 * Stops communication and minimizes power consumption.
 */
void CommObd2Ble4::suspendDevice()
{
  suspendedDevice = true;
  liveData->commConnected = false;
  liveData->obd2ready = false;
  connectFailCount = 0;
  nextConnectRetryMs = 0;
  if (liveData->pClient != nullptr && liveData->pClient->isConnected())
  {
    liveData->pClient->disconnect();
  }
  if (liveData->pBLEScan != nullptr)
  {
    liveData->pBLEScan->stop();
  }
  connectStatus = "Suspended";
}

/**
 * Resumes the CAN device by reinitializing it to normal mode.
 */
void CommObd2Ble4::resumeDevice()
{
  suspendedDevice = false;
  liveData->obd2ready = true;
  connectFailCount = 0;
  nextConnectRetryMs = 0;
  connectStatus = "Resumed";
}
