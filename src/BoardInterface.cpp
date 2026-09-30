/*
BoardInterface.cpp provides an interface for configuring and interacting with the hardware components of a device.

It allows setting a LiveData object that holds live sensor data. It allows attaching a CarInterface object that interfaces with the car's OBD port.

It provides methods for:

ShutdownDevice() - Shuts down the device by disconnecting communications, turning off peripherals, and putting the microcontroller into deep sleep mode.
It displays a countdown message, reduces the CPU speed, turns off the screen, disables wifi/BT, disconnects the comm interface, and finally puts the ESP32 into deep sleep.

SaveSettings() - Saves settings from the LiveData object into EEPROM flash memory. This persists settings across reboots.

ResetSettings() - Resets settings to factory defaults, erases EEPROM, and restarts the device.

LoadSettings() - Loads settings from EEPROM into the LiveData object on startup. It first initializes the settings struct with default values.
Then it loads the actual values from EEPROM if they exist. It handles upgrading old format settings.

attachCar() - Attaches a CarInterface object to allow communicating with the car's OBD port.

setLiveData() - Sets the LiveData object that holds the live sensor data that will be displayed and logged.

So in summary, this provides a hardware abstraction layer for interacting with the device's peripherals and configuration in a simple way.
It handles attaching communications and live data objects. And provides methods for lifecycle events like shutdown, settings load/save, factory reset, etc.
*/

#define ARDUINOJSON_USE_LONG_LONG 1

#include <WiFi.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "ble_compat.h"
#include "BoardInterface.h"
#include "CommObd2Ble4.h"
#include "CommObd2Can.h"
#include "LiveData.h"
#include "Solarlib.h"

/**
 * Set live data
 */
void BoardInterface::setLiveData(LiveData *pLiveData)
{
  liveData = pLiveData;
}

/**
 * Attach car interface
 */
void BoardInterface::attachCar(CarInterface *pCarInterface)
{
  carInterface = pCarInterface;
}

/**
 * Shutdown device
 */
void BoardInterface::shutdownDevice()
{
  syslog->println("Shutdown.");

  char msg[20];
  for (int i = 3; i >= 1; i--)
  {
    snprintf(msg, sizeof(msg), "Shutdown in %d sec.", i);
    displayMessage(msg, "");
    delay(1000);
  }

  setCpuFrequencyMhz(80);
  turnOffScreen();
  // WiFi.disconnect(true);
  // WiFi.mode(WIFI_OFF);

  commInterface->disconnectDevice();
  // adc_power_off();
  // esp_wifi_stop();
  esp_bt_controller_disable();

  delay(2000);
  // esp_sleep_enable_timer_wakeup(525600L * 60L * 1000000L); // minutes
  esp_deep_sleep_start();
}

/**
 * Save setting to flash memory
 */
void BoardInterface::saveSettings()
{
  syslog->println("Settings saved to eeprom.");
  EEPROM.put(0, liveData->settings);
  EEPROM.commit();
}

/**
 * Reset settings (factory reset)
 */
void BoardInterface::resetSettings()
{
  syslog->println("Factory reset.");
  liveData->settings.initFlag = 1;
  EEPROM.put(0, liveData->settings);
  EEPROM.commit();

  displayMessage("Settings erased", "Restarting in 5 secs.");

  delay(5000);
  ESP.restart();
}

/**
 * Generate random alphanumeric string
 */
void BoardInterface::generateRandomAlphanumeric(char *buf, size_t count)
{
  static const char charset[] = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  const size_t charsetSize = sizeof(charset) - 1;
  for (size_t i = 0; i < count; i++)
  {
    buf[i] = charset[esp_random() % charsetSize];
  }
  buf[count] = '\0';
}

/**
 * Validate that password contains only printable alphanumeric characters of minimum length
 */
bool BoardInterface::isValidPassword(const char *pwd, size_t minLen)
{
  if (pwd == nullptr)
  {
    return false;
  }
  size_t len = strlen(pwd);
  if (len < minLen)
  {
    return false;
  }
  for (size_t i = 0; i < len; i++)
  {
    if (!isalnum((unsigned char)pwd[i]))
    {
      return false;
    }
  }
  return true;
}

/**
 * Load setting from flash memory, upgrade structure if version differs
 */
void BoardInterface::loadSettings()
{
  String tmpStr;

  // Default settings
  liveData->settings.initFlag = 183;
  liveData->settings.settingsVersion = SETTINGS_VERSION_CURRENT;
  liveData->settings.carType = CAR_KIA_ENIRO_2020_64;
  tmpStr = "00:00:00:00:00:00"; // Pair via menu (middle button)
  tmpStr.toCharArray(liveData->settings.obdMacAddress, tmpStr.length() + 1);
  tmpStr = "000018f0-0000-1000-8000-00805f9b34fb"; // Default UUID's for VGate iCar Pro BLE4 adapter
  tmpStr.toCharArray(liveData->settings.serviceUUID, tmpStr.length() + 1);
  tmpStr = "00002af0-0000-1000-8000-00805f9b34fb";
  tmpStr.toCharArray(liveData->settings.charTxUUID, tmpStr.length() + 1);
  tmpStr = "00002af1-0000-1000-8000-00805f9b34fb";
  tmpStr.toCharArray(liveData->settings.charRxUUID, tmpStr.length() + 1);
  liveData->settings.displayRotation = 1; // 1,3
  liveData->settings.distanceUnit = 'k';
  liveData->settings.temperatureUnit = 'c';
  liveData->settings.pressureUnit = 'b';
  liveData->settings.defaultScreen = 1;
  liveData->settings.lcdBrightness = 0;
  liveData->settings.predrawnChargingGraphs = 1;
  // liveData->settings.commType = COMM_TYPE_OBD2_BLE4; // BLE4
  liveData->settings.commType = COMM_TYPE_CAN_COMMU; // CAN
  liveData->settings.wifiEnabled = 0;
  tmpStr = "empty";
  tmpStr.toCharArray(liveData->settings.wifiSsid, tmpStr.length() + 1);
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.wifiPassword, tmpStr.length() + 1);
  liveData->settings.ntpEnabled = 1;
  liveData->settings.ntpTimezone = 1;
  liveData->settings.ntpDaySaveTime = 0;
  liveData->settings.sdcardEnabled = 0;
  liveData->settings.sdcardAutstartLog = 1;
  liveData->settings.sdcardConsoleLogEnabled = 0;
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.gprsApn, tmpStr.length() + 1);
  // Remote upload
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.remoteApiUrl, tmpStr.length() + 1);
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.remoteApiKey, tmpStr.length() + 1);
  liveData->settings.headlightsReminder = 0;
  liveData->settings.gpsHwSerialPort = 255;  // off
  liveData->settings.gprsHwSerialPort = 255; // off
  liveData->settings.serialConsolePort = 0;  // hwuart0
  liveData->settings.debugLevel = 1;         // 0 - info only, 1 - debug communication (BLE/CAN), 2 - debug GSM, 3 - debug SDcard
  liveData->settings.sdcardLogIntervalSec = 2;
  liveData->settings.gprsLogIntervalSec = 60;
  liveData->settings.sleepModeLevel = SLEEP_MODE_OFF;
  liveData->settings.voltmeterEnabled = 0;
  liveData->settings.voltmeterBasedSleep = 0;
  liveData->settings.voltmeterCutOff = 12.0;
  liveData->settings.voltmeterSleep = 12.8;
  liveData->settings.voltmeterWakeUp = 13.0;
  liveData->settings.remoteUploadIntervalSec = 60;
  liveData->settings.sleepModeIntervalSec = 30;
  liveData->settings.sleepModeShutdownHrs = 72;
  liveData->settings.remoteUploadModuleType = REMOTE_UPLOAD_WIFI;
  liveData->settings.remoteUploadAbrpIntervalSec = 0;
  tmpStr = "empty";
  tmpStr.toCharArray(liveData->settings.abrpApiToken, tmpStr.length() + 1);
  // v11
  liveData->settings.timezone = 0;
  liveData->settings.daylightSaving = 0;
  liveData->settings.rightHandDrive = 0;
  // v12
  tmpStr = "empty";
  tmpStr.toCharArray(liveData->settings.wifiSsid2, tmpStr.length() + 1);
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.wifiPassword2, tmpStr.length() + 1);
  liveData->settings.backupWifiEnabled = 0;
  // v13
  liveData->settings.threading = 0;
  liveData->settings.speedCorrection = 0;
  // v14
  liveData->settings.disableCommandOptimizer = 0;
  // v15
  liveData->settings.abrpSdcardLog = 0;
  // v16
  tmpStr = "OBD2"; // default BLE4 OBD2 adapter name
  tmpStr.toCharArray(liveData->settings.obd2Name, tmpStr.length() + 1);
  tmpStr = "192.168.0.10"; // legacy obd2wifi adapter ip
  tmpStr.toCharArray(liveData->settings.obd2WifiIp, tmpStr.length() + 1);
  liveData->settings.obd2WifiPort = 35000;
  // v17
  liveData->settings.settingsVersion = 17;
  liveData->settings.contributeData = 1;
  tmpStr = "\n";
  tmpStr.toCharArray(liveData->settings.contributeToken, tmpStr.length() + 1);
  liveData->settings.mqttEnabled = 0;
  tmpStr = "192.168.0.1";
  tmpStr.toCharArray(liveData->settings.mqttServer, tmpStr.length() + 1);
  tmpStr = "evdash";
  tmpStr.toCharArray(liveData->settings.mqttId, tmpStr.length() + 1);
  tmpStr = "evuser";
  tmpStr.toCharArray(liveData->settings.mqttUsername, tmpStr.length() + 1);
  tmpStr = "evpass";
  tmpStr.toCharArray(liveData->settings.mqttPassword, tmpStr.length() + 1);
  tmpStr = "evdash/sensors";
  tmpStr.toCharArray(liveData->settings.mqttPubTopic, tmpStr.length() + 1);
  // v18
  liveData->settings.settingsVersion = 18;
  liveData->settings.commandQueueAutoStop = 1;
  liveData->settings.gpsSerialPortSpeed = 9600;
  // v19
  liveData->settings.settingsVersion = 19;
  liveData->settings.boardPowerMode = 1;
  // v20
  liveData->settings.settingsVersion = 20;
  liveData->settings.gpsModuleType = GPS_MODULE_TYPE_NEO_M8N;
  // v21
  liveData->settings.settingsVersion = 21;
  liveData->settings.carSpeedType = CAR_SPEED_TYPE_AUTO;
  // v22
  liveData->settings.settingsVersion = 22;
  liveData->settings.contributeJsonType = CONTRIBUTE_JSON_TYPE_V2;
  // v23
  liveData->settings.settingsVersion = 23;
  liveData->settings.traccarEnabled = 0;
  // v24
  liveData->settings.settingsVersion = 24;
  tmpStr = "demo3.traccar.org";
  tmpStr.toCharArray(liveData->settings.traccarServerHost, tmpStr.length() + 1);
  liveData->settings.traccarServerPort = 5055;
  // v25
  liveData->settings.relayForMobileEnabled = 0;
  liveData->settings.relayToken[0] = '\0';
  liveData->settings.relayMobileId[0] = '\0';
  // v26
  tmpStr = "empty";
  tmpStr.toCharArray(liveData->settings.wifiSsid3, tmpStr.length() + 1);
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.wifiPassword3, tmpStr.length() + 1);
  tmpStr = "empty";
  tmpStr.toCharArray(liveData->settings.wifiSsid4, tmpStr.length() + 1);
  tmpStr = "not_set";
  tmpStr.toCharArray(liveData->settings.wifiPassword4, tmpStr.length() + 1);
  // v27
  liveData->settings.sdcardConsoleLogEnabled = 0;
  // v28
  generateRandomAlphanumeric(liveData->settings.webLogServerPassword, 8);
  // v29
  liveData->settings.bleAddressType = BLE_ADDRESS_TYPE_RANDOM;
  // v30
  liveData->settings.mqttUseTls = 0;
  liveData->settings.mqttPort = 0;
  // v31
  liveData->settings.mqttHomeAssistant = 0;
  // v32
  liveData->settings.settingsVersion = SETTINGS_VERSION_CURRENT;
  liveData->settings.haName[0] = '\0';
  liveData->settings.haModel[0] = '\0';

  // Load settings and replace default values
  syslog->println("Reading settings from eeprom.");
  EEPROM.begin(sizeof(SETTINGS_STRUC));
  EEPROM.get(0, liveData->tmpSettings);

  // Init flash with default settings
  if (liveData->tmpSettings.initFlag != 183)
  {
    syslog->println("Settings not found. Initialization.");
    saveSettings();
  }
  else
  {
    syslog->print("Loaded settings ver.: ");
    syslog->println(liveData->tmpSettings.settingsVersion);

    // Upgrade structure
    if (liveData->settings.settingsVersion != liveData->tmpSettings.settingsVersion)
    {
      if (liveData->tmpSettings.settingsVersion == 1)
      {
        liveData->tmpSettings.settingsVersion = 2;
        liveData->tmpSettings.defaultScreen = liveData->settings.defaultScreen;
        liveData->tmpSettings.lcdBrightness = liveData->settings.lcdBrightness;
      }
      if (liveData->tmpSettings.settingsVersion == 2)
      {
        liveData->tmpSettings.settingsVersion = 3;
        liveData->tmpSettings.predrawnChargingGraphs = liveData->settings.predrawnChargingGraphs;
      }
      if (liveData->tmpSettings.settingsVersion == 3)
      {
        liveData->tmpSettings.settingsVersion = 4;
        liveData->tmpSettings.commType = COMM_TYPE_OBD2_BLE4; // BLE4
        liveData->tmpSettings.wifiEnabled = 0;
        tmpStr = "empty";
        tmpStr.toCharArray(liveData->tmpSettings.wifiSsid, tmpStr.length() + 1);
        tmpStr = "not_set";
        tmpStr.toCharArray(liveData->tmpSettings.wifiPassword, tmpStr.length() + 1);
        liveData->tmpSettings.ntpEnabled = 1;
        liveData->tmpSettings.ntpTimezone = 1;
        liveData->tmpSettings.ntpDaySaveTime = 0;
        liveData->tmpSettings.sdcardEnabled = 0;
        liveData->tmpSettings.sdcardAutstartLog = 1;
        tmpStr = "internet.t-mobile.cz";
        tmpStr.toCharArray(liveData->tmpSettings.gprsApn, tmpStr.length() + 1);
        // Remote upload
        tmpStr = "http://api.example.com";
        tmpStr.toCharArray(liveData->tmpSettings.remoteApiUrl, tmpStr.length() + 1);
        tmpStr = "example";
        tmpStr.toCharArray(liveData->tmpSettings.remoteApiKey, tmpStr.length() + 1);
        liveData->tmpSettings.headlightsReminder = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 4)
      {
        liveData->tmpSettings.settingsVersion = 5;
        liveData->tmpSettings.gpsHwSerialPort = 255; // off
      }
      if (liveData->tmpSettings.settingsVersion == 5)
      {
        liveData->tmpSettings.settingsVersion = 6;
        liveData->tmpSettings.serialConsolePort = 0; // hwuart0
        liveData->tmpSettings.debugLevel = 0;        // show all
        liveData->tmpSettings.sdcardLogIntervalSec = 2;
        liveData->tmpSettings.gprsLogIntervalSec = 60;
      }
      if (liveData->tmpSettings.settingsVersion == 6)
      {
        liveData->tmpSettings.settingsVersion = 7;
        liveData->tmpSettings.sleepModeLevel = SLEEP_MODE_OFF;
      }
      if (liveData->tmpSettings.settingsVersion == 7)
      {
        liveData->tmpSettings.settingsVersion = 8;
        liveData->tmpSettings.voltmeterEnabled = 0;
        liveData->tmpSettings.voltmeterBasedSleep = 0;
        liveData->tmpSettings.voltmeterCutOff = 12.0;
        liveData->tmpSettings.voltmeterSleep = 12.8;
        liveData->tmpSettings.voltmeterWakeUp = 13.0;
      }
      if (liveData->tmpSettings.settingsVersion == 8)
      {
        liveData->tmpSettings.settingsVersion = 9;
        liveData->tmpSettings.remoteUploadIntervalSec = 60;
        liveData->tmpSettings.sleepModeIntervalSec = 30;
        liveData->tmpSettings.sleepModeShutdownHrs = 72;
        liveData->tmpSettings.remoteUploadModuleType = REMOTE_UPLOAD_WIFI;
      }
      if (liveData->tmpSettings.settingsVersion == 9)
      {
        liveData->tmpSettings.settingsVersion = 10;
        liveData->tmpSettings.remoteUploadAbrpIntervalSec = 0;
        tmpStr = "empty";
        tmpStr.toCharArray(liveData->tmpSettings.abrpApiToken, tmpStr.length() + 1);
      }
      if (liveData->tmpSettings.settingsVersion == 10)
      {
        liveData->tmpSettings.settingsVersion = 11;
        liveData->tmpSettings.timezone = 0;
        liveData->tmpSettings.daylightSaving = 0;
        liveData->tmpSettings.rightHandDrive = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 11)
      {
        liveData->tmpSettings.settingsVersion = 12;
        tmpStr = "empty";
        tmpStr.toCharArray(liveData->tmpSettings.wifiSsid2, tmpStr.length() + 1);
        tmpStr = "not_set";
        tmpStr.toCharArray(liveData->tmpSettings.wifiPassword2, tmpStr.length() + 1);
        liveData->tmpSettings.backupWifiEnabled = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 12)
      {
        liveData->tmpSettings.settingsVersion = 13;
        liveData->tmpSettings.threading = 0;
        liveData->tmpSettings.speedCorrection = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 13)
      {
        liveData->tmpSettings.settingsVersion = 14;
        liveData->tmpSettings.disableCommandOptimizer = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 14)
      {
        liveData->tmpSettings.settingsVersion = 15;
        liveData->tmpSettings.abrpSdcardLog = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 15)
      {
        liveData->tmpSettings.settingsVersion = 16;
        tmpStr = "OBD2"; // default BLE4 OBD2 adapter name
        tmpStr.toCharArray(liveData->tmpSettings.obd2Name, tmpStr.length() + 1);
        tmpStr = "192.168.0.10"; // legacy obd2wifi adapter ip
        tmpStr.toCharArray(liveData->tmpSettings.obd2WifiIp, tmpStr.length() + 1);
        liveData->tmpSettings.obd2WifiPort = 35000;
      }
      if (liveData->tmpSettings.settingsVersion == 16)
      {
        liveData->tmpSettings.settingsVersion = 17;
        liveData->tmpSettings.contributeData = 1;
        tmpStr = "\n";
        tmpStr.toCharArray(liveData->tmpSettings.contributeToken, tmpStr.length() + 1);
        liveData->tmpSettings.mqttEnabled = 0;
        tmpStr = "192.168.0.1";
        tmpStr.toCharArray(liveData->tmpSettings.mqttServer, tmpStr.length() + 1);
        tmpStr = "evdash";
        tmpStr.toCharArray(liveData->tmpSettings.mqttId, tmpStr.length() + 1);
        tmpStr = "evuser";
        tmpStr.toCharArray(liveData->tmpSettings.mqttUsername, tmpStr.length() + 1);
        tmpStr = "evpass";
        tmpStr.toCharArray(liveData->tmpSettings.mqttPassword, tmpStr.length() + 1);
        tmpStr = "evdash/sensors";
        tmpStr.toCharArray(liveData->tmpSettings.mqttPubTopic, tmpStr.length() + 1);
      }
      if (liveData->tmpSettings.settingsVersion == 17)
      {
        liveData->tmpSettings.settingsVersion = 18;
        liveData->tmpSettings.commandQueueAutoStop = 1;
        liveData->tmpSettings.gpsSerialPortSpeed = 9600;
      }
      if (liveData->tmpSettings.settingsVersion == 18)
      {
        liveData->tmpSettings.settingsVersion = 19;
        liveData->tmpSettings.boardPowerMode = 1;
      }
      if (liveData->tmpSettings.settingsVersion == 19)
      {
        liveData->tmpSettings.settingsVersion = 20;
        liveData->tmpSettings.gpsModuleType = GPS_MODULE_TYPE_NEO_M8N;
      }
      if (liveData->tmpSettings.settingsVersion == 20)
      {
        liveData->tmpSettings.settingsVersion = 21;
        liveData->tmpSettings.carSpeedType = CAR_SPEED_TYPE_AUTO;
      }
      if (liveData->tmpSettings.settingsVersion == 21)
      {
        liveData->tmpSettings.settingsVersion = 22;
        liveData->tmpSettings.contributeJsonType = CONTRIBUTE_JSON_TYPE_V2;
      }
      if (liveData->tmpSettings.settingsVersion == 22)
      {
        liveData->tmpSettings.settingsVersion = 23;
        liveData->tmpSettings.traccarEnabled = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 23)
      {
        liveData->tmpSettings.settingsVersion = 24;
        tmpStr = "demo3.traccar.org";
        tmpStr.toCharArray(liveData->tmpSettings.traccarServerHost, tmpStr.length() + 1);
        liveData->tmpSettings.traccarServerPort = 5055;
      }
      if (liveData->tmpSettings.settingsVersion == 24)
      {
        liveData->tmpSettings.settingsVersion = 25;
        liveData->tmpSettings.relayForMobileEnabled = 0;
        liveData->tmpSettings.relayToken[0] = '\0';
        liveData->tmpSettings.relayMobileId[0] = '\0';
      }
      if (liveData->tmpSettings.settingsVersion == 25)
      {
        liveData->tmpSettings.settingsVersion = 26;
        tmpStr = "empty";
        tmpStr.toCharArray(liveData->tmpSettings.wifiSsid3, tmpStr.length() + 1);
        tmpStr = "not_set";
        tmpStr.toCharArray(liveData->tmpSettings.wifiPassword3, tmpStr.length() + 1);
        tmpStr = "empty";
        tmpStr.toCharArray(liveData->tmpSettings.wifiSsid4, tmpStr.length() + 1);
        tmpStr = "not_set";
        tmpStr.toCharArray(liveData->tmpSettings.wifiPassword4, tmpStr.length() + 1);
      }
      if (liveData->tmpSettings.settingsVersion == 26)
      {
        liveData->tmpSettings.settingsVersion = 27;
        liveData->tmpSettings.sdcardConsoleLogEnabled = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 27)
      {
        liveData->tmpSettings.settingsVersion = 28;
        generateRandomAlphanumeric(liveData->tmpSettings.webLogServerPassword, 8);
      }
      if (liveData->tmpSettings.settingsVersion == 28)
      {
        liveData->tmpSettings.settingsVersion = 29;
        liveData->tmpSettings.bleAddressType = BLE_ADDRESS_TYPE_RANDOM;
      }
      if (liveData->tmpSettings.settingsVersion == 29)
      {
        liveData->tmpSettings.settingsVersion = 30;
        liveData->tmpSettings.mqttUseTls = 0;
        liveData->tmpSettings.mqttPort = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 30)
      {
        liveData->tmpSettings.settingsVersion = 31;
        liveData->tmpSettings.mqttHomeAssistant = 0;
      }
      if (liveData->tmpSettings.settingsVersion == 31)
      {
        liveData->tmpSettings.settingsVersion = SETTINGS_VERSION_CURRENT;
        liveData->tmpSettings.haName[0] = '\0';
        liveData->tmpSettings.haModel[0] = '\0';
      }

      // Save upgraded structure
      liveData->settings = liveData->tmpSettings;
      saveSettings();
    }

    // Apply settings from flash if needed
    liveData->settings = liveData->tmpSettings;
  }

  // Defensive: force NUL termination on char[] settings fields loaded from flash,
  // so a blob corrupted by an older firmware can never be read past the array end
  // (garbage hostname/SSID DNS queries, issue #123).
#define EVDASH_TERMINATE_FIELD(f) liveData->settings.f[sizeof(liveData->settings.f) - 1] = '\0'
  EVDASH_TERMINATE_FIELD(obdMacAddress);
  EVDASH_TERMINATE_FIELD(serviceUUID);
  EVDASH_TERMINATE_FIELD(charTxUUID);
  EVDASH_TERMINATE_FIELD(charRxUUID);
  EVDASH_TERMINATE_FIELD(wifiSsid);
  EVDASH_TERMINATE_FIELD(wifiPassword);
  EVDASH_TERMINATE_FIELD(gprsApn);
  EVDASH_TERMINATE_FIELD(remoteApiUrl);
  EVDASH_TERMINATE_FIELD(remoteApiKey);
  EVDASH_TERMINATE_FIELD(abrpApiToken);
  EVDASH_TERMINATE_FIELD(wifiSsid2);
  EVDASH_TERMINATE_FIELD(wifiPassword2);
  EVDASH_TERMINATE_FIELD(wifiSsid3);
  EVDASH_TERMINATE_FIELD(wifiPassword3);
  EVDASH_TERMINATE_FIELD(wifiSsid4);
  EVDASH_TERMINATE_FIELD(wifiPassword4);
  EVDASH_TERMINATE_FIELD(obd2Name);
  EVDASH_TERMINATE_FIELD(obd2WifiIp);
  EVDASH_TERMINATE_FIELD(contributeToken);
  EVDASH_TERMINATE_FIELD(mqttServer);
  EVDASH_TERMINATE_FIELD(mqttId);
  EVDASH_TERMINATE_FIELD(mqttUsername);
  EVDASH_TERMINATE_FIELD(mqttPassword);
  EVDASH_TERMINATE_FIELD(mqttPubTopic);
  EVDASH_TERMINATE_FIELD(haName);
  EVDASH_TERMINATE_FIELD(haModel);
  EVDASH_TERMINATE_FIELD(traccarServerHost);
  EVDASH_TERMINATE_FIELD(relayToken);
  EVDASH_TERMINATE_FIELD(relayMobileId);
  EVDASH_TERMINATE_FIELD(webLogServerPassword);
#undef EVDASH_TERMINATE_FIELD

  if (!isValidPassword(liveData->settings.webLogServerPassword, 8))
  {
    generateRandomAlphanumeric(liveData->settings.webLogServerPassword, 8);
    saveSettings();
  }

  if (liveData->settings.contributeJsonType != CONTRIBUTE_JSON_TYPE_V2)
  {
    liveData->settings.contributeJsonType = CONTRIBUTE_JSON_TYPE_V2;
    saveSettings();
  }

  if (liveData->settings.remoteUploadModuleType != REMOTE_UPLOAD_WIFI)
  {
    liveData->settings.remoteUploadModuleType = REMOTE_UPLOAD_WIFI;
    saveSettings();
  }

  if (liveData->settings.traccarEnabled > 1)
  {
    liveData->settings.traccarEnabled = 0;
    saveSettings();
  }

  if (strlen(liveData->settings.traccarServerHost) == 0)
  {
    tmpStr = "demo3.traccar.org";
    tmpStr.toCharArray(liveData->settings.traccarServerHost, tmpStr.length() + 1);
    saveSettings();
  }
  if (liveData->settings.traccarServerPort == 0)
  {
    liveData->settings.traccarServerPort = 5055;
    saveSettings();
  }

  if (liveData->settings.commType != COMM_TYPE_OBD2_BLE4 &&
      liveData->settings.commType != COMM_TYPE_CAN_COMMU)
  {
    liveData->settings.commType = COMM_TYPE_OBD2_BLE4;
    saveSettings();
  }

  syslog->setDebugLevel(liveData->settings.debugLevel);
}

/**
 * After setup
 */
void BoardInterface::afterSetup()
{
  syslog->println("BoardInterface::afterSetup");

  // Init COMM iterface
  syslog->print("Init communication device: ");
  syslog->println(liveData->settings.commType);

  if (liveData->settings.commType == COMM_TYPE_CAN_COMMU)
  {
    commInterface = new CommObd2Can();
  }
  else
  {
    commInterface = new CommObd2Ble4();
  }

  // Connect device
  commInterface->initComm(liveData, this);
  commInterface->connectDevice();
  carInterface->setCommInterface(commInterface);
}

/**
 * Process incoming serial console characters continuously.
 */
void BoardInterface::processSerialConsole()
{
  while (syslog != nullptr && syslog->available())
  {
    int ch = syslog->read();
    if (ch == '\r' || ch == '\n')
    {
      consoleBuffer.trim();
      if (consoleBuffer.length() > 0)
      {
        bool handled = customConsoleCommand(consoleBuffer);
        if (!handled && commInterface != nullptr && !commInterface->isSuspended() && !liveData->params.stopCommandQueue)
        {
          commInterface->executeCommand(consoleBuffer);
        }
      }
      consoleBuffer = "";
    }
    else if (ch == '\b' || ch == 127)
    {
      if (consoleBuffer.length() > 0)
      {
        consoleBuffer.remove(consoleBuffer.length() - 1);
      }
    }
    else if (ch >= 32 && ch <= 126)
    {
      consoleBuffer += (char)ch;
    }
  }
}

/**
 * Custom commands
 */
bool BoardInterface::customConsoleCommand(String cmd)
{
  cmd.trim();
  if (cmd.length() == 0)
    return false;

  if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("commands") || cmd.equals("?"))
  {
    showHelp();
    return true;
  }
  if (cmd.equalsIgnoreCase("reboot"))
  {
    ESP.restart();
    return true;
  }
  if (cmd.equalsIgnoreCase("saveSettings"))
  {
    saveSettings();
    return true;
  }
  if (cmd.equalsIgnoreCase("factoryReset"))
  {
    resetSettings();
    return true;
  }
  if (cmd.equalsIgnoreCase("time"))
  {
    showTime();
    return true;
  }
  if (cmd.equalsIgnoreCase("ntpSync"))
  {
    ntpSync();
    return true;
  }
  if (cmd.equalsIgnoreCase("ipconfig"))
  {
    showNet();
    return true;
  }
  if (cmd.equalsIgnoreCase("ABRP_debug"))
  {
    syslog->println(liveData->settings.abrpApiToken);
    return true;
  }
  if (cmd.equalsIgnoreCase("shutdown"))
  {
    enterSleepMode(0);
    return true;
  }
  if (cmd.equalsIgnoreCase("apPassword"))
  {
    syslog->print("AP password: ");
    syslog->println(liveData->settings.webLogServerPassword);
    return true;
  }
  if (cmd.equalsIgnoreCase("bleAddressType") || cmd.equalsIgnoreCase("bleAddrType"))
  {
    syslog->print("BLE MAC address type: ");
    syslog->println((liveData->settings.bleAddressType == BLE_ADDRESS_TYPE_PUBLIC) ? "PUBLIC (fallback RANDOM)" : "RANDOM (fallback PUBLIC)");
    return true;
  }
  if (cmd.equalsIgnoreCase("debugLevel"))
  {
    syslog->printf("Debug level bitmask: %u (comm=%s, net=%s, sd=%s, gps=%s, abrp=%s)\n",
                   liveData->settings.debugLevel,
                   (liveData->settings.debugLevel & DEBUG_COMM) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_NET) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_SDCARD) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_GPS) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_ABRP) ? "on" : "off");
    return true;
  }
  // CAN comparer
  if (cmd.equalsIgnoreCase("compare"))
  {
    if (commInterface != nullptr)
      commInterface->compareCanRecords();
    return true;
  }

  if (cmd.equalsIgnoreCase("testMqtt") || cmd.equalsIgnoreCase("sendMqtt"))
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      syslog->print("WiFi not connected. Status: ");
      syslog->println(WiFi.status());
      return true;
    }
    bool tempEnabled = false;
    if (liveData->settings.mqttEnabled != 1)
    {
      syslog->println("Note: MQTT is disabled (mqttEnabled=0). Enabling temporarily for test...");
      liveData->settings.mqttEnabled = 1;
      tempEnabled = true;
    }
    syslog->println("Triggering MQTT send test...");
    bool prevDebugNet = (liveData->settings.debugLevel & DEBUG_NET) != 0;
    liveData->settings.debugLevel |= DEBUG_NET;
    syslog->setDebugLevel(liveData->settings.debugLevel);
    bool res = netSendData(false);
    if (!prevDebugNet)
    {
      liveData->settings.debugLevel &= ~DEBUG_NET;
      syslog->setDebugLevel(liveData->settings.debugLevel);
    }
    if (tempEnabled)
    {
      liveData->settings.mqttEnabled = 0;
      disconnectMqtt(false);
      syslog->println("Note: MQTT reverted to disabled (run 'mqttEnabled=1' or use menu to enable for background upload).");
    }
    syslog->print("MQTT test finished. Result: ");
    syslog->println(res ? "OK" : "FAILED");
    return true;
  }

  // MQTT getters
  if (cmd.equalsIgnoreCase("mqtt") || cmd.equalsIgnoreCase("showMqtt"))
  {
    syslog->println("MQTT settings:");
    syslog->printf("  enabled:  %s\n", (liveData->settings.mqttEnabled == 1) ? "ON" : "OFF");
    syslog->printf("  secure:   %s\n", (liveData->settings.mqttUseTls == 1) ? "ON (TLS)" : "OFF (plain)");
    syslog->printf("  server:   %s\n", liveData->settings.mqttServer);
    if (liveData->settings.mqttPort == 0)
    {
      syslog->printf("  port:     0 (default: %u)\n", (liveData->settings.mqttUseTls == 1) ? 8883 : 1883);
    }
    else
    {
      syslog->printf("  port:     %u\n", liveData->settings.mqttPort);
    }
    syslog->printf("  id:       %s\n", liveData->settings.mqttId);
    syslog->printf("  user:     %s\n", liveData->settings.mqttUsername);
    syslog->printf("  passwd:   %s\n", (strlen(liveData->settings.mqttPassword) > 0) ? "******" : "(empty)");
    syslog->printf("  topic:    %s\n", liveData->settings.mqttPubTopic);
    syslog->printf("  haName:   %s\n", (strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : "(unset, uses mqttId)");
    syslog->printf("  haModel:  %s\n", (strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel : "(unset, uses mqttTopic)");
    syslog->printf("  interval: %u sec%s\n",
                   liveData->settings.remoteUploadIntervalSec,
                   (liveData->settings.remoteUploadIntervalSec == 0) ? " (auto 60s)" : "");
    syslog->printf("  homeassistant: %s\n", (liveData->settings.mqttHomeAssistant == 1) ? "ON" : "OFF");
    return true;
  }
  if (cmd.equalsIgnoreCase("haName"))
  {
    syslog->print("HA Name: ");
    syslog->println((strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : "(unset, uses mqttId)");
    return true;
  }
  if (cmd.equalsIgnoreCase("haModel"))
  {
    syslog->print("HA Model: ");
    syslog->println((strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel : "(unset, uses mqttTopic)");
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttInterval"))
  {
    syslog->print("MQTT upload interval: ");
    if (liveData->settings.remoteUploadIntervalSec == 0)
    {
      syslog->println("0 (auto 60s)");
    }
    else
    {
      syslog->printf("%u sec\n", liveData->settings.remoteUploadIntervalSec);
    }
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttHa") || cmd.equalsIgnoreCase("mqttHomeAssistant"))
  {
    syslog->print("MQTT Home Assistant discovery: ");
    syslog->println((liveData->settings.mqttHomeAssistant == 1) ? "ON" : "OFF");
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttEnabled"))
  {
    syslog->print("MQTT enabled: ");
    syslog->println((liveData->settings.mqttEnabled == 1) ? "ON" : "OFF");
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttSecure") || cmd.equalsIgnoreCase("mqttTls") || cmd.equalsIgnoreCase("mqttUseTls"))
  {
    syslog->print("MQTT TLS/secure: ");
    syslog->println((liveData->settings.mqttUseTls == 1) ? "ON" : "OFF");
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttPort"))
  {
    syslog->print("MQTT port: ");
    if (liveData->settings.mqttPort == 0)
    {
      syslog->printf("0 (default: %u)\n", (liveData->settings.mqttUseTls == 1) ? 8883 : 1883);
    }
    else
    {
      syslog->println(liveData->settings.mqttPort);
    }
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttServer"))
  {
    syslog->print("MQTT server: ");
    syslog->println(liveData->settings.mqttServer);
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttId"))
  {
    syslog->print("MQTT id: ");
    syslog->println(liveData->settings.mqttId);
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttUsername") || cmd.equalsIgnoreCase("mqttUser"))
  {
    syslog->print("MQTT username: ");
    syslog->println(liveData->settings.mqttUsername);
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttPassword") || cmd.equalsIgnoreCase("mqttPasswd"))
  {
    syslog->print("MQTT password: ");
    syslog->println((strlen(liveData->settings.mqttPassword) > 0) ? "******" : "(empty)");
    return true;
  }
  if (cmd.equalsIgnoreCase("mqttPubTopic") || cmd.equalsIgnoreCase("mqttTopic"))
  {
    syslog->print("MQTT topic: ");
    syslog->println(liveData->settings.mqttPubTopic);
    return true;
  }

  int8_t idx = cmd.indexOf("=");
  if (idx == -1)
    return false;

  String key = cmd.substring(0, idx);
  String value = cmd.substring(idx + 1);
  key.trim();
  value.trim();

  // Bounded to destination size (truncates + NUL-terminates); an over-length value
  // would otherwise overflow into adjacent settings fields (issue #123).
  if (key.equalsIgnoreCase("serviceUUID"))
  {
    value.toCharArray(liveData->settings.serviceUUID, sizeof(liveData->settings.serviceUUID));
    return true;
  }
  if (key.equalsIgnoreCase("charTxUUID"))
  {
    value.toCharArray(liveData->settings.charTxUUID, sizeof(liveData->settings.charTxUUID));
    return true;
  }
  if (key.equalsIgnoreCase("charRxUUID"))
  {
    value.toCharArray(liveData->settings.charRxUUID, sizeof(liveData->settings.charRxUUID));
    return true;
  }

  if (key.equalsIgnoreCase("wifiSsid"))
  {
    value.toCharArray(liveData->settings.wifiSsid, sizeof(liveData->settings.wifiSsid));
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword"))
  {
    value.toCharArray(liveData->settings.wifiPassword, sizeof(liveData->settings.wifiPassword));
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid2"))
  {
    value.toCharArray(liveData->settings.wifiSsid2, sizeof(liveData->settings.wifiSsid2));
    if (strcmp(liveData->settings.wifiSsid2, "empty") == 0)
    {
      liveData->settings.backupWifiEnabled = 0;
    }
    else
    {
      liveData->settings.backupWifiEnabled = 1;
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword2"))
  {
    value.toCharArray(liveData->settings.wifiPassword2, sizeof(liveData->settings.wifiPassword2));
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid3"))
  {
    value.toCharArray(liveData->settings.wifiSsid3, sizeof(liveData->settings.wifiSsid3));
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword3"))
  {
    value.toCharArray(liveData->settings.wifiPassword3, sizeof(liveData->settings.wifiPassword3));
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid4"))
  {
    value.toCharArray(liveData->settings.wifiSsid4, sizeof(liveData->settings.wifiSsid4));
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword4"))
  {
    value.toCharArray(liveData->settings.wifiPassword4, sizeof(liveData->settings.wifiPassword4));
    return true;
  }
  if (key.equalsIgnoreCase("apPassword"))
  {
    if (value.length() < 8)
    {
      syslog->println("Error: AP password must be at least 8 characters");
      return true;
    }
    if (!isValidPassword(value.c_str(), 8))
    {
      syslog->println("Error: AP password must contain only alphanumeric characters");
      return true;
    }
    value.toCharArray(liveData->settings.webLogServerPassword, sizeof(liveData->settings.webLogServerPassword));
    syslog->print("AP password set to: ");
    syslog->println(liveData->settings.webLogServerPassword);
    saveSettings();
    return true;
  }
  if (key.equalsIgnoreCase("bleAddressType") || key.equalsIgnoreCase("bleAddrType"))
  {
    if (value.equalsIgnoreCase("public") || value.equalsIgnoreCase("1") || value.equalsIgnoreCase("pub"))
    {
      liveData->settings.bleAddressType = BLE_ADDRESS_TYPE_PUBLIC;
    }
    else
    {
      liveData->settings.bleAddressType = BLE_ADDRESS_TYPE_RANDOM;
    }
    syslog->print("BLE MAC address type set to: ");
    syslog->println((liveData->settings.bleAddressType == BLE_ADDRESS_TYPE_PUBLIC) ? "PUBLIC (fallback RANDOM)" : "RANDOM (fallback PUBLIC)");
    saveSettings();
    return true;
  }
  if (key.equalsIgnoreCase("remoteApiUrl"))
  {
    value.toCharArray(liveData->settings.remoteApiUrl, sizeof(liveData->settings.remoteApiUrl));
    return true;
  }
  if (key.equalsIgnoreCase("remoteApiKey"))
  {
    value.toCharArray(liveData->settings.remoteApiKey, sizeof(liveData->settings.remoteApiKey));
    return true;
  }
  if (key.equalsIgnoreCase("abrpApiToken"))
  {
    value.toCharArray(liveData->settings.abrpApiToken, sizeof(liveData->settings.abrpApiToken));
    return true;
  }

  // Mqtt
  if (key.equalsIgnoreCase("mqttEnabled"))
  {
    liveData->settings.mqttEnabled = (value == "1" || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("yes") || value.equalsIgnoreCase("on")) ? 1 : 0;
    syslog->print("MQTT enabled set to: ");
    syslog->println((liveData->settings.mqttEnabled == 1) ? "ON" : "OFF");
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttHa") || key.equalsIgnoreCase("mqttHomeAssistant"))
  {
    liveData->settings.mqttHomeAssistant = (value == "1" || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("yes") || value.equalsIgnoreCase("on")) ? 1 : 0;
    syslog->print("MQTT Home Assistant discovery set to: ");
    syslog->println((liveData->settings.mqttHomeAssistant == 1) ? "ON" : "OFF");
    saveSettings();
    if (liveData->settings.mqttHomeAssistant == 1)
    {
      if (liveData->settings.mqttEnabled == 1)
      {
        publishHomeAssistantDiscovery();
      }
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttSecure") || key.equalsIgnoreCase("mqttTls") || key.equalsIgnoreCase("mqttUseTls"))
  {
    liveData->settings.mqttUseTls = (value == "1" || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("yes") || value.equalsIgnoreCase("on")) ? 1 : 0;
    syslog->print("MQTT TLS/secure set to: ");
    syslog->println((liveData->settings.mqttUseTls == 1) ? "ON" : "OFF");
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttPort"))
  {
    long port = value.toInt();
    if (port < 0)
      port = 0;
    if (port > 65535)
      port = 65535;
    liveData->settings.mqttPort = static_cast<uint16_t>(port);
    syslog->print("MQTT port set to: ");
    if (liveData->settings.mqttPort == 0)
    {
      syslog->println("0 (default)");
    }
    else
    {
      syslog->println(liveData->settings.mqttPort);
    }
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttServer"))
  {
    value.toCharArray(liveData->settings.mqttServer, sizeof(liveData->settings.mqttServer));
    syslog->print("MQTT server set to: ");
    syslog->println(liveData->settings.mqttServer);
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttId"))
  {
    value.toCharArray(liveData->settings.mqttId, sizeof(liveData->settings.mqttId));
    syslog->print("MQTT id set to: ");
    syslog->println(liveData->settings.mqttId);
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttUsername") || key.equalsIgnoreCase("mqttUser"))
  {
    value.toCharArray(liveData->settings.mqttUsername, sizeof(liveData->settings.mqttUsername));
    syslog->print("MQTT username set to: ");
    syslog->println(liveData->settings.mqttUsername);
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttPassword") || key.equalsIgnoreCase("mqttPasswd"))
  {
    value.toCharArray(liveData->settings.mqttPassword, sizeof(liveData->settings.mqttPassword));
    syslog->print("MQTT password set to: ");
    syslog->println((strlen(liveData->settings.mqttPassword) > 0) ? "******" : "(empty)");
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("mqttPubTopic") || key.equalsIgnoreCase("mqttTopic"))
  {
    value.toCharArray(liveData->settings.mqttPubTopic, sizeof(liveData->settings.mqttPubTopic));
    syslog->print("MQTT topic set to: ");
    syslog->println(liveData->settings.mqttPubTopic);
    saveSettings();
    disconnectMqtt(false);
    return true;
  }
  if (key.equalsIgnoreCase("haName"))
  {
    value.toCharArray(liveData->settings.haName, sizeof(liveData->settings.haName));
    syslog->print("HA Name set to: ");
    syslog->println((strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : "(unset, uses mqttId)");
    saveSettings();
    if (liveData->settings.mqttHomeAssistant == 1 && liveData->settings.mqttEnabled == 1)
    {
      publishHomeAssistantDiscovery();
    }
    return true;
  }
  if (key.equalsIgnoreCase("haModel"))
  {
    value.toCharArray(liveData->settings.haModel, sizeof(liveData->settings.haModel));
    syslog->print("HA Model set to: ");
    syslog->println((strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel : "(unset, uses mqttTopic)");
    saveSettings();
    if (liveData->settings.mqttHomeAssistant == 1 && liveData->settings.mqttEnabled == 1)
    {
      publishHomeAssistantDiscovery();
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttInterval") || key.equalsIgnoreCase("remoteUploadIntervalSec"))
  {
    long interval = value.toInt();
    if (interval < 0)
      interval = 0;
    if (interval > 3600)
      interval = 3600;
    liveData->settings.remoteUploadIntervalSec = static_cast<uint16_t>(interval);
    syslog->print("MQTT upload interval set to: ");
    if (liveData->settings.remoteUploadIntervalSec == 0)
    {
      syslog->println("0 (auto 60s)");
    }
    else
    {
      syslog->printf("%u sec\n", liveData->settings.remoteUploadIntervalSec);
    }
    saveSettings();
    return true;
  }

  //
  if (key.equalsIgnoreCase("debugLevel"))
  {
    if (value.equalsIgnoreCase("all") || value == "255")
    {
      liveData->settings.debugLevel = DEBUG_COMM | DEBUG_NET | DEBUG_SDCARD | DEBUG_GPS | DEBUG_ABRP;
    }
    else if (value.equalsIgnoreCase("none") || value.equalsIgnoreCase("off") || value == "0")
    {
      liveData->settings.debugLevel = DEBUG_NONE;
    }
    else if (value.indexOf(',') != -1 || value.startsWith("+") || value.startsWith("-"))
    {
      int start = 0;
      while (start < value.length())
      {
        int comma = value.indexOf(',', start);
        String token = (comma == -1) ? value.substring(start) : value.substring(start, comma);
        token.trim();
        bool remove = token.startsWith("-");
        if (token.startsWith("+") || token.startsWith("-"))
          token = token.substring(1);
        uint8_t bit = 0;
        if (token.equalsIgnoreCase("comm"))
          bit = DEBUG_COMM;
        else if (token.equalsIgnoreCase("net") || token.equalsIgnoreCase("gsm"))
          bit = DEBUG_NET;
        else if (token.equalsIgnoreCase("sd") || token.equalsIgnoreCase("sdcard"))
          bit = DEBUG_SDCARD;
        else if (token.equalsIgnoreCase("gps"))
          bit = DEBUG_GPS;
        else if (token.equalsIgnoreCase("abrp"))
          bit = DEBUG_ABRP;
        else if (token.equalsIgnoreCase("all"))
          bit = DEBUG_COMM | DEBUG_NET | DEBUG_SDCARD | DEBUG_GPS | DEBUG_ABRP;
        if (remove)
          liveData->settings.debugLevel &= ~bit;
        else
          liveData->settings.debugLevel |= bit;
        if (comma == -1)
          break;
        start = comma + 1;
      }
    }
    else if (value.equalsIgnoreCase("comm"))
    {
      liveData->settings.debugLevel = DEBUG_COMM;
    }
    else if (value.equalsIgnoreCase("net") || value.equalsIgnoreCase("gsm"))
    {
      liveData->settings.debugLevel = DEBUG_NET;
    }
    else if (value.equalsIgnoreCase("sd") || value.equalsIgnoreCase("sdcard"))
    {
      liveData->settings.debugLevel = DEBUG_SDCARD;
    }
    else if (value.equalsIgnoreCase("gps"))
    {
      liveData->settings.debugLevel = DEBUG_GPS;
    }
    else if (value.equalsIgnoreCase("abrp"))
    {
      liveData->settings.debugLevel = DEBUG_ABRP;
    }
    else
    {
      liveData->settings.debugLevel = value.toInt();
    }
    syslog->setDebugLevel(liveData->settings.debugLevel);
    saveSettings();
    syslog->printf("Debug level set to: %u (comm=%s, net=%s, sd=%s, gps=%s, abrp=%s)\n",
                   liveData->settings.debugLevel,
                   (liveData->settings.debugLevel & DEBUG_COMM) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_NET) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_SDCARD) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_GPS) ? "on" : "off",
                   (liveData->settings.debugLevel & DEBUG_ABRP) ? "on" : "off");
    return true;
  }
  if (key.equalsIgnoreCase("setTime"))
  {
    setTime(value);
    return true;
  }
  // CAN comparer
  if (key.equalsIgnoreCase("record"))
  {
    if (commInterface != nullptr)
      commInterface->recordLoop(value.toInt());
    return true;
  }
  if (key.equalsIgnoreCase("test"))
  {
    if (carInterface != nullptr)
      carInterface->testHandler(value);
    return true;
  }

  return false;
}

/**
 *  Parser response from obd2/can
 */
void BoardInterface::parseRowMerged()
{
  carInterface->parseRowMerged();
}

/**
 * Serialize parameters for abrp/remote upload/sdcard
 */
namespace
{
  void populateParamsJson(LiveData *liveData, StaticJsonDocument<4096> &jsonData, bool inclApiKey)
  {
    if (inclApiKey)
      jsonData["apiKey"] = liveData->settings.remoteApiKey;

    jsonData["carType"] = liveData->settings.carType;
    jsonData["batTotalKwh"] = liveData->params.batteryTotalAvailableKWh;
    jsonData["currTime"] = liveData->params.currentTime + (liveData->settings.timezone * 3600) + (liveData->settings.daylightSaving * 3600);
    jsonData["opTime"] = liveData->params.operationTimeSec;

    jsonData["gpsSat"] = liveData->params.gpsSat;
    jsonData["lat"] = liveData->params.gpsLat;
    jsonData["lon"] = liveData->params.gpsLon;
    jsonData["alt"] = liveData->params.gpsAlt;
    jsonData["speedKmhGPS"] = liveData->params.speedKmhGPS;
    jsonData["gpsHeading"] = liveData->params.gpsHeadingDeg;

    jsonData["ignitionOn"] = liveData->params.ignitionOn;
    jsonData["chargingOn"] = liveData->params.chargingOn;

    jsonData["socPerc"] = liveData->params.socPerc;
    jsonData["socPercBms"] = liveData->params.socPercBms;
    jsonData["sohPerc"] = liveData->params.sohPerc;
    jsonData["powKwh100"] = liveData->params.batPowerKwh100;
    jsonData["speedKmh"] = liveData->params.speedKmh;
    jsonData["motorRpm"] = liveData->params.motor1Rpm;
    jsonData["motor2Rpm"] = liveData->params.motor2Rpm;
    jsonData["motorTqNm"] = liveData->params.motor1TorqueNm;
    jsonData["motor2TqNm"] = liveData->params.motor2TorqueNm;
    jsonData["odoKm"] = liveData->params.odoKm;

    if (liveData->params.batEnergyContent != 1)
      jsonData["batEneWh"] = liveData->params.batEnergyContent;
    if (liveData->params.batMaxEnergyContent != 1)
      jsonData["batMaxEneWh"] = liveData->params.batMaxEnergyContent;

    jsonData["batPowKw"] = liveData->params.batPowerKw;
    jsonData["batPowA"] = liveData->params.batPowerAmp;
    jsonData["batV"] = liveData->params.batVoltage;
    jsonData["cecKwh"] = liveData->params.cumulativeEnergyChargedKWh;
    jsonData["cedKwh"] = liveData->params.cumulativeEnergyDischargedKWh;
    jsonData["cccAh"] = liveData->params.cumulativeChargeCurrentAh;
    jsonData["cdcAh"] = liveData->params.cumulativeDischargeCurrentAh;
    jsonData["maxChKw"] = liveData->params.availableChargePower;
    jsonData["maxDisKw"] = liveData->params.availableDischargePower;

    jsonData["cellMinV"] = liveData->params.batCellMinV;
    jsonData["cellMaxV"] = liveData->params.batCellMaxV;
    if (liveData->params.batCellMinVNo != 255)
      jsonData["cellMinVNo"] = liveData->params.batCellMinVNo;
    if (liveData->params.batCellMaxVNo != 255)
      jsonData["cellMaxVNo"] = liveData->params.batCellMaxVNo;
    jsonData["bMinC"] = round(liveData->params.batMinC);
    jsonData["bMaxC"] = round(liveData->params.batMaxC);
    jsonData["bHeatC"] = round(liveData->params.batHeaterC);
    jsonData["bInletC"] = round(liveData->params.batInletC);
    jsonData["bFanSt"] = liveData->params.batFanStatus;
    jsonData["bWatC"] = round(liveData->params.coolingWaterTempC);
    jsonData["tmpA"] = round(liveData->params.bmsUnknownTempA);
    jsonData["tmpB"] = round(liveData->params.bmsUnknownTempB);
    jsonData["tmpC"] = round(liveData->params.bmsUnknownTempC);
    jsonData["tmpD"] = round(liveData->params.bmsUnknownTempD);
    if (liveData->params.normalChargePort > -50.0f)
      jsonData["chgPortAc"] = round(liveData->params.normalChargePort);
    if (liveData->params.rapidChargePort > -50.0f)
      jsonData["chgPortDc"] = round(liveData->params.rapidChargePort);
    if (liveData->params.chargerVoltage > 0.0f)
      jsonData["chgV"] = liveData->params.chargerVoltage;
    if (liveData->params.chargerCurrent > 0.0f)
      jsonData["chgA"] = liveData->params.chargerCurrent;

    jsonData["invC"] = round(liveData->params.inverterTempC);
    jsonData["motC"] = round(liveData->params.motorTempC);

    jsonData["auxPerc"] = liveData->params.auxPerc;
    jsonData["auxV"] = liveData->params.auxVoltage;
    jsonData["auxA"] = liveData->params.auxCurrentAmp;

    jsonData["inC"] = liveData->params.indoorTemperature;
    jsonData["outC"] = liveData->params.outdoorTemperature;
    jsonData["evapC"] = liveData->params.evaporatorTempC;
    jsonData["c1C"] = liveData->params.coolantTemp1C;
    jsonData["c2C"] = liveData->params.coolantTemp2C;

    jsonData["tFlC"] = liveData->params.tireFrontLeftTempC;
    jsonData["tFlBar"] = round(liveData->params.tireFrontLeftPressureBar * 10) / 10;
    jsonData["tFrC"] = liveData->params.tireFrontRightTempC;
    jsonData["tFrBar"] = round(liveData->params.tireFrontRightPressureBar * 10) / 10;
    jsonData["tRlC"] = liveData->params.tireRearLeftTempC;
    jsonData["tRlBar"] = round(liveData->params.tireRearLeftPressureBar * 10) / 10;
    jsonData["tRrC"] = liveData->params.tireRearRightTempC;
    jsonData["tRrBar"] = round(liveData->params.tireRearRightPressureBar * 10) / 10;
    jsonData["brakeL"] = liveData->params.brakeLights;

    jsonData["bmMode"] = liveData->getBatteryManagementModeStr(liveData->params.batteryManagementMode);

    // cell voltage
    for (int i = 0; i < liveData->params.cellCount; i++)
    {
      if (liveData->params.cellVoltage[i] == -1)
        continue;
      char key[8] = {0};
      snprintf(key, sizeof(key), "c%dV", i);
      jsonData[key] = liveData->params.cellVoltage[i];
    }
  }
} // namespace

bool BoardInterface::serializeParamsToJson(File file, bool inclApiKey)
{
  StaticJsonDocument<4096> jsonData;
  populateParamsJson(liveData, jsonData, inclApiKey);
  serializeJson(jsonData, Serial);
  serializeJson(jsonData, file);

  return true;
}

bool BoardInterface::serializeParamsToJson(String &outJson, bool inclApiKey)
{
  StaticJsonDocument<4096> jsonData;
  populateParamsJson(liveData, jsonData, inclApiKey);
  outJson = "";
  serializeJson(jsonData, outJson);
  return true;
}

/**
 * Show Network Settings
 */
void BoardInterface::showNet()
{
  syslog->print("wifiSsid:  ");
  syslog->println(liveData->settings.wifiSsid);

  if (liveData->settings.backupWifiEnabled == 1)
  {
    syslog->print("wifiSsid2: ");
    syslog->println(liveData->settings.wifiSsid2);
  }
  if (strlen(liveData->settings.wifiSsid3) > 0 && strcmp(liveData->settings.wifiSsid3, "empty") != 0)
  {
    syslog->print("wifiSsid3: ");
    syslog->println(liveData->settings.wifiSsid3);
  }
  if (strlen(liveData->settings.wifiSsid4) > 0 && strcmp(liveData->settings.wifiSsid4, "empty") != 0)
  {
    syslog->print("wifiSsid4: ");
    syslog->println(liveData->settings.wifiSsid4);
  }

  String activeStr = "main";
  if (liveData->params.wifiActiveIndex == 1)
    activeStr = "2nd AP (SSID2)";
  else if (liveData->params.wifiActiveIndex == 2)
    activeStr = "3rd AP (SSID3)";
  else if (liveData->params.wifiActiveIndex == 3)
    activeStr = "4th AP (SSID4)";

  syslog->print("Active: ");
  syslog->println(activeStr);
  syslog->print("IP-Address: ");
  syslog->println(WiFi.localIP().toString());
}

/**
 * Show time
 */
void BoardInterface::showTime()
{
  struct tm now;
  if (getLocalTime(&now, 0))
  {
    char dts[32];
    strftime(dts, sizeof(dts), "%Y-%m-%d %X", &now);
    syslog->print("Current time: ");
    syslog->println(dts);
  }
  else
  {
    syslog->println("Current time: not set");
  }
}

/**
 * Show console commands help
 */
void BoardInterface::showHelp()
{
  syslog->println("");
  syslog->println(".-[ HELP: Console commands ]-_.");
  syslog->println("help           ... show console commands help (aliases: commands, ?)");
  syslog->println("reboot         ... reboot device");
  syslog->println("shutdown       ... shutdown device");
  syslog->println("saveSettings   ... save current settings");
  syslog->println("factoryReset   ... reset settings to defaults");
  syslog->println("ipconfig       ... print network settings");
  syslog->println("ABRP_debug     ... print ABRP user token");
  syslog->println("debugLevel     ... print current debug level bitmask");
  syslog->println("debugLevel=n   ... set debug level bitmask: 0=none, 1=comm, 2=net, 4=sd, 8=gps, 16=abrp (e.g. 9=comm+gps, all, none)");
  syslog->println("wifiSsid=x     ... set primary AP ssid");
  syslog->println("wifiPassword=x ... set primary AP password");
  syslog->println("wifiSsid2=x    ... set 2nd AP ssid (replace primary wifi automatically in 1-2 minutes)");
  syslog->println("wifiPassword2=x... set 2nd AP password");
  syslog->println("wifiSsid3=x    ... set 3rd AP ssid");
  syslog->println("wifiPassword3=x... set 3rd AP password");
  syslog->println("wifiSsid4=x    ... set 4th AP ssid");
  syslog->println("wifiPassword4=x... set 4th AP password");
  syslog->println("apPassword     ... print current web log server AP password");
  syslog->println("apPassword=x   ... set web log server AP password (min 8 alphanumeric chars)");
  syslog->println("abrpApiToken=x ... set abrp api token for live data");
  syslog->println("remoteApiUrl=x ... set remote api url");
  syslog->println("remoteApiKey=x ... set remote api key");
  syslog->println("testMqtt       ... test MQTT connection and send heartbeat now");
  syslog->println("mqtt           ... print all MQTT settings");
  syslog->println("mqttEnabled[=0|1] ... get/set MQTT upload");
  syslog->println("mqttSecure[=0|1]  ... get/set MQTT TLS/SSL encryption");
  syslog->println("mqttServer[=x] ... get/set MQTT server");
  syslog->println("mqttPort[=x]   ... get/set MQTT port (0 = default 1883/8883)");
  syslog->println("mqttId[=x]     ... get/set MQTT id");
  syslog->println("mqttUsername[=x] ... get/set MQTT username");
  syslog->println("mqttPassword[=x] ... get/set MQTT password");
  syslog->println("mqttPubTopic[=x] ... get/set MQTT publish topic");
  syslog->println("mqttHa[=0|1]      ... get/set Home Assistant MQTT autodiscovery");
  syslog->println("serviceUUID=x  ... set device uuid for obd2 ble adapter");
  syslog->println("charTxUUID=x   ... set tx uuid for obd2 ble adapter");
  syslog->println("charRxUUID=x   ... set rx uuid for obd2 ble adapter");
  syslog->println("bleAddressType ... print current BLE MAC address type");
  syslog->println("bleAddressType=x ... set BLE MAC address type: random (0) or public (1)");
  syslog->println("obd2ip=x       ... set ip for obd2 wifi adapter");
  syslog->println("obd2port=x     ... set port for obd2 wifi adapter");
  syslog->println("time           ... print current time");
  syslog->println("ntpSync        ... sync Time with pool.ntp.org");
  syslog->println("setTime=2022-12-30 05:00:00  ... set current time");
  syslog->println("record=n       [n = 1..4]  ... record can response to buffer 1..4");
  syslog->println("compare        ... compare buffers");
  syslog->println("test=x         ... test handler");
  syslog->println("__________________________________________________");
}

/**
 * Set time
 */
void BoardInterface::setTime(String timestamp)
{
  struct timeval tv;
  struct tm tm_tmp;
  tm_tmp.tm_year = timestamp.substring(0, 4).toInt() - 1900;
  tm_tmp.tm_mon = timestamp.substring(5, 7).toInt() - 1;
  tm_tmp.tm_mday = timestamp.substring(8, 10).toInt();
  tm_tmp.tm_hour = timestamp.substring(11, 13).toInt();
  tm_tmp.tm_min = timestamp.substring(14, 16).toInt();
  tm_tmp.tm_sec = timestamp.substring(17, 19).toInt();

  time_t t = mktime(&tm_tmp);
  tv.tv_sec = t;

  settimeofday(&tv, NULL);
  struct tm tm;
  if (getLocalTime(&tm, 0))
  {
    liveData->params.currentTime = mktime(&tm);
  }
  liveData->params.chargingStartTime = liveData->params.currentTime;

  syslog->println("New time set. Only M5 Core2 is supported.");
  showTime();
}

/**
 * Automatic brightness by sunset/sunrise
 */
void BoardInterface::calcAutomaticBrightnessLatLon()
{
  if (liveData->settings.lcdBrightness == 0) // only for automatic mode
  {
    if (liveData->params.lcdBrightnessCalc == -1 && liveData->params.gpsLat != -1.0 && liveData->params.gpsLon != -1.0)
    {
      initSolarCalc(liveData->settings.timezone, liveData->params.gpsLat, liveData->params.gpsLon);
    }
    // angle from zenith
    // <70 = 100% brightnesss
    // >100 = 15%
    double sunDeg = getSZA(liveData->params.currentTime);
    syslog->infoNolf(DEBUG_GPS, "SUN from zenith, degrees: ");
    syslog->info(DEBUG_GPS, sunDeg);
    int32_t newBrightness = (105 - sunDeg) * 3.5;
    newBrightness = (newBrightness < 15 ? 15 : (newBrightness > 100) ? 100
                                                                   : newBrightness);
    if (liveData->params.lcdBrightnessCalc != newBrightness)
    {
      liveData->params.lcdBrightnessCalc = newBrightness;
      syslog->print("New automatic brightness: ");
      syslog->println(newBrightness);
      setBrightness();
    }
  }
}
