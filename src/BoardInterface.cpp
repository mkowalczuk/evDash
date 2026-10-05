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
#include "CarModelUtils.h"
#include "EvDashMobileRelay.h"

extern EvDashMobileRelay *mobileRelay;

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

  // MQTT getters summary
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

  if (cmd.equalsIgnoreCase("wifiScan") || cmd.equalsIgnoreCase("scanWifi"))
  {
    syslog->println("Scanning WiFi networks...");
    int n = WiFi.scanNetworks();
    syslog->printf("Found %d networks:\n", n);
    for (int i = 0; i < n; ++i)
    {
      syslog->printf("  %2d: %-32s (%4d dBm, ch %2d, %s)\n",
                     i + 1,
                     WiFi.SSID(i).c_str(),
                     WiFi.RSSI(i),
                     WiFi.channel(i),
                     (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "encrypted");
    }
    WiFi.scanDelete();
    return true;
  }

  if (cmd.equalsIgnoreCase("wifiConnect") || cmd.equalsIgnoreCase("wifiSetup"))
  {
    syslog->println("Triggering WiFi connect...");
    wifiSetup();
    return true;
  }

  if (cmd.equalsIgnoreCase("cars") || cmd.equalsIgnoreCase("listCars"))
  {
    syslog->println("Available vehicles (carType):");
    for (int i = 0; i <= 50; i++)
    {
      String abrp = getCarModelAbrpStr(i);
      if (abrp != "n/a")
      {
        syslog->printf("  %2d: %-32s (%s)\n", i, abrp.c_str(), getCarModelRelayId(i).c_str());
      }
    }
    return true;
  }

  if (cmd.equalsIgnoreCase("clearStats"))
  {
    liveData->clearDrivingAndChargingStats(CAR_MODE_DRIVE);
    syslog->println("Driving and charging stats cleared.");
    return true;
  }

  if (cmd.equalsIgnoreCase("loadTestData"))
  {
    if (carInterface != nullptr)
      carInterface->loadTestData();
    syslog->println("Test data loaded.");
    return true;
  }

  if (cmd.equalsIgnoreCase("mobileRelayPair"))
  {
    liveData->settings.relayForMobileEnabled = 1;
    saveSettings();
    String code = (mobileRelay != nullptr) ? mobileRelay->startPairing() : "";
    syslog->printf("Mobile relay pairing code: %s\n", code.c_str());
    return true;
  }

  if (cmd.equalsIgnoreCase("mobileRelayForget"))
  {
    if (mobileRelay != nullptr)
    {
      mobileRelay->forgetPairing();
    }
    else
    {
      liveData->settings.relayToken[0] = '\0';
      liveData->settings.relayMobileId[0] = '\0';
      saveSettings();
    }
    syslog->println("Mobile relay pairing forgotten.");
    return true;
  }

  if (cmd.equalsIgnoreCase("voltmeterInfo") || cmd.equalsIgnoreCase("voltmeter"))
  {
    if (liveData->settings.voltmeterEnabled == 1)
    {
      syslog->printf("Voltmeter enabled. Aux voltage: %.2fV\n", liveData->params.auxVoltage);
    }
    else
    {
      syslog->println("Voltmeter is disabled (voltmeterEnabled=0).");
    }
    return true;
  }

  if (cmd.equalsIgnoreCase("sdcardStatus") || cmd.equalsIgnoreCase("sdcardMount"))
  {
    bool mounted = sdcardMount();
    syslog->printf("SD card status: %s\n", mounted ? "mounted" : "not mounted / not available");
    return true;
  }

  if (cmd.equalsIgnoreCase("contributeOnce"))
  {
    syslog->println("Triggering contribute once...");
    liveData->params.netAvailable = true;
    liveData->params.lastContributeSent = liveData->params.currentTime;
    liveData->params.contributeStatus = CONTRIBUTE_READY_TO_SEND;
    return true;
  }

  int8_t idx = cmd.indexOf("=");
  bool isSetter = (idx != -1);
  String key = isSetter ? cmd.substring(0, idx) : cmd;
  String value = isSetter ? cmd.substring(idx + 1) : "";
  key.trim();
  value.trim();

  auto parseBool = [](const String &v) -> int8_t {
    if (v == "1" || v.equalsIgnoreCase("true") || v.equalsIgnoreCase("yes") || v.equalsIgnoreCase("on")) return 1;
    if (v == "0" || v.equalsIgnoreCase("false") || v.equalsIgnoreCase("no") || v.equalsIgnoreCase("off")) return 0;
    return -1;
  };

  // WiFi
  if (key.equalsIgnoreCase("wifiEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.wifiEnabled = b;
        saveSettings();
        syslog->printf("wifiEnabled set to: %s\n", (liveData->settings.wifiEnabled == 1) ? "ON (1)" : "OFF (0)");
        if (liveData->settings.wifiEnabled == 1)
        {
          wifiSetup();
        }
        else
        {
          WiFi.disconnect(true, false);
        }
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("wifiEnabled: %s\n", (liveData->settings.wifiEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiSsid, sizeof(liveData->settings.wifiSsid));
      saveSettings();
      syslog->printf("wifiSsid set to: %s\n", liveData->settings.wifiSsid);
      if (liveData->settings.wifiEnabled == 1)
      {
        wifiSetup();
      }
    }
    else
    {
      syslog->printf("wifiSsid: %s\n", liveData->settings.wifiSsid);
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiPassword, sizeof(liveData->settings.wifiPassword));
      saveSettings();
      syslog->println("wifiPassword updated.");
      if (liveData->settings.wifiEnabled == 1)
      {
        wifiSetup();
      }
    }
    else
    {
      syslog->printf("wifiPassword: %s\n", (strlen(liveData->settings.wifiPassword) > 0) ? "******" : "(empty)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("backupWifi") || key.equalsIgnoreCase("backupWifiEnabled") || key.equalsIgnoreCase("wifiEnabled2"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.backupWifiEnabled = b;
        saveSettings();
        syslog->printf("backupWifiEnabled set to: %s\n", (liveData->settings.backupWifiEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("backupWifiEnabled: %s\n", (liveData->settings.backupWifiEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid2"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiSsid2, sizeof(liveData->settings.wifiSsid2));
      if (strcmp(liveData->settings.wifiSsid2, "empty") == 0)
        liveData->settings.backupWifiEnabled = 0;
      else
        liveData->settings.backupWifiEnabled = 1;
      saveSettings();
      syslog->printf("wifiSsid2 set to: %s\n", liveData->settings.wifiSsid2);
    }
    else
    {
      syslog->printf("wifiSsid2: %s\n", liveData->settings.wifiSsid2);
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword2"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiPassword2, sizeof(liveData->settings.wifiPassword2));
      saveSettings();
      syslog->println("wifiPassword2 updated.");
    }
    else
    {
      syslog->printf("wifiPassword2: %s\n", (strlen(liveData->settings.wifiPassword2) > 0) ? "******" : "(empty)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid3"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiSsid3, sizeof(liveData->settings.wifiSsid3));
      saveSettings();
      syslog->printf("wifiSsid3 set to: %s\n", liveData->settings.wifiSsid3);
    }
    else
    {
      syslog->printf("wifiSsid3: %s\n", liveData->settings.wifiSsid3);
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword3"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiPassword3, sizeof(liveData->settings.wifiPassword3));
      saveSettings();
      syslog->println("wifiPassword3 updated.");
    }
    else
    {
      syslog->printf("wifiPassword3: %s\n", (strlen(liveData->settings.wifiPassword3) > 0) ? "******" : "(empty)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiSsid4"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiSsid4, sizeof(liveData->settings.wifiSsid4));
      saveSettings();
      syslog->printf("wifiSsid4 set to: %s\n", liveData->settings.wifiSsid4);
    }
    else
    {
      syslog->printf("wifiSsid4: %s\n", liveData->settings.wifiSsid4);
    }
    return true;
  }
  if (key.equalsIgnoreCase("wifiPassword4"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.wifiPassword4, sizeof(liveData->settings.wifiPassword4));
      saveSettings();
      syslog->println("wifiPassword4 updated.");
    }
    else
    {
      syslog->printf("wifiPassword4: %s\n", (strlen(liveData->settings.wifiPassword4) > 0) ? "******" : "(empty)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("apPassword"))
  {
    if (isSetter)
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
      saveSettings();
      syslog->printf("AP password set to: %s\n", liveData->settings.webLogServerPassword);
    }
    else
    {
      syslog->printf("AP password: %s\n", liveData->settings.webLogServerPassword);
    }
    return true;
  }
  if (key.equalsIgnoreCase("ntpEnabled") || key.equalsIgnoreCase("wifiNtp"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.ntpEnabled = b;
        saveSettings();
        syslog->printf("ntpEnabled set to: %s\n", (liveData->settings.ntpEnabled == 1) ? "ON (1)" : "OFF (0)");
        if (liveData->settings.ntpEnabled == 1)
          ntpSync();
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("ntpEnabled: %s\n", (liveData->settings.ntpEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }

  // Vehicle
  if (key.equalsIgnoreCase("carType"))
  {
    if (isSetter)
    {
      int val = -1;
      if (value.length() > 0 && isdigit(value[0]))
      {
        val = value.toInt();
      }
      else
      {
        for (int i = 0; i <= 50; i++)
        {
          String abrp = getCarModelAbrpStr(i);
          String relay = getCarModelRelayId(i);
          if (abrp != "n/a" && abrp.indexOf(value) != -1) { val = i; break; }
          if (relay != "unknown" && relay.indexOf(value) != -1) { val = i; break; }
        }
      }
      if (val >= 0)
      {
        liveData->settings.carType = static_cast<uint16_t>(val);
        saveSettings();
        syslog->printf("carType set to: %u (%s). Note: reboot required to apply.\n",
                       liveData->settings.carType, getCarModelAbrpStr(liveData->settings.carType).c_str());
      }
      else
      {
        syslog->println("Error: Unknown carType. Pass numeric ID or model string (run 'cars' to list).");
      }
    }
    else
    {
      syslog->printf("carType: %u (%s / %s)\n",
                     liveData->settings.carType,
                     getCarModelAbrpStr(liveData->settings.carType).c_str(),
                     getCarModelRelayId(liveData->settings.carType).c_str());
    }
    return true;
  }

  // OBD2 / CAN Adapter
  if (key.equalsIgnoreCase("commType"))
  {
    if (isSetter)
    {
      if (value.equalsIgnoreCase("can") || value == "1")
        liveData->settings.commType = COMM_TYPE_CAN_COMMU;
      else if (value.equalsIgnoreCase("ble") || value.equalsIgnoreCase("ble4") || value == "0")
        liveData->settings.commType = COMM_TYPE_OBD2_BLE4;
      else
      {
        syslog->println("Error: commType must be 0 (BLE) or 1 (CAN)");
        return true;
      }
      saveSettings();
      syslog->printf("commType set to: %u (%s). Note: reboot required to apply.\n",
                     liveData->settings.commType,
                     (liveData->settings.commType == COMM_TYPE_CAN_COMMU) ? "CAN" : "OBD2 BLE4");
    }
    else
    {
      syslog->printf("commType: %u (%s)\n",
                     liveData->settings.commType,
                     (liveData->settings.commType == COMM_TYPE_CAN_COMMU) ? "CAN" : "OBD2 BLE4");
    }
    return true;
  }
  if (key.equalsIgnoreCase("obdMacAddress") || key.equalsIgnoreCase("obdMac"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.obdMacAddress, sizeof(liveData->settings.obdMacAddress));
      saveSettings();
      syslog->printf("obdMacAddress set to: %s\n", liveData->settings.obdMacAddress);
    }
    else
    {
      syslog->printf("obdMacAddress: %s\n", liveData->settings.obdMacAddress);
    }
    return true;
  }
  if (key.equalsIgnoreCase("obd2Name") || key.equalsIgnoreCase("bleName"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.obd2Name, sizeof(liveData->settings.obd2Name));
      saveSettings();
      syslog->printf("obd2Name set to: %s\n", liveData->settings.obd2Name);
    }
    else
    {
      syslog->printf("obd2Name: %s\n", liveData->settings.obd2Name);
    }
    return true;
  }
  if (key.equalsIgnoreCase("serviceUUID"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.serviceUUID, sizeof(liveData->settings.serviceUUID));
      saveSettings();
      syslog->printf("serviceUUID set to: %s\n", liveData->settings.serviceUUID);
    }
    else
    {
      syslog->printf("serviceUUID: %s\n", liveData->settings.serviceUUID);
    }
    return true;
  }
  if (key.equalsIgnoreCase("charTxUUID"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.charTxUUID, sizeof(liveData->settings.charTxUUID));
      saveSettings();
      syslog->printf("charTxUUID set to: %s\n", liveData->settings.charTxUUID);
    }
    else
    {
      syslog->printf("charTxUUID: %s\n", liveData->settings.charTxUUID);
    }
    return true;
  }
  if (key.equalsIgnoreCase("charRxUUID"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.charRxUUID, sizeof(liveData->settings.charRxUUID));
      saveSettings();
      syslog->printf("charRxUUID set to: %s\n", liveData->settings.charRxUUID);
    }
    else
    {
      syslog->printf("charRxUUID: %s\n", liveData->settings.charRxUUID);
    }
    return true;
  }
  if (key.equalsIgnoreCase("bleAddressType") || key.equalsIgnoreCase("bleAddrType"))
  {
    if (isSetter)
    {
      if (value.equalsIgnoreCase("public") || value.equalsIgnoreCase("1") || value.equalsIgnoreCase("pub"))
      {
        liveData->settings.bleAddressType = BLE_ADDRESS_TYPE_PUBLIC;
      }
      else
      {
        liveData->settings.bleAddressType = BLE_ADDRESS_TYPE_RANDOM;
      }
      saveSettings();
      syslog->printf("BLE MAC address type set to: %s\n",
                     (liveData->settings.bleAddressType == BLE_ADDRESS_TYPE_PUBLIC) ? "PUBLIC (fallback RANDOM)" : "RANDOM (fallback PUBLIC)");
    }
    else
    {
      syslog->printf("BLE MAC address type: %s\n",
                     (liveData->settings.bleAddressType == BLE_ADDRESS_TYPE_PUBLIC) ? "PUBLIC (fallback RANDOM)" : "RANDOM (fallback PUBLIC)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("commandQueueAutoStop"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.commandQueueAutoStop = b;
        saveSettings();
        syslog->printf("commandQueueAutoStop set to: %s\n", (liveData->settings.commandQueueAutoStop == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("commandQueueAutoStop: %s\n", (liveData->settings.commandQueueAutoStop == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("disableCommandOptimizer"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.disableCommandOptimizer = b;
        saveSettings();
        syslog->printf("disableCommandOptimizer set to: %s\n", (liveData->settings.disableCommandOptimizer == 1) ? "1 (Optimizer disabled)" : "0 (Optimizer active)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("disableCommandOptimizer: %s\n", (liveData->settings.disableCommandOptimizer == 1) ? "1 (Optimizer disabled)" : "0 (Optimizer active)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mobileRelay") || key.equalsIgnoreCase("relayForMobileEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.relayForMobileEnabled = b;
        saveSettings();
        syslog->printf("relayForMobileEnabled set to: %s\n", (liveData->settings.relayForMobileEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("relayForMobileEnabled: %s\n", (liveData->settings.relayForMobileEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("relayToken"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.relayToken, sizeof(liveData->settings.relayToken));
      saveSettings();
      syslog->printf("relayToken set to: %s\n", liveData->settings.relayToken);
    }
    else
    {
      syslog->printf("relayToken: %s\n", liveData->settings.relayToken);
    }
    return true;
  }

  // Units
  if (key.equalsIgnoreCase("distanceUnit") || key.equalsIgnoreCase("distance"))
  {
    if (isSetter)
    {
      if (value.equalsIgnoreCase("m") || value.equalsIgnoreCase("mi") || value.equalsIgnoreCase("miles"))
        liveData->settings.distanceUnit = 'm';
      else
        liveData->settings.distanceUnit = 'k';
      saveSettings();
      syslog->printf("distanceUnit set to: %s\n", (liveData->settings.distanceUnit == 'm') ? "miles" : "km");
    }
    else
    {
      syslog->printf("distanceUnit: %s\n", (liveData->settings.distanceUnit == 'm') ? "miles" : "km");
    }
    return true;
  }
  if (key.equalsIgnoreCase("temperatureUnit") || key.equalsIgnoreCase("temperature"))
  {
    if (isSetter)
    {
      if (value.equalsIgnoreCase("f") || value.equalsIgnoreCase("fahrenheit"))
        liveData->settings.temperatureUnit = 'f';
      else
        liveData->settings.temperatureUnit = 'c';
      saveSettings();
      syslog->printf("temperatureUnit set to: %s\n", (liveData->settings.temperatureUnit == 'f') ? "Fahrenheit" : "Celsius");
    }
    else
    {
      syslog->printf("temperatureUnit: %s\n", (liveData->settings.temperatureUnit == 'f') ? "Fahrenheit" : "Celsius");
    }
    return true;
  }
  if (key.equalsIgnoreCase("pressureUnit") || key.equalsIgnoreCase("pressure"))
  {
    if (isSetter)
    {
      if (value.equalsIgnoreCase("p") || value.equalsIgnoreCase("psi"))
        liveData->settings.pressureUnit = 'p';
      else
        liveData->settings.pressureUnit = 'b';
      saveSettings();
      syslog->printf("pressureUnit set to: %s\n", (liveData->settings.pressureUnit == 'p') ? "psi" : "bar");
    }
    else
    {
      syslog->printf("pressureUnit: %s\n", (liveData->settings.pressureUnit == 'p') ? "psi" : "bar");
    }
    return true;
  }

  // Board Setup
  if (key.equalsIgnoreCase("boardPowerMode") || key.equalsIgnoreCase("powerMode"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.boardPowerMode = b;
        saveSettings();
        syslog->printf("boardPowerMode set to: %u (%s)\n", liveData->settings.boardPowerMode, (liveData->settings.boardPowerMode == 1) ? "external" : "USB");
      }
      else
      {
        syslog->println("Error: Use 1 (external) or 0 (USB)");
      }
    }
    else
    {
      syslog->printf("boardPowerMode: %u (%s)\n", liveData->settings.boardPowerMode, (liveData->settings.boardPowerMode == 1) ? "external" : "USB");
    }
    return true;
  }
  if (key.equalsIgnoreCase("timezone"))
  {
    if (isSetter)
    {
      liveData->settings.timezone = static_cast<int8_t>(value.toInt());
      saveSettings();
      syslog->printf("timezone set to: %d\n", liveData->settings.timezone);
    }
    else
    {
      syslog->printf("timezone: %d\n", liveData->settings.timezone);
    }
    return true;
  }
  if (key.equalsIgnoreCase("daylightSaving"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.daylightSaving = b;
        saveSettings();
        syslog->printf("daylightSaving set to: %s\n", (liveData->settings.daylightSaving == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("daylightSaving: %s\n", (liveData->settings.daylightSaving == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("defaultScreen"))
  {
    if (isSetter)
    {
      liveData->settings.defaultScreen = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("defaultScreen set to: %u\n", liveData->settings.defaultScreen);
    }
    else
    {
      syslog->printf("defaultScreen: %u\n", liveData->settings.defaultScreen);
    }
    return true;
  }
  if (key.equalsIgnoreCase("displayRotation") || key.equalsIgnoreCase("screenRotation"))
  {
    if (isSetter)
    {
      liveData->settings.displayRotation = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("displayRotation set to: %u\n", liveData->settings.displayRotation);
    }
    else
    {
      syslog->printf("displayRotation: %u\n", liveData->settings.displayRotation);
    }
    return true;
  }
  if (key.equalsIgnoreCase("lcdBrightness"))
  {
    if (isSetter)
    {
      liveData->settings.lcdBrightness = static_cast<uint8_t>(value.toInt());
      saveSettings();
      setBrightness();
      syslog->printf("lcdBrightness set to: %u%s\n", liveData->settings.lcdBrightness, (liveData->settings.lcdBrightness == 0) ? " (auto)" : "%");
    }
    else
    {
      syslog->printf("lcdBrightness: %u%s\n", liveData->settings.lcdBrightness, (liveData->settings.lcdBrightness == 0) ? " (auto)" : "%");
    }
    return true;
  }
  if (key.equalsIgnoreCase("sleepMode") || key.equalsIgnoreCase("sleepModeLevel"))
  {
    if (isSetter)
    {
      liveData->settings.sleepModeLevel = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("sleepModeLevel set to: %u (0=off, 1=screen only, 2=deep sleep)\n", liveData->settings.sleepModeLevel);
    }
    else
    {
      syslog->printf("sleepModeLevel: %u (0=off, 1=screen only, 2=deep sleep)\n", liveData->settings.sleepModeLevel);
    }
    return true;
  }
  if (key.equalsIgnoreCase("serialConsolePort"))
  {
    if (isSetter)
    {
      liveData->settings.serialConsolePort = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("serialConsolePort set to: %u\n", liveData->settings.serialConsolePort);
    }
    else
    {
      syslog->printf("serialConsolePort: %u\n", liveData->settings.serialConsolePort);
    }
    return true;
  }
  if (key.equalsIgnoreCase("speedCorrection"))
  {
    if (isSetter)
    {
      liveData->settings.speedCorrection = static_cast<int8_t>(value.toInt());
      saveSettings();
      syslog->printf("speedCorrection set to: %d\n", liveData->settings.speedCorrection);
    }
    else
    {
      syslog->printf("speedCorrection: %d\n", liveData->settings.speedCorrection);
    }
    return true;
  }
  if (key.equalsIgnoreCase("rightHandDrive") || key.equalsIgnoreCase("rhd"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.rightHandDrive = b;
        saveSettings();
        syslog->printf("rightHandDrive set to: %s\n", (liveData->settings.rightHandDrive == 1) ? "RHD (1)" : "LHD (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("rightHandDrive: %s\n", (liveData->settings.rightHandDrive == 1) ? "RHD (1)" : "LHD (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("gprsHwSerialPort"))
  {
    if (isSetter)
    {
      liveData->settings.gprsHwSerialPort = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("gprsHwSerialPort set to: %u\n", liveData->settings.gprsHwSerialPort);
    }
    else
    {
      syslog->printf("gprsHwSerialPort: %u\n", liveData->settings.gprsHwSerialPort);
    }
    return true;
  }

  // SD card
  if (key.equalsIgnoreCase("sdcardEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.sdcardEnabled = b;
        saveSettings();
        syslog->printf("sdcardEnabled set to: %s\n", (liveData->settings.sdcardEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("sdcardEnabled: %s\n", (liveData->settings.sdcardEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("sdcardConsoleLog") || key.equalsIgnoreCase("sdcardConsoleLogEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.sdcardConsoleLogEnabled = b;
        saveSettings();
        syslog->printf("sdcardConsoleLogEnabled set to: %s\n", (liveData->settings.sdcardConsoleLogEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("sdcardConsoleLogEnabled: %s\n", (liveData->settings.sdcardConsoleLogEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("sdcardAutstartLog"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.sdcardAutstartLog = b;
        saveSettings();
        syslog->printf("sdcardAutstartLog set to: %s\n", (liveData->settings.sdcardAutstartLog == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("sdcardAutstartLog: %s\n", (liveData->settings.sdcardAutstartLog == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }

  // GPS
  if (key.equalsIgnoreCase("gpsModuleType"))
  {
    if (isSetter)
    {
      liveData->settings.gpsModuleType = static_cast<uint8_t>(value.toInt());
      if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_NEO_M8N)
        liveData->settings.gpsSerialPortSpeed = 9600;
      else if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_M5_GNSS)
        liveData->settings.gpsSerialPortSpeed = 38400;
      else if (liveData->settings.gpsModuleType == GPS_MODULE_TYPE_GPS_V21_GNSS)
        liveData->settings.gpsSerialPortSpeed = 115200;
      saveSettings();
      syslog->printf("gpsModuleType set to: %u (baud %lu)\n", liveData->settings.gpsModuleType, liveData->settings.gpsSerialPortSpeed);
    }
    else
    {
      syslog->printf("gpsModuleType: %u\n", liveData->settings.gpsModuleType);
    }
    return true;
  }
  if (key.equalsIgnoreCase("gpsPort") || key.equalsIgnoreCase("gpsHwSerialPort"))
  {
    if (isSetter)
    {
      liveData->settings.gpsHwSerialPort = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("gpsHwSerialPort set to: %u\n", liveData->settings.gpsHwSerialPort);
    }
    else
    {
      syslog->printf("gpsHwSerialPort: %u\n", liveData->settings.gpsHwSerialPort);
    }
    return true;
  }
  if (key.equalsIgnoreCase("gpsSpeed") || key.equalsIgnoreCase("gpsSerialPortSpeed"))
  {
    if (isSetter)
    {
      liveData->settings.gpsSerialPortSpeed = value.toInt();
      saveSettings();
      syslog->printf("gpsSerialPortSpeed set to: %lu\n", liveData->settings.gpsSerialPortSpeed);
    }
    else
    {
      syslog->printf("gpsSerialPortSpeed: %lu\n", liveData->settings.gpsSerialPortSpeed);
    }
    return true;
  }
  if (key.equalsIgnoreCase("carSpeedType"))
  {
    if (isSetter)
    {
      liveData->settings.carSpeedType = static_cast<uint8_t>(value.toInt());
      saveSettings();
      syslog->printf("carSpeedType set to: %u (0=auto, 1=car, 2=gps)\n", liveData->settings.carSpeedType);
    }
    else
    {
      syslog->printf("carSpeedType: %u (0=auto, 1=car, 2=gps)\n", liveData->settings.carSpeedType);
    }
    return true;
  }

  // Voltmeter INA3221
  if (key.equalsIgnoreCase("voltmeterEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.voltmeterEnabled = b;
        saveSettings();
        syslog->printf("voltmeterEnabled set to: %s\n", (liveData->settings.voltmeterEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("voltmeterEnabled: %s\n", (liveData->settings.voltmeterEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("voltmeterSleep") || key.equalsIgnoreCase("voltmeterBasedSleep"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.voltmeterBasedSleep = b;
        saveSettings();
        syslog->printf("voltmeterBasedSleep set to: %s\n", (liveData->settings.voltmeterBasedSleep == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("voltmeterBasedSleep: %s\n", (liveData->settings.voltmeterBasedSleep == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("voltmeterSleepVol"))
  {
    if (isSetter)
    {
      liveData->settings.voltmeterSleep = value.toFloat();
      saveSettings();
      syslog->printf("voltmeterSleep set to: %.2fV\n", liveData->settings.voltmeterSleep);
    }
    else
    {
      syslog->printf("voltmeterSleep: %.2fV\n", liveData->settings.voltmeterSleep);
    }
    return true;
  }
  if (key.equalsIgnoreCase("voltmeterWakeUpVol"))
  {
    if (isSetter)
    {
      liveData->settings.voltmeterWakeUp = value.toFloat();
      saveSettings();
      syslog->printf("voltmeterWakeUp set to: %.2fV\n", liveData->settings.voltmeterWakeUp);
    }
    else
    {
      syslog->printf("voltmeterWakeUp: %.2fV\n", liveData->settings.voltmeterWakeUp);
    }
    return true;
  }
  if (key.equalsIgnoreCase("voltmeterCutOffVol"))
  {
    if (isSetter)
    {
      liveData->settings.voltmeterCutOff = value.toFloat();
      saveSettings();
      syslog->printf("voltmeterCutOff set to: %.2fV\n", liveData->settings.voltmeterCutOff);
    }
    else
    {
      syslog->printf("voltmeterCutOff: %.2fV\n", liveData->settings.voltmeterCutOff);
    }
    return true;
  }

  // Remote upload & ABRP
  if (key.equalsIgnoreCase("remoteUploadIntervalSec") || key.equalsIgnoreCase("apiInterval"))
  {
    if (isSetter)
    {
      liveData->settings.remoteUploadIntervalSec = static_cast<uint16_t>(value.toInt());
      saveSettings();
      syslog->printf("remoteUploadIntervalSec set to: %u\n", liveData->settings.remoteUploadIntervalSec);
    }
    else
    {
      syslog->printf("remoteUploadIntervalSec: %u\n", liveData->settings.remoteUploadIntervalSec);
    }
    return true;
  }
  if (key.equalsIgnoreCase("remoteUploadAbrpIntervalSec") || key.equalsIgnoreCase("abrpInterval"))
  {
    if (isSetter)
    {
      liveData->settings.remoteUploadAbrpIntervalSec = static_cast<uint16_t>(value.toInt());
      saveSettings();
      syslog->printf("remoteUploadAbrpIntervalSec set to: %u\n", liveData->settings.remoteUploadAbrpIntervalSec);
    }
    else
    {
      syslog->printf("remoteUploadAbrpIntervalSec: %u\n", liveData->settings.remoteUploadAbrpIntervalSec);
    }
    return true;
  }
  if (key.equalsIgnoreCase("remoteApiUrl"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.remoteApiUrl, sizeof(liveData->settings.remoteApiUrl));
      saveSettings();
      syslog->printf("remoteApiUrl set to: %s\n", liveData->settings.remoteApiUrl);
    }
    else
    {
      syslog->printf("remoteApiUrl: %s\n", liveData->settings.remoteApiUrl);
    }
    return true;
  }
  if (key.equalsIgnoreCase("remoteApiKey"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.remoteApiKey, sizeof(liveData->settings.remoteApiKey));
      saveSettings();
      syslog->printf("remoteApiKey set to: %s\n", liveData->settings.remoteApiKey);
    }
    else
    {
      syslog->printf("remoteApiKey: %s\n", liveData->settings.remoteApiKey);
    }
    return true;
  }
  if (key.equalsIgnoreCase("abrpApiToken"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.abrpApiToken, sizeof(liveData->settings.abrpApiToken));
      saveSettings();
      syslog->printf("abrpApiToken set to: %s\n", liveData->settings.abrpApiToken);
    }
    else
    {
      syslog->printf("abrpApiToken: %s\n", liveData->settings.abrpApiToken);
    }
    return true;
  }
  if (key.equalsIgnoreCase("abrpSdcardLog"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.abrpSdcardLog = b;
        saveSettings();
        syslog->printf("abrpSdcardLog set to: %s\n", (liveData->settings.abrpSdcardLog == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("abrpSdcardLog: %s\n", (liveData->settings.abrpSdcardLog == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("contributeData"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.contributeData = b;
        saveSettings();
        syslog->printf("contributeData set to: %s\n", (liveData->settings.contributeData == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("contributeData: %s\n", (liveData->settings.contributeData == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }

  // Traccar
  if (key.equalsIgnoreCase("traccarEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.traccarEnabled = b;
        saveSettings();
        syslog->printf("traccarEnabled set to: %s\n", (liveData->settings.traccarEnabled == 1) ? "ON (1)" : "OFF (0)");
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("traccarEnabled: %s\n", (liveData->settings.traccarEnabled == 1) ? "ON (1)" : "OFF (0)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("traccarServer") || key.equalsIgnoreCase("traccarServerHost"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.traccarServerHost, sizeof(liveData->settings.traccarServerHost));
      saveSettings();
      syslog->printf("traccarServer set to: %s\n", liveData->settings.traccarServerHost);
    }
    else
    {
      syslog->printf("traccarServer: %s\n", liveData->settings.traccarServerHost);
    }
    return true;
  }
  if (key.equalsIgnoreCase("traccarPort") || key.equalsIgnoreCase("traccarServerPort"))
  {
    if (isSetter)
    {
      liveData->settings.traccarServerPort = static_cast<uint16_t>(value.toInt());
      saveSettings();
      syslog->printf("traccarPort set to: %u\n", liveData->settings.traccarServerPort);
    }
    else
    {
      syslog->printf("traccarPort: %u\n", liveData->settings.traccarServerPort);
    }
    return true;
  }

  // MQTT
  if (key.equalsIgnoreCase("mqttEnabled"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.mqttEnabled = b;
        saveSettings();
        syslog->printf("MQTT enabled set to: %s\n", (liveData->settings.mqttEnabled == 1) ? "ON" : "OFF");
        disconnectMqtt(false);
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("MQTT enabled: %s\n", (liveData->settings.mqttEnabled == 1) ? "ON" : "OFF");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttHa") || key.equalsIgnoreCase("mqttHomeAssistant"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.mqttHomeAssistant = b;
        saveSettings();
        syslog->printf("MQTT Home Assistant discovery set to: %s\n", (liveData->settings.mqttHomeAssistant == 1) ? "ON" : "OFF");
        if (liveData->settings.mqttHomeAssistant == 1 && liveData->settings.mqttEnabled == 1)
        {
          publishHomeAssistantDiscovery();
        }
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("MQTT Home Assistant discovery: %s\n", (liveData->settings.mqttHomeAssistant == 1) ? "ON" : "OFF");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttSecure") || key.equalsIgnoreCase("mqttTls") || key.equalsIgnoreCase("mqttUseTls"))
  {
    if (isSetter)
    {
      int8_t b = parseBool(value);
      if (b != -1)
      {
        liveData->settings.mqttUseTls = b;
        saveSettings();
        syslog->printf("MQTT TLS/secure set to: %s\n", (liveData->settings.mqttUseTls == 1) ? "ON" : "OFF");
        disconnectMqtt(false);
      }
      else
      {
        syslog->println("Error: Use 1/0, on/off, true/false");
      }
    }
    else
    {
      syslog->printf("MQTT TLS/secure: %s\n", (liveData->settings.mqttUseTls == 1) ? "ON" : "OFF");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttPort"))
  {
    if (isSetter)
    {
      long port = value.toInt();
      if (port < 0) port = 0;
      if (port > 65535) port = 65535;
      liveData->settings.mqttPort = static_cast<uint16_t>(port);
      saveSettings();
      syslog->print("MQTT port set to: ");
      if (liveData->settings.mqttPort == 0)
        syslog->println("0 (default)");
      else
        syslog->println(liveData->settings.mqttPort);
      disconnectMqtt(false);
    }
    else
    {
      syslog->print("MQTT port: ");
      if (liveData->settings.mqttPort == 0)
        syslog->printf("0 (default: %u)\n", (liveData->settings.mqttUseTls == 1) ? 8883 : 1883);
      else
        syslog->println(liveData->settings.mqttPort);
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttServer"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.mqttServer, sizeof(liveData->settings.mqttServer));
      saveSettings();
      syslog->printf("MQTT server set to: %s\n", liveData->settings.mqttServer);
      disconnectMqtt(false);
    }
    else
    {
      syslog->printf("MQTT server: %s\n", liveData->settings.mqttServer);
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttId"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.mqttId, sizeof(liveData->settings.mqttId));
      saveSettings();
      syslog->printf("MQTT id set to: %s\n", liveData->settings.mqttId);
      disconnectMqtt(false);
    }
    else
    {
      syslog->printf("MQTT id: %s\n", liveData->settings.mqttId);
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttUsername") || key.equalsIgnoreCase("mqttUser"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.mqttUsername, sizeof(liveData->settings.mqttUsername));
      saveSettings();
      syslog->printf("MQTT username set to: %s\n", liveData->settings.mqttUsername);
      disconnectMqtt(false);
    }
    else
    {
      syslog->printf("MQTT username: %s\n", liveData->settings.mqttUsername);
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttPassword") || key.equalsIgnoreCase("mqttPasswd"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.mqttPassword, sizeof(liveData->settings.mqttPassword));
      saveSettings();
      syslog->printf("MQTT password set to: %s\n", (strlen(liveData->settings.mqttPassword) > 0) ? "******" : "(empty)");
      disconnectMqtt(false);
    }
    else
    {
      syslog->printf("MQTT password: %s\n", (strlen(liveData->settings.mqttPassword) > 0) ? "******" : "(empty)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttPubTopic") || key.equalsIgnoreCase("mqttTopic"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.mqttPubTopic, sizeof(liveData->settings.mqttPubTopic));
      saveSettings();
      syslog->printf("MQTT topic set to: %s\n", liveData->settings.mqttPubTopic);
      disconnectMqtt(false);
    }
    else
    {
      syslog->printf("MQTT topic: %s\n", liveData->settings.mqttPubTopic);
    }
    return true;
  }
  if (key.equalsIgnoreCase("haName"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.haName, sizeof(liveData->settings.haName));
      saveSettings();
      syslog->printf("HA Name set to: %s\n", (strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : "(unset, uses mqttId)");
      if (liveData->settings.mqttHomeAssistant == 1 && liveData->settings.mqttEnabled == 1)
      {
        publishHomeAssistantDiscovery();
      }
    }
    else
    {
      syslog->printf("HA Name: %s\n", (strlen(liveData->settings.haName) > 0) ? liveData->settings.haName : "(unset, uses mqttId)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("haModel"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.haModel, sizeof(liveData->settings.haModel));
      saveSettings();
      syslog->printf("HA Model set to: %s\n", (strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel : "(unset, uses mqttTopic)");
      if (liveData->settings.mqttHomeAssistant == 1 && liveData->settings.mqttEnabled == 1)
      {
        publishHomeAssistantDiscovery();
      }
    }
    else
    {
      syslog->printf("HA Model: %s\n", (strlen(liveData->settings.haModel) > 0) ? liveData->settings.haModel : "(unset, uses mqttTopic)");
    }
    return true;
  }
  if (key.equalsIgnoreCase("mqttInterval"))
  {
    if (isSetter)
    {
      long interval = value.toInt();
      if (interval < 0) interval = 0;
      if (interval > 3600) interval = 3600;
      liveData->settings.remoteUploadIntervalSec = static_cast<uint16_t>(interval);
      saveSettings();
      syslog->print("MQTT upload interval set to: ");
      if (liveData->settings.remoteUploadIntervalSec == 0)
        syslog->println("0 (auto 60s)");
      else
        syslog->printf("%u sec\n", liveData->settings.remoteUploadIntervalSec);
    }
    else
    {
      syslog->print("MQTT upload interval: ");
      if (liveData->settings.remoteUploadIntervalSec == 0)
        syslog->println("0 (auto 60s)");
      else
        syslog->printf("%u sec\n", liveData->settings.remoteUploadIntervalSec);
    }
    return true;
  }

  // Debug level
  if (key.equalsIgnoreCase("debugLevel"))
  {
    if (isSetter)
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
    }
    else
    {
      syslog->printf("Debug level bitmask: %u (comm=%s, net=%s, sd=%s, gps=%s, abrp=%s)\n",
                     liveData->settings.debugLevel,
                     (liveData->settings.debugLevel & DEBUG_COMM) ? "on" : "off",
                     (liveData->settings.debugLevel & DEBUG_NET) ? "on" : "off",
                     (liveData->settings.debugLevel & DEBUG_SDCARD) ? "on" : "off",
                     (liveData->settings.debugLevel & DEBUG_GPS) ? "on" : "off",
                     (liveData->settings.debugLevel & DEBUG_ABRP) ? "on" : "off");
    }
    return true;
  }

  // Legacy OBD2 WiFi
  if (key.equalsIgnoreCase("obd2WifiIp") || key.equalsIgnoreCase("obd2ip"))
  {
    if (isSetter)
    {
      value.toCharArray(liveData->settings.obd2WifiIp, sizeof(liveData->settings.obd2WifiIp));
      saveSettings();
      syslog->printf("obd2WifiIp set to: %s\n", liveData->settings.obd2WifiIp);
    }
    else
    {
      syslog->printf("obd2WifiIp: %s\n", liveData->settings.obd2WifiIp);
    }
    return true;
  }
  if (key.equalsIgnoreCase("obd2WifiPort") || key.equalsIgnoreCase("obd2port"))
  {
    if (isSetter)
    {
      liveData->settings.obd2WifiPort = static_cast<uint16_t>(value.toInt());
      saveSettings();
      syslog->printf("obd2WifiPort set to: %u\n", liveData->settings.obd2WifiPort);
    }
    else
    {
      syslog->printf("obd2WifiPort: %u\n", liveData->settings.obd2WifiPort);
    }
    return true;
  }

  // Time setter
  if (key.equalsIgnoreCase("setTime"))
  {
    if (isSetter)
      setTime(value);
    else
      showTime();
    return true;
  }

  // CAN comparer & record
  if (key.equalsIgnoreCase("record"))
  {
    if (isSetter && commInterface != nullptr)
      commInterface->recordLoop(value.toInt());
    return true;
  }
  if (key.equalsIgnoreCase("test"))
  {
    if (isSetter && carInterface != nullptr)
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
  syslog->println("System:");
  syslog->println("  help / ?              ... show this help (alias: commands)");
  syslog->println("  reboot / shutdown     ... reboot / deep sleep");
  syslog->println("  saveSettings          ... save settings to EEPROM");
  syslog->println("  factoryReset          ... reset settings to defaults");
  syslog->println("  time                  ... print current time");
  syslog->println("  setTime=YYYY-MM-DD HH:MM:SS ... set clock");
  syslog->println("  ntpSync               ... sync time via NTP");
  syslog->println("  ipconfig              ... print network status");
  syslog->println("  debugLevel[=n|comm,net,sd,gps,abrp,all,none] ... get/set debug mask");
  syslog->println("WiFi:");
  syslog->println("  wifiEnabled[=0|1]     ... get/set WiFi enabled");
  syslog->println("  wifiScan              ... scan and list WiFi APs");
  syslog->println("  wifiConnect           ... trigger WiFi connection now");
  syslog->println("  wifiSsid[=x]          ... get/set primary WiFi SSID");
  syslog->println("  wifiPassword[=x]      ... get/set primary WiFi password");
  syslog->println("  backupWifi[=0|1]      ... get/set backup WiFi enabled");
  syslog->println("  wifiSsid2..4[=x]      ... get/set backup WiFi SSID (2..4)");
  syslog->println("  wifiPassword2..4[=x]  ... get/set backup WiFi password (2..4)");
  syslog->println("  apPassword[=x]        ... get/set web log server AP password");
  syslog->println("  ntpEnabled[=0|1]      ... get/set NTP auto-sync");
  syslog->println("Vehicle & Adapter:");
  syslog->println("  cars                  ... list available car models & IDs");
  syslog->println("  carType[=id|name]     ... get/set car type (reboot required)");
  syslog->println("  commType[=0|1|ble|can]... get/set comm type (0=BLE, 1=CAN)");
  syslog->println("  obdMac[=mac]          ... get/set OBD2 BLE MAC address");
  syslog->println("  obd2Name[=name]       ... get/set OBD2 device display name");
  syslog->println("  serviceUUID[=uuid]    ... get/set BLE service UUID");
  syslog->println("  charTxUUID[=uuid]     ... get/set BLE Tx characteristic UUID");
  syslog->println("  charRxUUID[=uuid]     ... get/set BLE Rx characteristic UUID");
  syslog->println("  bleAddressType[=pub|rand] ... get/set BLE MAC address type");
  syslog->println("  commandQueueAutoStop[=0|1] ... get/set CAN queue autostop");
  syslog->println("  disableCommandOptimizer[=0|1] ... get/set command optimizer");
  syslog->println("  mobileRelay[=0|1]     ... get/set mobile app BLE relay");
  syslog->println("  mobileRelayPair       ... start mobile app pairing");
  syslog->println("  mobileRelayForget     ... clear mobile app pairing");
  syslog->println("  clearStats            ... clear driving & charging stats");
  syslog->println("  loadTestData          ... load demo telemetry data");
  syslog->println("Board & Settings:");
  syslog->println("  boardPowerMode[=0|1]  ... get/set power mode (0=USB, 1=ext)");
  syslog->println("  timezone[=n]          ... get/set timezone offset (-11..+14)");
  syslog->println("  daylightSaving[=0|1]  ... get/set daylight saving time");
  syslog->println("  distanceUnit[=k|m]    ... get/set unit: km or miles");
  syslog->println("  temperatureUnit[=c|f] ... get/set unit: C or F");
  syslog->println("  pressureUnit[=b|p]    ... get/set unit: bar or psi");
  syslog->println("  defaultScreen[=n]     ... get/set default screen (1..6, 8)");
  syslog->println("  displayRotation[=n]   ... get/set screen rotation (1 or 3)");
  syslog->println("  lcdBrightness[=0..100]... get/set brightness (0=auto)");
  syslog->println("  sleepMode[=0..2]      ... get/set sleep level (0=off,1=screen,2=deep)");
  syslog->println("  serialConsolePort[=n] ... get/set serial console port (0, 255=off)");
  syslog->println("  speedCorrection[=n]   ... get/set speed correction (-5..+5)");
  syslog->println("  rightHandDrive[=0|1]  ... get/set right hand drive (RHD)");
  syslog->println("Hardware modules:");
  syslog->println("  sdcardEnabled[=0|1]   ... get/set SD card logging");
  syslog->println("  sdcardConsoleLog[=0|1]... get/set console log to SD");
  syslog->println("  sdcardAutstartLog[=0|1] ... get/set SD autostart log");
  syslog->println("  sdcardStatus          ... check SD card mount status");
  syslog->println("  gpsModuleType[=0..3]  ... get/set GPS (0=none,1=M8N,2=GNSS,3=v2.1)");
  syslog->println("  gpsPort[=0|2|255]     ... get/set GPS hardware serial port");
  syslog->println("  gpsSpeed[=baud]       ... get/set GPS baud rate");
  syslog->println("  carSpeedType[=0..2]   ... get/set car speed (0=auto,1=car,2=gps)");
  syslog->println("  voltmeterEnabled[=0|1]... get/set INA3221 voltmeter");
  syslog->println("  voltmeterSleep[=0|1]  ... get/set voltmeter-based sleep");
  syslog->println("  voltmeterSleepVol[=v] ... get/set voltmeter sleep voltage threshold");
  syslog->println("  voltmeterWakeUpVol[=v]... get/set voltmeter wake-up voltage threshold");
  syslog->println("  voltmeterCutOffVol[=v]... get/set voltmeter cut-off voltage threshold");
  syslog->println("  voltmeterInfo         ... print current voltmeter readings");
  syslog->println("Remote upload & MQTT:");
  syslog->println("  remoteUploadIntervalSec[=s] ... get/set upload interval (0=disabled)");
  syslog->println("  remoteUploadAbrpIntervalSec[=s] ... get/set ABRP interval");
  syslog->println("  remoteApiUrl[=url]    ... get/set remote server URL");
  syslog->println("  remoteApiKey[=key]    ... get/set remote server API key");
  syslog->println("  abrpApiToken[=token]  ... get/set ABRP telemetry token");
  syslog->println("  abrpSdcardLog[=0|1]   ... get/set log ABRP to SD");
  syslog->println("  contributeData[=0|1]  ... get/set contribute to evdash.eu");
  syslog->println("  contributeOnce        ... send contribute snapshot now");
  syslog->println("  traccarEnabled[=0|1]  ... get/set Traccar client");
  syslog->println("  traccarServer[=host]  ... get/set Traccar server host");
  syslog->println("  traccarPort[=port]    ... get/set Traccar server port");
  syslog->println("  mqtt                  ... show all MQTT settings");
  syslog->println("  testMqtt              ... test MQTT connection & publish");
  syslog->println("  mqttEnabled[=0|1]     ... get/set MQTT upload");
  syslog->println("  mqttSecure[=0|1]      ... get/set MQTT TLS/SSL encryption");
  syslog->println("  mqttServer[=host]     ... get/set MQTT server host");
  syslog->println("  mqttPort[=port]       ... get/set MQTT port (0=auto 1883/8883)");
  syslog->println("  mqttId[=id]           ... get/set MQTT client ID");
  syslog->println("  mqttUsername[=user]   ... get/set MQTT username");
  syslog->println("  mqttPassword[=pwd]    ... get/set MQTT password");
  syslog->println("  mqttPubTopic[=topic]  ... get/set MQTT publish topic");
  syslog->println("  mqttHa[=0|1]          ... get/set Home Assistant MQTT discovery");
  syslog->println("  haName[=name]         ... get/set Home Assistant device name");
  syslog->println("  haModel[=model]       ... get/set Home Assistant device model");
  syslog->println("  mqttInterval[=sec]    ... get/set MQTT upload interval");
  syslog->println("Diagnostics & CAN:");
  syslog->println("  record=1..4           ... record CAN response buffer");
  syslog->println("  compare               ... compare CAN buffers");
  syslog->println("  test=x                ... run car test handler");
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
