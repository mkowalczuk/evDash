/**
 * Includes header files for different boards, cars, livedata,
 * and various components used by the evDash application.
 * Sets up the board, car interface, livedata, and logging.
 */

/*
  evDash (older name eNiroDashboard)

  Serial console commands
   serviceUUID=xxx
   charTxUUID=xxx
   charRxUUID=xxx
   wifiSsid=xxx
   wifiPassword=xxx
   wifiSsid2=xxx
   wifiPassword2=xxx
   remoteApiUrl=xxx
   remoteApiKey=xxx
   abrpApiToken=xxx

  Required libraries, see INSTALLATION.rd
*/

#define USE_M5_FONT_CREATOR

#include "Arduino.h"
#include <esp_heap_caps.h>
#include <esp_bt.h>
#include <mbedtls/platform.h>
#include <new>
#include <stdint.h>
#include <stdlib.h>

#include "config.h"
#include "BoardInterface.h"

#ifdef BOARD_M5STACK_CORE2
#include "BoardM5stackCore2.h"
#endif // BOARD_M5STACK_CORE2

#ifdef BOARD_M5STACK_CORES3
#include "BoardM5stackCoreS3.h"
#endif // BOARD_M5STACK_CORES3

#ifdef BOARD_WAVESHARE_SIM7670G
#include "BoardWaveshareSim7670g.h"
#endif // BOARD_WAVESHARE_SIM7670G

#include "LogSerial.h"
#include "LiveData.h"
#include "CarInterface.h"
#include "CarKiaEniro.h"
#include "CarHyundaiIoniq.h"
#include "CarHyundaiIoniqPHEV.h"
#include "CarHyundaiEgmp.h"
#include "CarKiaEV9.h"
#include "CarRenaultZoe.h"
#include "CarBmwI3.h"
#include "CarVWID3.h"
#include "CarVWUpMii.h"
#include "CarPeugeotE208.h"
#include "CarXpeng.h"
#include "EvDashMobileRelay.h"

// Board, Car, Livedata (params, settings)
BoardInterface *board;
CarInterface *car;
LiveData *liveData;
EvDashMobileRelay *mobileRelay;

/**
 * Prefer PSRAM for larger mbedTLS allocations.
 */
static void *evdashTlsCalloc(size_t count, size_t size)
{
  if (count != 0 && size > SIZE_MAX / count)
  {
    return nullptr;
  }
  const size_t bytes = count * size;
  void *ptr = nullptr;
  if (bytes >= 1024 && psramFound())
  {
    ptr = heap_caps_calloc(count, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (ptr == nullptr)
  {
    ptr = heap_caps_calloc(count, size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (ptr == nullptr)
  {
    ptr = calloc(count, size);
  }
  return ptr;
}

/**
 * Free mbedTLS memory allocated by evdashTlsCalloc.
 */
static void evdashTlsFree(void *ptr)
{
  free(ptr);
}

/**
 * Setup function that initializes the board, car interface, live data,
 * and logging. Also prints some introductory text.
 */
void setup(void)
{
  // Init settings/params
  bool liveDataAllocatedInPsram = false;
  void *liveDataMem = heap_caps_malloc(sizeof(LiveData), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (liveDataMem != nullptr)
  {
    liveData = new (liveDataMem) LiveData();
    liveDataAllocatedInPsram = true;
  }
  else
  {
    liveData = new LiveData();
  }
  liveData->initParams();

  // Serial console
  syslog = new LogSerial();
  syslog->begin(115200);
  if (psramFound())
  {
    int tlsAllocRc = mbedtls_platform_set_calloc_free(evdashTlsCalloc, evdashTlsFree);
    syslog->print("mbedTLS PSRAM allocator: ");
    syslog->println(tlsAllocRc == 0 ? "enabled" : "failed");
  }

#ifdef BOARD_M5STACK_CORE2
  board = new BoardM5stackCore2();
#endif // BOARD_M5STACK_CORE2

#ifdef BOARD_M5STACK_CORES3
  board = new BoardM5stackCoreS3();
#endif // BOARD_M5STACK_CORES3

#ifdef BOARD_WAVESHARE_SIM7670G
  board = new BoardWaveshareSim7670g();
#endif // BOARD_WAVESHARE_SIM7670G

  board->setLiveData(liveData);
  board->loadSettings();

#if CONFIG_BT_ENABLED
  bool keepBtController = (liveData->settings.commType == COMM_TYPE_OBD2_BLE4);
  keepBtController = keepBtController || (liveData->settings.relayForMobileEnabled == 1);
  if (!keepBtController)
  {
    esp_err_t btReleaseRc = esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
    if (btReleaseRc == ESP_OK)
    {
      syslog->println("BT controller memory released (non-BLE adapter mode)");
    }
    else if (btReleaseRc == ESP_ERR_INVALID_STATE)
    {
      // Already released or controller already started; keep booting.
      syslog->println("BT controller memory release skipped (already in use)");
    }
    else
    {
      syslog->print("BT controller memory release failed: ");
      syslog->println((int)btReleaseRc);
    }
  }
#endif

  board->initBoard();

  // Turn on serial console
  if (liveData->settings.serialConsolePort != 255 && liveData->settings.gpsHwSerialPort != liveData->settings.serialConsolePort)
  {
    syslog->begin(115200);
  }

  syslog->print("LiveData allocation: ");
  syslog->println(liveDataAllocatedInPsram ? "PSRAM" : "INTERNAL HEAP");
  syslog->print("Heap intFree/intLargest/psram: ");
  syslog->println(String(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) + " / " +
                  String(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) + " / " +
                  String(ESP.getFreePsram()));

  syslog->println("\nBooting device...");
  // board->resetSettings();

  // Init selected car interface
  switch (liveData->settings.carType)
  {
  case CAR_KIA_ENIRO_2020_39:
  case CAR_KIA_ENIRO_2020_64:
  case CAR_HYUNDAI_KONA_2020_39:
  case CAR_HYUNDAI_KONA_2020_64:
  case CAR_KIA_ESOUL_2020_64:
    car = new CarKiaEniro();
    break;
  case CAR_HYUNDAI_IONIQ5_58_63:
  case CAR_HYUNDAI_IONIQ5_72:
  case CAR_HYUNDAI_IONIQ5_77_84:
  case CAR_HYUNDAI_IONIQ6_53:
  case CAR_HYUNDAI_IONIQ6_58_63:
  case CAR_HYUNDAI_IONIQ6_77_84:
  case CAR_KIA_EV6_58_63:
  case CAR_KIA_EV6_77_84:
    car = new CarHyundaiEgmp();
    break;
  case CAR_KIA_EV9_100:
    car = new CarKiaEV9();
    break;
  case CAR_HYUNDAI_IONIQ_2018:
    car = new CarHyundaiIoniq();
    break;
  case CAR_HYUNDAI_IONIQ_PHEV:
    car = new CarHyundaiIoniqPHEV();
    break;
  case CAR_RENAULT_ZOE_ZE20_22:
  case CAR_RENAULT_ZOE_ZE40_41:
  case CAR_RENAULT_ZOE_ZE50_52:
    car = new CarRenaultZoe();
    break;
  case CAR_BMW_I3_2014:
    car = new CarBmwI3();
    break;
  case CAR_AUDI_Q4_35:
  case CAR_AUDI_Q4_40:
  case CAR_AUDI_Q4_45:
  case CAR_AUDI_Q4_50:
  case CAR_SKODA_ENYAQ_55:
  case CAR_SKODA_ENYAQ_62:
  case CAR_SKODA_ENYAQ_82:
  case CAR_VW_ID3_2021_45:
  case CAR_VW_ID3_2021_58:
  case CAR_VW_ID3_2021_77:
  case CAR_VW_ID4_2021_45:
  case CAR_VW_ID4_2021_58:
  case CAR_VW_ID4_2021_77:
    car = new CarVWID3();
    break;
  case CAR_SKODA_CITIGO_E_IV:
  case CAR_VW_EUP_36:
  case CAR_SEAT_MII_ELECTRIC_36:
    car = new CarVWUpMii();
    break;
  case CAR_PEUGEOT_E208:
    car = new CarPeugeotE208();
    break;
  case CAR_XPENG:
  case CAR_XPENG_G6_66:
  case CAR_XPENG_G6_88:
  case CAR_XPENG_G9_78:
  case CAR_XPENG_P7_60:
  case CAR_XPENG_P7_83:
  case CAR_XPENG_P7PLUS_75:
  case CAR_XPENG_P5_66:
  case CAR_XPENG_G3_66:
  case CAR_XPENG_X9_85:
  case CAR_XPENG_X9_101:
    car = new CarXpeng();
    break;
  default:
    car = new CarKiaEniro();
  }

  car->setLiveData(liveData);
  car->activateCommandQueue();
  board->attachCar(car);

  // Finish board setup
  board->afterSetup();
  mobileRelay = new EvDashMobileRelay();
  mobileRelay->begin(liveData, board);
  board->redrawScreen();

  // End
  syslog->println("Device setup completed");
  syslog->println("");
  syslog->println("▓█████ ██▒   █▓▓█████▄  ▄▄▄        ██████  ██░ ██ ");
  syslog->println("▓█   ▀▓██░   █▒▒██▀ ██▌▒████▄    ▒██    ▒ ▓██░ ██▒");
  syslog->println("▒███   ▓██  █▒░░██   █▌▒██  ▀█▄  ░ ▓██▄   ▒██▀▀██░");
  syslog->println("▒▓█  ▄  ▒██ █░░░▓█▄   ▌░██▄▄▄▄██   ▒   ██▒░▓█ ░██ ");
  syslog->println("░▒████▒  ▒▀█░  ░▒████▓  ▓█   ▓██▒▒██████▒▒░▓█▒░██▓");
  syslog->println("░░ ▒░ ░  ░ ▐░   ▒▒▓  ▒  ▒▒   ▓▒█░▒ ▒▓▒ ▒ ░ ▒ ░░▒░▒");
  syslog->println(" ░ ░  ░  ░ ░░   ░ ▒  ▒   ▒   ▒▒ ░░ ░▒  ░ ░ ▒ ░▒░ ░");
  syslog->println("   ░       ░░   ░ ░  ░   ░   ▒   ░  ░  ░   ░  ░░ ░");
  syslog->println("   ░  ░     ░     ░          ░  ░      ░   ░  ░  ░");
  syslog->println("           ░    ░                                 ");
  syslog->println("Type 'help' for console commands.");
  board->showHelp();
}

/**
 * Main program loop that calls the board's mainLoop() method.
 * This handles running the main program logic in a loop.
 */
void loop()
{
  if (mobileRelay != nullptr)
  {
    mobileRelay->loop();
  }
  board->mainLoop();
}
