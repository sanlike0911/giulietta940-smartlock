/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
   Changed to a beacon scanner to report iBeacon, EddystoneURL and EddystoneTLM beacons by beegee-tokyo
*/

#include <Arduino.h>
#include <Ticker.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
/*
  #include <BLEEddystoneURL.h>
  #include <BLEEddystoneTLM.h>
*/
#include <BLEBeacon.h>
#include "myBeacon.h"

#define _DEBUG_PRINT

//#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0xFF) << 8))

#define SCAN_TIME           (int)5  /* In seconds */

#define PORT_OUT_ON_BORAD_LED     2
#define PORT_IN_LOCK_LED          18
#define PORT_IN_ACC               19
#define PORT_OUT_DOOR_LOCK_SIGNAL 23

#define TICKER_LOOP_CYCLE         200

// LOCK LED入力：4回平均
#define AVERAGE_INPUT_GPIO        3

// Queue item size
#define QUEUE_LENGTH              16

// Event message
char *dbgEventMsg[] = {
  "EVENT_NON",
  "EVENT_LOST_BEACON",
  "EVENT_FOUND_BEACON",
  "EVENT_LOCK_LED_OFF",
  "EVENT_LOCK_LED_ON",
  "EVENT_ACC_OFF",
  "EVENT_ACC_ON",
  "EVENT_DOOR_LOCK_SIGNAL",
  "EVENT_DOOR_UNLOCK_SIGNAL",
  NULL
};

// Event ID
enum EventID {
  EVENT_NON = (int32_t)0,
  EVENT_LOST_BEACON,
  EVENT_FOUND_BEACON,
  EVENT_LOCK_LED_OFF,
  EVENT_LOCK_LED_ON,
  EVENT_ACC_OFF,
  EVENT_ACC_ON,
  EVENT_DOOR_LOCK_SIGNAL,
  EVENT_DOOR_UNLOCK_SIGNAL,
  EVENT_MAX
};


//*****************************************************************************
// macro function
//*****************************************************************************
#ifdef _DEBUG_PRINT
 #define SERIAL_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
 #define SERIAL_PRINTF(...)
#endif

//*****************************************************************************
// prototypes
//*****************************************************************************
void bleScan();
void dispLed();
void EventAnalyze(EventID _emEventID);
void checkLockLed();
void checkAcc();
void setDoorLockSignal();
void inGPIO();

void task1( void *param );
void task2( void *param );

bool setEventQueue( EventID _emEventID );
EventID getEventQueue();

void interruptTimer();

//*****************************************************************************
// variables
//*****************************************************************************
// Timer
volatile uint32_t tm1000msCount = 0;
volatile uint32_t tm10000msCount = 0;

// EVENT
volatile uint8_t tm1000msEvent = 0;
volatile uint8_t tm10000msEvent = 0;

// BLE
BLEScan *pBLEScan;
volatile uint8_t _iBeaconScanStatus = 0;

// Ticker
Ticker ticker;

// Queue
QueueHandle_t  hQueue;

//*****************************************************************************
// class
//*****************************************************************************
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // アドバタイジングデータを受け取ったとき
      myBeaconAdvertisedDevice _mybcn;
      if (_mybcn.createByAdvertisedDevice(advertisedDevice)) {
        //Serial.println("Found an iBeacon!");
        if (_mybcn.IsAdvertisedDevice("cb3b0426-10ec-45bd-b58e-f2858c0dbc2b")) {
          _iBeaconScanStatus = 1;
          BLEDevice::getScan()->stop();
#if 0
          char uuid[37];
          _mybcn.getUUID().toCharArray(uuid, 37);
          SERIAL_PRINTF("UUID: %s, Major: %d, Minor: %d, RSSI: %d \n", uuid, _mybcn.getMajor(), _mybcn.getMinor(), _mybcn.getRSSI());
#endif
        } else {
          //Serial.println("Found another ibeacon!");
        }
      }
      /*
            if (advertisedDevice.haveName()){
              Serial.print("Device name: ");
              Serial.println(advertisedDevice.getName().c_str());
              Serial.println("");
            }

            if (advertisedDevice.haveServiceUUID()){
              BLEUUID devUUID = advertisedDevice.getServiceUUID();
              Serial.print("Found ServiceUUID: ");
              Serial.println(devUUID.toString().c_str());
              Serial.println("");
            } else {
              if (advertisedDevice.haveManufacturerData() == true) {
                std::string strManufacturerData = advertisedDevice.getManufacturerData();

                uint8_t cManufacturerData[100];
                strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);

                if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C && cManufacturerData[1] == 0x00)
                {
                  Serial.println("Found an iBeacon!");
                  BLEBeacon oBeacon = BLEBeacon();
                  oBeacon.setData(strManufacturerData);

                  char cUUID[100];
                  oBeacon.getProximityUUID().toString().copy(cUUID, oBeacon.getProximityUUID().toString().length());

                  SERIAL_PRINTF("iBeacon Frame\n");
                  SERIAL_PRINTF("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n", oBeacon.getManufacturerId(), ENDIAN_CHANGE_U16(oBeacon.getMajor()), ENDIAN_CHANGE_U16(oBeacon.getMinor()), oBeacon.getProximityUUID().toString().c_str(), oBeacon.getSignalPower());

                  eventID |= EVENT_FOUND_BEACON;
                }
                else
                {
                  Serial.println("Found another manufacturers beacon!");
                  SERIAL_PRINTF("strManufacturerData: %d ", strManufacturerData.length());
                  for (int i = 0; i < strManufacturerData.length(); i++)
                  {
                    SERIAL_PRINTF("[%X]", cManufacturerData[i]);
                  }
                  SERIAL_PRINTF("\n");
                }
              }
              return;
            }
      */
      /*
            uint8_t *payLoad = advertisedDevice.getPayload();

            BLEUUID checkUrlUUID = (uint16_t)0xfeaa;

            if (advertisedDevice.getServiceUUID().equals(checkUrlUUID))
            {
              if (payLoad[11] == 0x10)
              {
                Serial.println("Found an EddystoneURL beacon!");
                BLEEddystoneURL foundEddyURL = BLEEddystoneURL();
                std::string eddyContent((char *)&payLoad[11]); // incomplete EddystoneURL struct!

                foundEddyURL.setData(eddyContent);
                std::string bareURL = foundEddyURL.getURL();
                if (bareURL[0] == 0x00)
                {
                  size_t payLoadLen = advertisedDevice.getPayloadLength();
                  Serial.println("DATA-->");
                  for (int idx = 0; idx < payLoadLen; idx++)
                  {
                    SERIAL_PRINTF("0x%08X ", payLoad[idx]);
                  }
                  Serial.println("\nInvalid Data");
                  return;
                }

                SERIAL_PRINTF("Found URL: %s\n", foundEddyURL.getURL().c_str());
                SERIAL_PRINTF("Decoded URL: %s\n", foundEddyURL.getDecodedURL().c_str());
                SERIAL_PRINTF("TX power %d\n", foundEddyURL.getPower());
                Serial.println("\n");
              }
              else if (payLoad[11] == 0x20)
              {
                Serial.println("Found an EddystoneTLM beacon!");
                BLEEddystoneTLM foundEddyURL = BLEEddystoneTLM();
                std::string eddyContent((char *)&payLoad[11]); // incomplete EddystoneURL struct!

                eddyContent = "01234567890123";

                for (int idx = 0; idx < 14; idx++)
                {
                  eddyContent[idx] = payLoad[idx + 11];
                }

                foundEddyURL.setData(eddyContent);
                SERIAL_PRINTF("Reported battery voltage: %dmV\n", foundEddyURL.getVolt());
                SERIAL_PRINTF("Reported temperature from TLM class: %.2fC\n", (double)foundEddyURL.getTemp());
                int temp = (int)payLoad[16] + (int)(payLoad[15] << 8);
                float calcTemp = temp / 256.0f;
                SERIAL_PRINTF("Reported temperature from data: %.2fC\n", calcTemp);
                SERIAL_PRINTF("Reported advertise count: %d\n", foundEddyURL.getCount());
                SERIAL_PRINTF("Reported time since last reboot: %ds\n", foundEddyURL.getTime());
                Serial.println("\n");
                Serial.print(foundEddyURL.toString().c_str());
                Serial.println("\n");
              }
            }
      */
    }
};

//*****************************************************************************
// functions
//*****************************************************************************
void setup() {
#if 1
  /* create gpio */
  pinMode(PORT_OUT_ON_BORAD_LED, OUTPUT);
  digitalWrite(PORT_OUT_ON_BORAD_LED, HIGH);

  pinMode(PORT_OUT_DOOR_LOCK_SIGNAL, OUTPUT);
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);

  pinMode(PORT_IN_LOCK_LED, INPUT);
  pinMode(PORT_IN_LOCK_LED, INPUT_PULLUP);
  pinMode(PORT_IN_ACC, INPUT);
  pinMode(PORT_IN_ACC, INPUT_PULLUP);
#endif

#if 1
  /* create serial */
  Serial.begin(115200);
  SERIAL_PRINTF("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  SERIAL_PRINTF("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  SERIAL_PRINTF("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  SERIAL_PRINTF("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
#endif

#if 1
  /* create ticker */
  ticker.attach_ms(TICKER_LOOP_CYCLE, interruptTimer);
#endif

#if 1
  /* create event-group */
  hQueue = xQueueCreate( QUEUE_LENGTH, sizeof(int32_t));
#endif

#if 1
  /* create task */
  xTaskCreatePinnedToCore( task1,   /* タスクの入口となる関数名 */
                           "TASK1", /* タスクの名称 */
                           0x800,   /* スタックサイズ */
                           NULL,    /* パラメータのポインタ */
                           1,       /* プライオリティ */
                           NULL,    /* ハンドル構造体のポインタ */
                           0 );     /* 割り当てるコア (0/1) */

  xTaskCreatePinnedToCore( task2,
                           "TASK2",
                           0x800,
                           NULL,
                           1,
                           NULL,
                           1 );
#endif

#if 1
  /* create ble */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  //pBLEScan->setInterval(1000);
  //pBLEScan->setWindow(99); // less or equal setInterval value
#endif

  /*
    Serial.begin(115200);
    gettimeofday(&now, NULL);

    SERIAL_PRINTF("start ESP32 %d\n",bootcount++);

    SERIAL_PRINTF("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);

    last = now.tv_sec;

    // Create the BLE Device
    BLEDevice::init("ESP32-BLEBacon-SERVER");

    // Create the BLE Server
    // BLEServer *pServer = BLEDevice::createServer(); // <-- no longer required to instantiate BLEServer, less flash and ram usage

    pAdvertising = BLEDevice::getAdvertising();

    setBeacon();

    // Start advertising
    pAdvertising->start();
    Serial.println("Advertizing started...");
    delay(100);
    pAdvertising->stop();
    SERIAL_PRINTF("enter deep sleep\n");
    esp_deep_sleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);
    SERIAL_PRINTF("in deep sleep\n");
  */
}

bool setEventQueue( EventID _emEventID ) {
  bool bRes = true;
  int32_t _tempEventID = (int32_t)_emEventID;
  BaseType_t xStatus = xQueueSend( hQueue, &_tempEventID, 0 );
  if ( xStatus != pdPASS ) {
    SERIAL_PRINTF("xQueueSend() fail. %d\n", xStatus);
    bRes = false;
  }
  return bRes;
}

EventID getEventQueue() {
  int32_t ReceiveValue = 0;
  BaseType_t xStatus = xQueueReceive( hQueue, &ReceiveValue, portMAX_DELAY );
  if ( xStatus != pdPASS ) {
    SERIAL_PRINTF("xQueueReceive() fail. %d\n", xStatus);
    ReceiveValue = (int32_t)EVENT_NON;
  }
  return (EventID)ReceiveValue;
}

void inGPIO() {
  // IN:GPIO
  checkLockLed();
  checkAcc();
}

void interruptTimer() {

  inGPIO();

  if ( ++tm1000msCount >= (uint32_t)(1000 / TICKER_LOOP_CYCLE) ) {
    tm1000msCount = 0; tm1000msEvent = 1;
  }
  if ( ++tm10000msCount >= (uint32_t)(10000 / TICKER_LOOP_CYCLE) ) {
    tm10000msCount = 0; tm10000msEvent = 1;
  }
}

void setDoorLockSignal() {
#if 0
  if (subEventID & SUB_EVENT_SET_DOOR_LOCK_SIGNAL) {
    subEventID &= ~SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
  }
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
  delay(500);
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
#endif
}

void EventAnalyze(EventID _emEventID) {
  switch(_emEventID){
  case EVENT_LOST_BEACON:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_LOST_BEACON\n");
    break;
  case EVENT_FOUND_BEACON:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_FOUND_BEACON\n");
    break;
  case EVENT_LOCK_LED_OFF:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_LOCK_LED_OFF\n");
    break;
  case EVENT_LOCK_LED_ON:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_LOCK_LED_ON\n");
    break;
  case EVENT_ACC_OFF:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_ACC_OFF\n");
    break;
  case EVENT_ACC_ON:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_ACC_ON\n");
    break;
  case EVENT_DOOR_LOCK_SIGNAL:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_DOOR_LOCK_SIGNAL\n");
    break;
  case EVENT_DOOR_UNLOCK_SIGNAL:
    //SERIAL_PRINTF("EventAnalyze(): EVENT_DOOR_UNLOCK_SIGNAL\n");
    break;
  default:
    SERIAL_PRINTF("EventAnalyze(): EVENT_NON\n");
    break;
  }

#if 0
  while (eventID) {
    SERIAL_PRINTF("EventAnalyze(): ID:%x\n", eventID);
    // EVENT優先度順
    if (eventID & EVENT_ACC_OFF) {
      eventID &= ~EVENT_ACC_OFF;
      SERIAL_PRINTF("EventAnalyze(): EVENT_ACC_OFF\n");
    } else if (eventID & EVENT_ACC_ON) {
      eventID &= ~EVENT_ACC_ON;
      SERIAL_PRINTF("EventAnalyze(): EVENT_ACC_ON\n");
    } else if (eventID & EVENT_LOCK_LED_OFF) {
      eventID &= ~EVENT_LOCK_LED_OFF;
      SERIAL_PRINTF("EventAnalyze(): EVENT_LOCK_LED_OFF\n");
    } else if (eventID & EVENT_LOCK_LED_ON) {
      eventID &= ~EVENT_LOCK_LED_ON;
      SERIAL_PRINTF("EventAnalyze(): EVENT_LOCK_LED_ON\n");
    } else if (eventID & EVENT_LOST_BEACON) {
      eventID &= ~EVENT_LOST_BEACON;
      SERIAL_PRINTF("EventAnalyze(): EVENT_LOST_BEACON\n");
    } else if (eventID & EVENT_FOUND_BEACON) {
      eventID &= ~EVENT_FOUND_BEACON;
      SERIAL_PRINTF("EventAnalyze(): EVENT_FOUND_BEACON\n");
    } else if (eventID & EVENT_DOOR_LOCK_SIGNAL) {
      eventID &= ~EVENT_DOOR_LOCK_SIGNAL;
      SERIAL_PRINTF("EventAnalyze(): EVENT_DOOR_LOCK_SIGNAL\n");
    } else if (eventID & EVENT_DOOR_UNLOCK_SIGNAL) {
      eventID &= ~EVENT_DOOR_UNLOCK_SIGNAL;
      SERIAL_PRINTF("EventAnalyze(): EVENT_DOOR_UNLOCK_SIGNAL\n");
    } else {
      SERIAL_PRINTF("EventAnalyze(): ERROR EVENT ID\n");
      eventID = EVENT_INIT;
    }
  }
#endif
}

void dispLed() {
#if 1
  //SERIAL_PRINTF("dispLed() _iBeaconScanStatus: %d\n",_iBeaconScanStatus);
  digitalWrite(PORT_OUT_ON_BORAD_LED, (_iBeaconScanStatus == 1 ? HIGH : LOW));
#else
  // blink
  static int ledBlink = HIGH;
  digitalWrite(PORT_OUT_ON_BORAD_LED, ledBlink);
  ledBlink = (ledBlink == HIGH ? LOW : HIGH);
#endif
}

void bleScan() {
  static byte __iBeaconScanStatusOLD = 0;

  _iBeaconScanStatus = 0;
  //BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME/*, false*/);
  pBLEScan->start(SCAN_TIME, false);
  SERIAL_PRINTF("Scan done!");
  //pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

  if (_iBeaconScanStatus) {
    // Edge:ON
    if (0 == __iBeaconScanStatusOLD) {
      setEventQueue(EVENT_FOUND_BEACON);
    }
  } else {
    // Edge:OFF
    if (1 == __iBeaconScanStatusOLD) {
      setEventQueue(EVENT_LOST_BEACON);
    }
  }
  __iBeaconScanStatusOLD = _iBeaconScanStatus;
}

void checkLockLed() {
  static byte _activeCount = 0;
  static byte _inGpioStatusFix = 0xFF;
  byte _inGpioStatus = digitalRead(PORT_IN_LOCK_LED);
  if (HIGH == _inGpioStatus) {
    _activeCount = 0;
    if (HIGH != _inGpioStatusFix) {
      setEventQueue(EVENT_LOCK_LED_OFF);
      _inGpioStatusFix = _inGpioStatus;
    }
  } else {
    if (++_activeCount >= AVERAGE_INPUT_GPIO) {
      _activeCount = AVERAGE_INPUT_GPIO;
      if (LOW != _inGpioStatusFix) {
        setEventQueue(EVENT_LOCK_LED_ON);
        _inGpioStatusFix = _inGpioStatus;
      }
    }
  }
}

void checkAcc() {
  static byte _activeCount = 0;
  static byte _inGpioStatusFix = 0xFF;
  byte _inGpioStatus = digitalRead(PORT_IN_ACC);
  if (HIGH == _inGpioStatus) {
    _activeCount = 0;
    if (HIGH != _inGpioStatusFix) {
      setEventQueue(EVENT_ACC_OFF);
      _inGpioStatusFix = _inGpioStatus;
    }
  } else {
    if (++_activeCount >= AVERAGE_INPUT_GPIO) {
      _activeCount = AVERAGE_INPUT_GPIO;
      if (LOW != _inGpioStatusFix) {
        setEventQueue(EVENT_ACC_ON);
        _inGpioStatusFix = _inGpioStatus;
      }
    }
  }
}

void loop() {
  bleScan();
  vTaskDelay(1000);
}

void task1( void *param )
{
  SERIAL_PRINTF( "task1() : start\n" );
  EventID _emEventID;
  while ( 1 ) {
    _emEventID = getEventQueue();
    SERIAL_PRINTF( "task1(): id=%d msg=%s\n", _emEventID, dbgEventMsg[_emEventID] );
    EventAnalyze(_emEventID);
    //vTaskDelay(1000);
  }
}

void task2( void *param )
{
  SERIAL_PRINTF( "task2() : start\n");
  while ( 1 ) {
    // 1sec
    if ( tm1000msEvent ) {
      tm1000msEvent = 0;
      dispLed();
    }
    // 10sec
    if ( tm10000msEvent ) {
      tm10000msEvent = 0;
    }
  }
}
