#include <Arduino.h>
#include <Ticker.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEBeacon.h>
#include "myBeacon.h"

#define _DEBUG_PRINT

/* BLS settings */
#define SCAN_TIME                     (uint32_t)5  /* In seconds */

/* GPIO settings */
#define PORT_OUT_ON_BORAD_LED         2
#define PORT_IN_LOCK_LED              18
#define PORT_IN_ACC                   33
#define PORT_OUT_DOOR_LOCK_SIGNAL     23

#define PORT_IN_AVERAGE_NUM           (uint8_t)3  /* GPI平均回数:LOCK LED, ACC */

#define DOOR_LOOK_ACTIVE_COUNT        (uint8_t)2  /* active time n×400ms */

/* timer settings */
#define TICKER_MAIN_LOOP_INTERVAL     (uint32_t)200
#define TICKER_CHECK_DOOR_LAMP        (float)115.0

#define DOOR_LOCK_FUNC_INVALID_TIMER  (float)3.0  /* 起動直後のドアロック機能無効タイマ */

/* event aueue settings */
#define QUEUE_LENGTH                  8       /* event queue length */

/* touch sensor 移動平均(n) */
#define AVEREGE_TOUCH_SENSOR_NUM      (uint8_t)5
/* touch sensor ヒステリシスループ閾値 */
#define TOUCH_SENSOR_THRESHOLD_ACTIVE   (uint16_t)30
#define TOUCH_SENSOR_THRESHOLD_DEACTIVE (uint16_t)50

/* debug Event Message */
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
  "EVENT_ACTIVE_TOUCH_SENSOR",
  "EVENT_DOOR_LOCK_LAMP_TIMEOUT",
  "EVENT_MAX"
};

/* event id */
enum EventID {
  EVENT_NON = (uint8_t)0,
  EVENT_LOST_BEACON,
  EVENT_FOUND_BEACON,
  EVENT_LOCK_LED_OFF,
  EVENT_LOCK_LED_ON,
  EVENT_ACC_OFF,
  EVENT_ACC_ON,
  EVENT_DOOR_LOCK_SIGNAL,
  EVENT_DOOR_UNLOCK_SIGNAL,
  EVENT_ACTIVE_TOUCH_SENSOR,
  EVENT_DOOR_LOCK_LAMP_TIMEOUT,
  EVENT_MAX
};

/* sub event id */
#define SUB_EVENT_NON                     0x00
#define SUB_EVENT_SET_DOOR_LOCK_SIGNAL    0x01

/* debug beacon status message */
char *dbgBeaconStatusMsg[] = {
  "BEACON_STATUS_NON",
  "BEACON_STATUS_LOST",
  "BEACON_STATUS_FOUND",
  "BEACON_STATUS_MAX",
};

/* beacon status id */
enum BEACON_STATUS_ID {
  BEACON_STATUS_NON = (uint8_t)0,
  BEACON_STATUS_LOST,
  BEACON_STATUS_FOUND,
  BEACON_STATUS_MAX
};

/* debug door status message */
char *dbgDoorStatusMsg[] = {
  "DOOR_STATUS_NON",
  "DOOR_STATUS_UNLOCK_WAIT",
  "DOOR_STATUS_UNLOCK",
  "DOOR_STATUS_LOCK_WAIT",
  "DOOR_STATUS_LOCK",
  "DOOR_STATUS_MAX",
};

/* door status id */
enum DOOR_STATUS_ID {
  DOOR_STATUS_NON = (uint8_t)0,
  DOOR_STATUS_UNLOCK_WAIT,
  DOOR_STATUS_UNLOCK,
  DOOR_STATUS_LOCK_WAIT,
  DOOR_STATUS_LOCK,
  DOOR_STATUS_MAX
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
void EventControl(EventID _emEventID);
void getDoorLockLed();
void getAccStatus();
void getToucSensor();
void doorLockSignalControl();
void GpiControl();
void print_wakeup_reason();
void startdoorLockLampOffTimer();
void stopdoorLockLampOffTimer();

void task1( void *param );
void task2( void *param );

bool setEventQueue( EventID _emEventID );
bool getEventQueue( EventID* _emEventID );

void tickerMainLoopTimer();
void tickerDoorLockLampOffTimer();

//*****************************************************************************
// variables
//*****************************************************************************
// Timer
volatile uint32_t tm400msCount = 0;
volatile uint32_t tm1000msCount = 0;
volatile uint32_t tm10000msCount = 0;
volatile uint8_t  tm400msEvent = 0;
volatile uint8_t  tm1000msEvent = 0;
volatile uint8_t  tm10000msEvent = 0;

// Sub Event
volatile uint8_t subEventID = SUB_EVENT_NON;

// Status
volatile BEACON_STATUS_ID beaconStatusID = BEACON_STATUS_NON;
volatile DOOR_STATUS_ID doorStatusID = DOOR_STATUS_NON;

// BLE
BLEScan *pBLEScan;
volatile uint8_t doConnectBleDevice = 0;

// Ticker
Ticker tickerMainLoop;
Ticker doorLockLampOffTimer;

// Queue
volatile QueueHandle_t  hQueue;

//*****************************************************************************
// class
//*****************************************************************************
/**
 * @brief BLEAdvertisedDeviceCallbacks
 * @author sanlike
 * @date 2021/03/02
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // アドバタイジングデータを受け取ったとき
      myBeaconAdvertisedDevice _mybcn;
      if (_mybcn.createByAdvertisedDevice(advertisedDevice)) {
        //Serial.println("Found an iBeacon!");
        if (_mybcn.IsAdvertisedDevice("cb3b0426-10ec-45bd-b58e-f2858c0dbc2b")) {
          doConnectBleDevice = 1;
          BLEDevice::getScan()->stop();
#if 1
          char uuid[37];
          _mybcn.getUUID().toCharArray(uuid, 37);
          SERIAL_PRINTF("UUID: %s, Major: %d, Minor: %d, RSSI: %d \n", uuid, _mybcn.getMajor(), _mybcn.getMinor(), _mybcn.getRSSI());
#endif
        } else {
          //SERIAL_PRINTF("Found another ibeacon!\n");
        }
      } else {
        //SERIAL_PRINTF("...\n");
      }
    }
};

//*****************************************************************************
// functions
//*****************************************************************************
/**
 * @brief 初期設定処理
 * @author sanlike
 * @date 2021/03/02
 */
void setup() {

  /* create gpio */
  pinMode(PORT_OUT_ON_BORAD_LED, OUTPUT);
  digitalWrite(PORT_OUT_ON_BORAD_LED, HIGH);

  pinMode(PORT_OUT_DOOR_LOCK_SIGNAL, OUTPUT);
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);

  pinMode(PORT_IN_LOCK_LED, INPUT);
  pinMode(PORT_IN_LOCK_LED, INPUT_PULLUP);

  pinMode(PORT_IN_ACC, INPUT);
  pinMode(PORT_IN_ACC, INPUT_PULLUP);

  /* create serial */
  Serial.begin(115200);
  SERIAL_PRINTF("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  SERIAL_PRINTF("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  SERIAL_PRINTF("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  SERIAL_PRINTF("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());

  /* setup sleep ext0 wakeup */
  print_wakeup_reason();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //1 = High, 0 = Low
  
  /* create ticker */
  tickerMainLoop.attach_ms(TICKER_MAIN_LOOP_INTERVAL, tickerMainLoopTimer);

  /* create event-group */
  hQueue = xQueueCreate( QUEUE_LENGTH, sizeof(uint8_t));

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

  /* create ble */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
}

/**
 * @brief print wakeup reason
 * @author sanlike
 * @date 2021/03/03
 */
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :      SERIAL_PRINTF("Wakeup caused by external signal using RTC_IO\n"); break;
    case ESP_SLEEP_WAKEUP_EXT1 :      SERIAL_PRINTF("Wakeup caused by external signal using RTC_CNTL\n"); break;
    case ESP_SLEEP_WAKEUP_TIMER :     SERIAL_PRINTF("Wakeup caused by timer\n"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :  SERIAL_PRINTF("Wakeup caused by touchpad\n"); break;
    case ESP_SLEEP_WAKEUP_ULP :       SERIAL_PRINTF("Wakeup caused by ULP program\n"); break;
    default :                         SERIAL_PRINTF("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/**
 * @brief イベントキュー設定処理
 * @author sanlike
 * @date 2021/03/02
 */
bool setEventQueue( EventID _emEventID ) {
  bool bRes = true;
  BaseType_t xStatus = xQueueSend( hQueue, &_emEventID, 0 );
  if ( xStatus != pdPASS ) {
    SERIAL_PRINTF("xQueueSend() fail. %d\n", xStatus);
    bRes = false;
  }
  return bRes;
}

/**
 * @brief イベントキュー取得処理
 * @author sanlike
 * @date 2021/03/02
 */
bool getEventQueue( EventID* _emEventID ) {
  bool bRes = true;
  BaseType_t xStatus = xQueueReceive( hQueue, _emEventID, portMAX_DELAY );
  if ( xStatus != pdPASS ) {
    SERIAL_PRINTF("xQueueReceive() fail. %d\n", xStatus);
    bRes = false;
  }
  return bRes;
}

/**
 * @brief GPIO入力制御(ノイズ除去：n回一致)
 * @author sanlike
 * @date 2021/03/02
 */
void GpiControl() {
  /* 入力：車両側)Lock Button LED状態 */
  getDoorLockLed();
  /* 入力：車両側)ACC状態 */
  getAccStatus();
#if 1 // touchセンサ入力テスト
  /* 入力：touchセンサ */
  getToucSensor();
#endif // touchセンサ入力テスト  
}

/**
 * @brief ドアロックランプオフ監視タイマの開始
 * @author sanlike
 * @date 2021/03/08
 */
void startdoorLockLampOffTimer(){
  // タイマが起動していたら停止する
  stopdoorLockLampOffTimer();
  // タイマ開始
  doorLockLampOffTimer.once(TICKER_CHECK_DOOR_LAMP, tickerDoorLockLampOffTimer);
}

/**
 * @brief ドアロックランプオフ監視タイマの停止
 * @author sanlike
 * @date 2021/03/08
 */
void stopdoorLockLampOffTimer(){
  doorLockLampOffTimer.detach();
}

/**
 * @brief ドアロックランプオフ監視タイマ(ドアロックランプの2分後消灯機能を監視)
 * @author sanlike
 * @date 2021/03/06
 */
void tickerDoorLockLampOffTimer(){
  setEventQueue(EVENT_DOOR_LOCK_LAMP_TIMEOUT);
}

/**
 * @brief 割り込みタイマ(200ms)
 * @author sanlike
 * @date 2021/03/02
 */
void tickerMainLoopTimer() {
  /* GPI制御処理 */
  GpiControl();

  if ( ++tm400msCount >= (uint32_t)(400 / TICKER_MAIN_LOOP_INTERVAL) ) {
    tm400msCount = 0;
    tm400msEvent = 1;
  }
  if ( ++tm1000msCount >= (uint32_t)(1000 / TICKER_MAIN_LOOP_INTERVAL) ) {
    tm1000msCount = 0;
    tm1000msEvent = 1;
  }
  if ( ++tm10000msCount >= (uint32_t)(10000 / TICKER_MAIN_LOOP_INTERVAL) ) {
    tm10000msCount = 0;
    tm10000msEvent = 1;
  }
}

/**
 * @brief タッチセンサ入力処理
 * @author sanlike
 * @date 2021/03/12
 */
void getToucSensor(){
  uint16_t _value = touchRead(T6);
  uint16_t _averageTouchSensor = 0; 

  static uint8_t _index = 0;
  static uint16_t _touchSensor[AVEREGE_TOUCH_SENSOR_NUM];
  static uint32_t _touchSensorSum = 0;

  uint8_t _threshold = LOW;
  static uint8_t _thresholdOld = LOW;

  // 移動平均
  if (_index == AVEREGE_TOUCH_SENSOR_NUM) _index = 0;
  _touchSensorSum -= _touchSensor[_index];
  _touchSensor[_index] = _value;
  _touchSensorSum += _touchSensor[_index];
  _index++;
  _averageTouchSensor = _touchSensorSum / AVEREGE_TOUCH_SENSOR_NUM;

  // active
  if( _averageTouchSensor <= TOUCH_SENSOR_THRESHOLD_ACTIVE ){
    _threshold = HIGH;
  // deactive
  } else if( _averageTouchSensor >= TOUCH_SENSOR_THRESHOLD_DEACTIVE ){
    _threshold = LOW;
  }

  // edge:low->high
  if( HIGH == _threshold && LOW == _thresholdOld ){
    setEventQueue(EVENT_ACTIVE_TOUCH_SENSOR);
  }
  _thresholdOld = _threshold;

#ifdef _DEBUG_PRINT // _DEBUG_PRINT
  if(_value != _averageTouchSensor || _threshold != _thresholdOld ){
    SERIAL_PRINTF("touchRead() now:%d average:%d threshold:%d\n", _value, _averageTouchSensor,_threshold);  
  }
#endif              // _DEBUG_PRINT
}

/**
 * @brief ドアロックLED入力処理
 * @author sanlike
 * @date 2021/03/02
 */
void getDoorLockLed() {
  static uint8_t _activeCount = 0;
  static uint8_t _GpiControlStatusFix = 0xFF;
  byte _GpiControlStatus = digitalRead(PORT_IN_LOCK_LED);
  if (HIGH == _GpiControlStatus) {
    //digitalWrite(PORT_OUT_ON_BORAD_LED, LOWk);
    _activeCount = 0;
    if (HIGH != _GpiControlStatusFix) {
      setEventQueue(EVENT_LOCK_LED_OFF);
      _GpiControlStatusFix = _GpiControlStatus;
    }
  } else {
    //digitalWrite(PORT_OUT_ON_BORAD_LED, HIGH);
    if (++_activeCount >= PORT_IN_AVERAGE_NUM) {
      _activeCount = PORT_IN_AVERAGE_NUM;
      if (LOW != _GpiControlStatusFix) {
        setEventQueue(EVENT_LOCK_LED_ON);
        _GpiControlStatusFix = _GpiControlStatus;
      }
    }
  }
}

/**
 * @brief ACC入力処理
 * @author sanlike
 * @date 2021/03/02
 */
void getAccStatus() {
  static uint8_t _activeCount = 0;
  static uint8_t _GpiControlStatusFix = 0xFF;
  byte _GpiControlStatus = digitalRead(PORT_IN_ACC);
  if (HIGH == _GpiControlStatus) {
    _activeCount = 0;
    if (HIGH != _GpiControlStatusFix) {
      setEventQueue(EVENT_ACC_OFF);
      _GpiControlStatusFix = _GpiControlStatus;
    }
  } else {
    if (++_activeCount >= PORT_IN_AVERAGE_NUM) {
      _activeCount = PORT_IN_AVERAGE_NUM;
      if (LOW != _GpiControlStatusFix) {
        setEventQueue(EVENT_ACC_ON);
        _GpiControlStatusFix = _GpiControlStatus;
      }
    }
  }
}

/**
 * @brief ドアロック信号制御処理
 * @author sanlike
 * @date 2021/03/02
 */
void doorLockSignalControl() {
  static uint8_t _activeCounter = 0;
  if (subEventID & SUB_EVENT_SET_DOOR_LOCK_SIGNAL) {
    if ( ++_activeCounter >= DOOR_LOOK_ACTIVE_COUNT ) {
      subEventID &= ~SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
    }
    digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, HIGH);
  } else {
    _activeCounter = 0;
    digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
  }
}

/**
 * @brief イベント制御処理
 * @author sanlike
 * @date 2021/03/02
 */
void EventControl(EventID _emEventID) {
  static BEACON_STATUS_ID _beaconStatusIdOld = BEACON_STATUS_NON;
  static DOOR_STATUS_ID _doorStatusIdOld = DOOR_STATUS_NON;
  switch(_emEventID){
  case EVENT_LOST_BEACON:
    beaconStatusID = BEACON_STATUS_LOST;
    break;
  case EVENT_FOUND_BEACON:
    beaconStatusID = BEACON_STATUS_FOUND;
    break;
  case EVENT_LOCK_LED_OFF:
    if( DOOR_STATUS_UNLOCK_WAIT == doorStatusID || DOOR_STATUS_NON == doorStatusID ){
      stopdoorLockLampOffTimer();
      doorStatusID = DOOR_STATUS_UNLOCK;
    }
    break;
  case EVENT_LOCK_LED_ON:
    if( DOOR_STATUS_LOCK_WAIT == doorStatusID || DOOR_STATUS_NON == doorStatusID ){
      stopdoorLockLampOffTimer();
      doorStatusID = DOOR_STATUS_UNLOCK;
    }
    break;
  case EVENT_ACC_OFF:
    break;
  case EVENT_ACC_ON:
    SERIAL_PRINTF("Going to sleep now\n");
    esp_deep_sleep_start();
    break;
  case EVENT_DOOR_LOCK_SIGNAL:
    break;
  case EVENT_DOOR_UNLOCK_SIGNAL:
    break;
  case EVENT_ACTIVE_TOUCH_SENSOR:
    if( BEACON_STATUS_FOUND == beaconStatusID ){
      switch( doorStatusID ) {
        case DOOR_STATUS_LOCK:
        case DOOR_STATUS_UNLOCK:
          startdoorLockLampOffTimer();
          doorStatusID = ( DOOR_STATUS_UNLOCK == doorStatusID ? DOOR_STATUS_LOCK_WAIT : DOOR_STATUS_UNLOCK_WAIT);
          SERIAL_PRINTF("sub event : SUB_EVENT_SET_DOOR_LOCK_SIGNAL\n");
          subEventID |= SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
          break;
        default:
          break;
      }
    }
    break;
  case EVENT_DOOR_LOCK_LAMP_TIMEOUT:
    switch( doorStatusID ) {
      case DOOR_STATUS_LOCK_WAIT:
        doorStatusID = DOOR_STATUS_UNLOCK;
        break;
      case DOOR_STATUS_UNLOCK_WAIT:
        doorStatusID = DOOR_STATUS_LOCK;
        break;
      default:
        /* 破棄 */
        break;
    }
    break;
  default:
    SERIAL_PRINTF("EventControl(): EVENT_NON\n");
    break;
  }

  // edge statusId
  if( doorStatusID != _doorStatusIdOld ){
    SERIAL_PRINTF("change doorStatusID %d(%s) -> %d(%s)\n", _doorStatusIdOld, dbgDoorStatusMsg[_doorStatusIdOld], doorStatusID, dbgDoorStatusMsg[doorStatusID]);
  }
  _doorStatusIdOld = doorStatusID;
  if( beaconStatusID != _beaconStatusIdOld ){
    SERIAL_PRINTF("change beaconStatusID %d(%s) -> %d(%s)\n", _beaconStatusIdOld, dbgBeaconStatusMsg[_beaconStatusIdOld], beaconStatusID, dbgBeaconStatusMsg[beaconStatusID]);
  }
  _beaconStatusIdOld = beaconStatusID;
}

/**
 * @brief ON BOARD LED表示処理
 * @author sanlike
 * @date 2021/03/02
 */
void dispLed() {
#if 1
  //SERIAL_PRINTF("dispLed() doConnectBleDevice: %d\n",doConnectBleDevice);
  digitalWrite(PORT_OUT_ON_BORAD_LED, (doConnectBleDevice == 1 ? HIGH : LOW));
#else
  // blink
  static int ledBlink = HIGH;
  digitalWrite(PORT_OUT_ON_BORAD_LED, ledBlink);
  ledBlink = (ledBlink == HIGH ? LOW : HIGH);
#endif
}

/**
 * @brief BLE Beaconスキャン処理
 * @author sanlike
 * @date 2021/03/02
 */
void bleScan() {
  static byte _doConnectBleDeviceOLD = 0;
  doConnectBleDevice = 0;
  //BLEScanResults foundDevices = pBLEScan->start(SCAN_TIME/*, false*/);
  //SERIAL_PRINTF("Scan start!\n");
  pBLEScan->start(SCAN_TIME, false);
  //SERIAL_PRINTF("Scan done!\n");
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

  if (doConnectBleDevice) {
    // Edge:ON
    if (0 == _doConnectBleDeviceOLD) {
      setEventQueue(EVENT_FOUND_BEACON);
    }
  } else {
    // Edge:OFF
    if (1 == _doConnectBleDeviceOLD) {
      setEventQueue(EVENT_LOST_BEACON);
    }
  }
  _doConnectBleDeviceOLD = doConnectBleDevice;
}

/**
 * @brief メインループ処理
 * @author sanlike
 * @date 2021/03/02
 */
void loop() {
  bleScan();
  //SERIAL_PRINTF("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  delay(2000);
}

/**
 * @brief イベントタスク
 * @author sanlike
 * @date 2021/03/02
 */
void task1( void *param )
{
  SERIAL_PRINTF( "task1() : start\n" );
  EventID _emEventID;
  while ( 1 ) {
    getEventQueue(&_emEventID);
    SERIAL_PRINTF( "task1(): id=%d msg=%s\n", _emEventID, dbgEventMsg[_emEventID] );
    EventControl(_emEventID);
    //vTaskDelay(1000);
  }
}

/**
 * @brief 時間制御タスク
 * @author sanlike
 * @date 2021/03/02
 */
void task2( void *param )
{
  SERIAL_PRINTF( "task2() : start\n");
  while ( 1 ) {
    if ( tm400msEvent ) {
      tm400msEvent = 0;
      doorLockSignalControl();
    }
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
