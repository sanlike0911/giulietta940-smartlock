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
  EVENT_MAX
};

/* sub event id */
#define SUB_EVENT_NON                     0x00
#define SUB_EVENT_SET_DOOR_LOCK_SIGNAL    0x01

/* debug Status Message */
char *dbgStatusMsg[] = {
  "STATUS_NON",
  "STATUS_DOOR_UNLOCK",
  "STATUS_DOOR_LOCK",
  "STATUS_MAX",
};

/* status id */
enum STATUS_ID {
  STATUS_NON = (uint8_t)0,
  STATUS_DOOR_UNLOCK,
  STATUS_DOOR_LOCK,
  STATUS_MAX
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
void checkLockLed();
void checkAcc();
void doorLockSignalControl();
void GpiControl();
void print_wakeup_reason();
void startdoorLockLampOffTimer();
void stopdoorLockLampOffTimer();
void startDoorLockFuncInvalidTimer(float _seconds);
void stopDoorLockFuncInvalidTimer();

void task1( void *param );
void task2( void *param );

bool setEventQueue( EventID _emEventID );
bool getEventQueue( EventID* _emEventID );

void tickerMainLoopTimer();
void tickerDoorLockLampOffTimer();
void tickerDoorLockFuncInvalidTimer();

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
volatile STATUS_ID statusID = STATUS_NON;

// Flgs
volatile bool doorLockFuncInvalid = false;
volatile bool doorLockLampOffTimerTMO = false;

// BLE
BLEScan *pBLEScan;
volatile uint8_t doConnectBleDevice = 0;

// Ticker
Ticker tickerMainLoop;
Ticker doorLockLampOffTimer;
Ticker doorLockFuncInvalidTimer;

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

  /* 起動直後のドアロック機能無効タイマ */
  startDoorLockFuncInvalidTimer(DOOR_LOCK_FUNC_INVALID_TIMER);

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
  checkLockLed();
  /* 入力：車両側)ACC状態 */
  checkAcc();
}

/**
 * @brief ドアロック機能無効タイマの開始
 * @author sanlike
 * @date 2021/03/08
 */
void startDoorLockFuncInvalidTimer(float _seconds){
  // タイマが起動していたら停止する
  stopDoorLockFuncInvalidTimer();
  // ドアロック機能禁止:禁止[true]
  doorLockFuncInvalid = true;
  // タイマ開始
  doorLockFuncInvalidTimer.once(_seconds, tickerDoorLockFuncInvalidTimer);
}

/**
 * @brief ドアロック機能無効タイマの停止
 * @author sanlike
 * @date 2021/03/08
 */
void stopDoorLockFuncInvalidTimer(){
  doorLockFuncInvalidTimer.detach();
}

/**
 * @brief ドアロック機能無効タイマー
 * @author sanlike
 * @date 2021/03/08
 */
void tickerDoorLockFuncInvalidTimer(){
  // ドアロック機能禁止:解除[false]
  doorLockFuncInvalid = false;
}

/**
 * @brief ドアロックランプオフ監視タイマの開始
 * @author sanlike
 * @date 2021/03/08
 */
void startdoorLockLampOffTimer(){
  // タイマが起動していたら停止する
  stopdoorLockLampOffTimer();
  // ドアロックランプのオフタイマタイムアウト:無[false]
  doorLockLampOffTimerTMO = false;
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
 * @brief ドアロックランプオフ監視タイマ(ドアロックランプの2分後消灯機能を監視するタイマ。ドアロックランプはロック状態でも消灯するため、ステータス不一致を抑制する目的で使用)
 * @author sanlike
 * @date 2021/03/06
 */
void tickerDoorLockLampOffTimer(){
  doorLockLampOffTimerTMO = true;
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
 * @brief ドアロックLED入力処理
 * @author sanlike
 * @date 2021/03/02
 */
void checkLockLed() {
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
void checkAcc() {
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
  if(false==doorLockFuncInvalid){
    if (subEventID & SUB_EVENT_SET_DOOR_LOCK_SIGNAL) {
      if ( ++_activeCounter >= DOOR_LOOK_ACTIVE_COUNT ) {
        subEventID &= ~SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
      }
      digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, HIGH);
    } else {
      _activeCounter = 0;
      digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
    }
  } else {
    _activeCounter = 0;
    subEventID &= ~SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
  }
}

/**
 * @brief イベント制御処理
 * @author sanlike
 * @date 2021/03/02
 */
void EventControl(EventID _emEventID) {
  static STATUS_ID _statusIDOld = STATUS_NON;
  switch(_emEventID){
  case EVENT_LOST_BEACON:
#if 0
    // アンロック状態の場合はドアロック信号を送る
    if( STATUS_DOOR_UNLOCK == statusID ){
      subEventID |= SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
    }
#else
    subEventID |= SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
#endif
    break;
  case EVENT_FOUND_BEACON:
#if 0
    // 通常ロック状態の場合はドアロック信号を送る
    if( STATUS_DOOR_LOCK == statusID ){
      subEventID |= SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
    }
#else
    subEventID |= SUB_EVENT_SET_DOOR_LOCK_SIGNAL;
#endif
    break;
  case EVENT_LOCK_LED_OFF:
#if 1   // test:doorLockLampOffTimer
    // door lampのON時間がタイムアウトした場合、door lock状態を継続する
    stopdoorLockLampOffTimer();
    if( true != doorLockLampOffTimerTMO ){
      statusID = STATUS_DOOR_UNLOCK;
    }
#else
    statusID = STATUS_DOOR_UNLOCK;
#endif  // test:doorLockLampOffTimer
    break;
  case EVENT_LOCK_LED_ON:
    statusID = STATUS_DOOR_LOCK;
#if 1   // test:doorLockLampOffTimer
    startdoorLockLampOffTimer();
#endif  // test:doorLockLampOffTimer
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
  default:
    SERIAL_PRINTF("EventControl(): EVENT_NON\n");
    break;
  }

  // edge statusId
  if( statusID != _statusIDOld ){
    SERIAL_PRINTF("change statusID %d(%s) -> %d(%s)\n", _statusIDOld, dbgStatusMsg[_statusIDOld], statusID, dbgStatusMsg[statusID]);
  }

  _statusIDOld = statusID;
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
