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
#define BLE_SCAN_TIME           (uint32_t)5 /* In seconds */
#define BLE_SLEEP_ENABLE_COUNT  (uint16_t)6  /* time = BLE_SCAN_TIME × BLE_SLEEP_ENABLE_COUNT*/
#define BLE_ADVERTISED_DEVIECE_UUID   "cb3b0426-10ec-45bd-b58e-f2858c0dbc2b"

/* GPIO settings */
#define PORT_OUT_ON_BORAD_LED         2
#define PORT_OUT_DOOR_LOCK_SIGNAL     17
#define PORT_IN_TOUCH_SENSOR          T6  /*  T6(GPIO14) */

/* touch sensor ID */
#define TOUCH_SENSOR_ID               6   /*  T6(GPIO14) */

/* port[out] setting */
#define DOOR_LOOK_SIGNAL_ACTIVE_EDGE_COUNT (uint8_t)2  /* active edge time = n × 400ms */

/* timer settings */
#define TICKER_MAIN_LOOP_INTERVAL         (uint32_t)200
#define DOOR_LOCK_SIGNAL_ACTIVE_DURATION  (uint32_t)800

/* event aueue settings */
#define QUEUE_LENGTH                  8       /* event queue length */

/* touch sensor 移動平均(n) */
#define AVEREGE_TOUCH_SENSOR_NUM      (uint8_t)4

/* touch sensor ヒステリシスループ閾値 */
#define TOUCH_SENSOR_THRESHOLD_ACTIVE   (uint16_t)30  /* active   :  0～30 */
#define TOUCH_SENSOR_THRESHOLD_DEACTIVE (uint16_t)50  /* deactive : 50～   */

/* debug Event Message */
char *dbgEventMsg[] = {
  "EVENT_NON",
  "EVENT_LOST_BEACON",
  "EVENT_FOUND_BEACON",
  "EVENT_TOUCH_SENSOR_DEACTIVE",
  "EVENT_TOUCH_SENSOR_ACTIVE",
  "EVENT_MAX"
};

/* event id */
enum EventID {
  EVENT_NON = (uint8_t)0,
  EVENT_LOST_BEACON,
  EVENT_FOUND_BEACON,
  EVENT_TOUCH_SENSOR_DEACTIVE,
  EVENT_TOUCH_SENSOR_ACTIVE,
  EVENT_MAX
};

/* debug beacon status message */
char *dbgBeaconStatusMsg[] = {
  "BEACON_STATUS_NON",
  "BEACON_STATUS_LOST",
  "BEACON_STATUS_FOUND",
  "BEACON_STATUS_MAX",
};

// /* beacon status id */
enum BEACON_STATUS_ID {
  BEACON_STATUS_NON = (uint8_t)0,
  BEACON_STATUS_LOST,
  BEACON_STATUS_FOUND,
  BEACON_STATUS_MAX
};

/* debug touchpad status message */
char *dbgTouchpadStatusMsg[] = {
  "TOUCHPAD_STATUS_NON",
  "TOUCHPAD_STATUS_ACTIVE",
  "TOUCHPAD_STATUS_DEACTIVE",
  "TOUCHPAD_STATUS_MAX",
};

// /* touchpad status id */
enum TOUCHPAD_STATUS_ID {
  TOUCHPAD_STATUS_NON = (uint8_t)0,
  TOUCHPAD_STATUS_ACTIVE,
  TOUCHPAD_STATUS_DEACTIVE,
  TOUCHPAD_STATUS_MAX
};

//*****************************************************************************
// macro function
//*****************************************************************************
#ifdef _DEBUG_PRINT // _DEBUG_PRINT
 #define SERIAL_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else               // _DEBUG_PRINT
 #define SERIAL_PRINTF(...)
#endif              // _DEBUG_PRINT

//*****************************************************************************
// prototypes
//*****************************************************************************
/* bletooth */
void bleScan();

/* display */
void dispLed();

/* door lock signal control */
void startTickerDoorLockSignalControl();
void stopTickerDoorLockSignalControl();

/* gpio */
void GpiControl();

/* touch sensor */
void getToucSensor();
void setupDeepSleepEnableToutchpad();
void interruptTouchSensor();

/* wikeup */
esp_sleep_wakeup_cause_t getWakeupReason();
touch_pad_t getWakeupTouchpadReason();

/* event */
void eventControl(EventID _emEventID);
bool setEventQueue( EventID _emEventID );
bool getEventQueue( EventID* _emEventID );
void jugeEventDoorLockSignal();

/* task */
void taskEventControl( void *param );
void taskTimeControl( void *param );

/* ticker */
void tickerFuncMainLoop();
void tickerFuncDoorLockSignalControl();

//*****************************************************************************
// variables
//*****************************************************************************
/* timer */
volatile uint32_t tm400msCount = 0;
volatile uint32_t tm1000msCount = 0;
volatile uint32_t tm10000msCount = 0;
volatile uint8_t  tm400msEvent = 0;
volatile uint8_t  tm1000msEvent = 0;
volatile uint8_t  tm10000msEvent = 0;

// Status
volatile BEACON_STATUS_ID beaconStatusId = BEACON_STATUS_NON;
volatile TOUCHPAD_STATUS_ID touchpadStatusId = TOUCHPAD_STATUS_NON;

// BLE
BLEScan *pBLEScan;
volatile bool doConnectBleDevice = false;

// Ticker
Ticker tickerMainLoop;
Ticker tickerDoorLockSignalControl;

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
public:
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // SERIAL_PRINTF("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      // アドバタイジングデータを受け取ったとき
      myBeaconAdvertisedDevice _mybcn;
      if (_mybcn.createByAdvertisedDevice(advertisedDevice)) {
        // SERIAL_PRINTF("Found an iBeacon!!\n");
        if (_mybcn.IsAdvertisedDevice(BLE_ADVERTISED_DEVIECE_UUID)) {
          doConnectBleDevice = true;
          BLEDevice::getScan()->stop();
#ifdef _DEBUG_PRINT   // _DEBUG_PRINT
          // char uuid[37];
          // _mybcn.getUUID().toCharArray(uuid, 37);
          // SERIAL_PRINTF("Found a device!! UUID: %s, Major: %d, Minor: %d, RSSI: %d \n", uuid, _mybcn.getMajor(), _mybcn.getMinor(), _mybcn.getRSSI());
#endif                // _DEBUG_PRINT
        } else {}
      } else {}
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

  /* create serial */
  Serial.begin(115200);

  /* create gpio */
  pinMode(PORT_OUT_ON_BORAD_LED, OUTPUT);
  digitalWrite(PORT_OUT_ON_BORAD_LED, HIGH);

  pinMode(PORT_OUT_DOOR_LOCK_SIGNAL, OUTPUT);
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);

  /* esp32 device infomation */
  // SERIAL_PRINTF("--------------- esp32 device infomation ---------------\n");
  // SERIAL_PRINTF("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  // SERIAL_PRINTF("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  // SERIAL_PRINTF("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  // SERIAL_PRINTF("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());

  /* wakeup reason */
  SERIAL_PRINTF("--------------- esp32 wakeup reason ---------------\n");
  esp_sleep_wakeup_cause_t _wakeupReason = getWakeupReason();
  touch_pad_t _wakeupTouchpadReason = getWakeupTouchpadReason();
#if 1
  /* first boot? */
  if( ESP_SLEEP_WAKEUP_UNDEFINED == _wakeupReason || TOUCH_SENSOR_ID != _wakeupTouchpadReason ){
    setupDeepSleepEnableToutchpad();
  }
#endif
  
  /* create ticker */
  tickerMainLoop.attach_ms( TICKER_MAIN_LOOP_INTERVAL, tickerFuncMainLoop );

  /* create event-group */
  hQueue = xQueueCreate( QUEUE_LENGTH, sizeof(uint8_t));

  /* create task */
  xTaskCreatePinnedToCore( taskEventControl,   /* タスクの入口となる関数名 */
                           "taskEventControl", /* タスクの名称 */
                           0x800,           /* スタックサイズ */
                           NULL,            /* パラメータのポインタ */
                           1,               /* プライオリティ */
                           NULL,            /* ハンドル構造体のポインタ */
                           0 );             /* 割り当てるコア (0/1) */

  xTaskCreatePinnedToCore( taskTimeControl,
                           "taskTimeControl",
                           0x800,
                           NULL,
                           1,
                           NULL,
                           1 );

  /* Brownout detector was triggered 対策 */
  delay(100);

  /* create ble */
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
}

/**
 * @brief setup deep sleep enable toutchpad
 * @author sanlike
 * @date 2021/03/24
 */
void setupDeepSleepEnableToutchpad(){
  /* Setup interrupt on Touch Pad 6 (GPIO14) */
  touchAttachInterrupt(PORT_IN_TOUCH_SENSOR, interruptTouchSensor, TOUCH_SENSOR_THRESHOLD_ACTIVE );
  /* Configure Touchpad as wakeup source */
  esp_sleep_enable_touchpad_wakeup();
  /* start deep sleep */
  SERIAL_PRINTF("Going to sleep now\n");
  esp_deep_sleep_start();
}

/**
 * @brief print wakeup reason
 * @author sanlike
 * @date 2021/03/03
 */
esp_sleep_wakeup_cause_t getWakeupReason() {
  esp_sleep_wakeup_cause_t _wakeupReason = esp_sleep_get_wakeup_cause();
#ifdef _DEBUG_PRINT // _DEBUG_PRINT
  switch(_wakeupReason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 :      SERIAL_PRINTF("Wakeup caused by external signal using RTC_IO\n"); break;
    case ESP_SLEEP_WAKEUP_EXT1 :      SERIAL_PRINTF("Wakeup caused by external signal using RTC_CNTL\n"); break;
    case ESP_SLEEP_WAKEUP_TIMER :     SERIAL_PRINTF("Wakeup caused by timer\n"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD :  SERIAL_PRINTF("Wakeup caused by touchpad\n"); break;
    case ESP_SLEEP_WAKEUP_ULP :       SERIAL_PRINTF("Wakeup caused by ULP program\n"); break;
    default :                         SERIAL_PRINTF("Wakeup was not caused by deep sleep: %d\n", _wakeupReason); break;
  }
#endif              // _DEBUG_PRINT
  return _wakeupReason;
}

/**
 * @brief print wakeup touchpad reason
 * @author sanlike
 * @date 2021/03/24
 */
touch_pad_t getWakeupTouchpadReason() {
  touch_pad_t _touchPin = esp_sleep_get_touchpad_wakeup_status();
#ifdef _DEBUG_PRINT // _DEBUG_PRINT
  switch(_touchPin)
  {
    case 0  : SERIAL_PRINTF("Touch detected on GPIO  4\n"); break;
    case 1  : SERIAL_PRINTF("Touch detected on GPIO  0\n"); break;
    case 2  : SERIAL_PRINTF("Touch detected on GPIO  2\n"); break;
    case 3  : SERIAL_PRINTF("Touch detected on GPIO 15\n"); break;
    case 4  : SERIAL_PRINTF("Touch detected on GPIO 13\n"); break;
    case 5  : SERIAL_PRINTF("Touch detected on GPIO 12\n"); break;
    case 6  : SERIAL_PRINTF("Touch detected on GPIO 14\n"); break;
    case 7  : SERIAL_PRINTF("Touch detected on GPIO 27\n"); break;
    case 8  : SERIAL_PRINTF("Touch detected on GPIO 33\n"); break;
    case 9  : SERIAL_PRINTF("Touch detected on GPIO 32\n"); break;
    default : SERIAL_PRINTF("Wakeup not by touchpad :%d\n", _touchPin); break;
  }
#endif              // _DEBUG_PRINT
  return _touchPin;
}

/**
 * @brief タッチセンサ割り込み処理
 * @author sanlike
 * @date 2021/03/24
 */
void interruptTouchSensor(){
  SERIAL_PRINTF("interruptTouchSensor()\n");
  return;
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
void GpiControl () {
  /* 入力：touchセンサ */
  getToucSensor();
}

/**
 * @brief ドアロック信号制御タイマの開始
 * @author sanlike
 * @date 2021/03/24
 */
void startTickerDoorLockSignalControl(){
  // タイマが起動していたら停止する
  stopTickerDoorLockSignalControl();
  // active
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, HIGH);
  SERIAL_PRINTF("@@@ door lock signal : high\n");
  // タイマ開始
  tickerDoorLockSignalControl.once_ms(DOOR_LOCK_SIGNAL_ACTIVE_DURATION, tickerFuncDoorLockSignalControl);
}

/**
 * @brief ドアロック信号制御タイマの停止
 * @author sanlike
 * @date 2021/03/24
 */
void stopTickerDoorLockSignalControl(){
  // diactive
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
  // detach timer
  tickerDoorLockSignalControl.detach();
}

/**
 * @brief ドアロック信号制御(HIGH->LOW)
 * @author sanlike
 * @date 2021/03/24
 */
void tickerFuncDoorLockSignalControl(){
  // diactive
  digitalWrite(PORT_OUT_DOOR_LOCK_SIGNAL, LOW);
  SERIAL_PRINTF("@@@ door lock signal : low\n");
}

/**
 * @brief 割り込みタイマ(200ms)
 * @author sanlike
 * @date 2021/03/02
 */
void tickerFuncMainLoop() {
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
  uint16_t _value = touchRead(PORT_IN_TOUCH_SENSOR);
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
    setEventQueue(EVENT_TOUCH_SENSOR_ACTIVE);
  } else if( LOW == _threshold && HIGH == _thresholdOld ){
    setEventQueue(EVENT_TOUCH_SENSOR_DEACTIVE);
  }
  _thresholdOld = _threshold;

#ifdef _DEBUG_PRINT // _DEBUG_PRINT
  if(_value != _averageTouchSensor || _threshold != _thresholdOld ){
    SERIAL_PRINTF("touchRead() now:%d average:%d threshold:%d\n", _value, _averageTouchSensor,_threshold);  
  }
#endif              // _DEBUG_PRINT
}

/**
 * @brief BLE Beaconスキャン処理
 * @author sanlike
 * @date 2021/03/02
 */
void bleScan() {
  static bool _doConnectBleDeviceOLD = false;
  doConnectBleDevice = false;
  // SERIAL_PRINTF("Scan start!\n");
  BLEScanResults _foundDevices = pBLEScan->start(BLE_SCAN_TIME, false);
  // SERIAL_PRINTF("Scan done!\n");
  pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory

  if ( doConnectBleDevice != _doConnectBleDeviceOLD ) {
    if ( true == doConnectBleDevice ) {
      // Edge:ON
      setEventQueue(EVENT_FOUND_BEACON);
    } else {
      // Edge:OFF
      setEventQueue(EVENT_LOST_BEACON);
    }
  }
  _doConnectBleDeviceOLD = doConnectBleDevice;
}

/**
 * @brief ドアロック信号イベント判定処理
 * @author sanlike
 * @date 2021/03/24
 */
void jugeEventDoorLockSignal(){
  if( BEACON_STATUS_FOUND == beaconStatusId && 
      TOUCHPAD_STATUS_ACTIVE == touchpadStatusId ){
    startTickerDoorLockSignalControl();
  }
  return;
}

/**
 * @brief イベント制御処理
 * @author sanlike
 * @date 2021/03/02
 */
void eventControl(EventID _emEventID) {
  static BEACON_STATUS_ID _beaconStatusIdOld = BEACON_STATUS_NON;
  static TOUCHPAD_STATUS_ID _touchpadStatusIdOld = TOUCHPAD_STATUS_NON;
  switch(_emEventID){
  case EVENT_LOST_BEACON:
    beaconStatusId = BEACON_STATUS_LOST;
    break;
  case EVENT_FOUND_BEACON:
    beaconStatusId = BEACON_STATUS_FOUND;
    jugeEventDoorLockSignal();
    break;
  case EVENT_TOUCH_SENSOR_DEACTIVE:
    touchpadStatusId = TOUCHPAD_STATUS_DEACTIVE;
    break;
  case EVENT_TOUCH_SENSOR_ACTIVE:
    touchpadStatusId = TOUCHPAD_STATUS_ACTIVE;
    jugeEventDoorLockSignal();
    break;
  default:
    SERIAL_PRINTF("eventControl(): EVENT_NON\n");
    break;
  }

  // edge beaconStatusId
  if( beaconStatusId != _beaconStatusIdOld ){
    SERIAL_PRINTF("change beaconStatusId %d(%s) -> %d(%s)\n", _beaconStatusIdOld, dbgBeaconStatusMsg[_beaconStatusIdOld], beaconStatusId, dbgBeaconStatusMsg[beaconStatusId]);
  }
  _beaconStatusIdOld = beaconStatusId;

  // edge touchpadStatusId
  if( touchpadStatusId != _touchpadStatusIdOld ){
    SERIAL_PRINTF("change touchpadStatusId %d(%s) -> %d(%s)\n", _touchpadStatusIdOld, dbgTouchpadStatusMsg[_touchpadStatusIdOld], touchpadStatusId, dbgTouchpadStatusMsg[touchpadStatusId]);
  }
  _touchpadStatusIdOld = touchpadStatusId;

}

/**
 * @brief ON BOARD LED表示処理
 * @author sanlike
 * @date 2021/03/02
 */
void dispLed() {
#if 1
  //SERIAL_PRINTF("dispLed() doConnectBleDevice: %d\n",doConnectBleDevice);
  digitalWrite(PORT_OUT_ON_BORAD_LED, (beaconStatusId == BEACON_STATUS_FOUND?HIGH:LOW));
#else
  // blink
  static int ledBlink = HIGH;
  digitalWrite(PORT_OUT_ON_BORAD_LED, ledBlink);
  ledBlink = (ledBlink == HIGH ? LOW : HIGH);
#endif
}

/**
 * @brief メインループ処理
 * @author sanlike
 * @date 2021/03/02
 */
void loop() {
  static uint16_t _beaconLostCounter = 0;
  bleScan();
#if 1   // test
  if( false == doConnectBleDevice ){
    /* デバイス未検知継続でスリープする：time= BLE_SCAN_TIME(5sec) * BLE_SLEEP_ENABLE_COUNT */
    if( ++_beaconLostCounter > BLE_SLEEP_ENABLE_COUNT ){
      setupDeepSleepEnableToutchpad(); 
    } 
  } else {
    _beaconLostCounter = 0;
  }
#endif  // test
}

/**
 * @brief イベントタスク
 * @author sanlike
 * @date 2021/03/02
 */
void taskEventControl( void *param )
{
  SERIAL_PRINTF( "taskEventControl() : start\n" );
  EventID _emEventID;
  while ( 1 ) {
    getEventQueue(&_emEventID);
    SERIAL_PRINTF( "taskEventControl(): id=%d msg=%s\n", _emEventID, dbgEventMsg[_emEventID] );
    eventControl(_emEventID);
  }
}

/**
 * @brief 時間制御タスク
 * @author sanlike
 * @date 2021/03/02
 */
void taskTimeControl( void *param )
{
  uint8_t _tm30000msCount = 0;
  SERIAL_PRINTF( "taskTimeControl() : start\n");
  while ( 1 ) {
    /* 400ms */
    if ( tm400msEvent ) {
      tm400msEvent = 0;
    }
    /* 1sec */
    if ( tm1000msEvent ) {
      tm1000msEvent = 0;
      dispLed();
    }
    /* 10sec */
    if ( tm10000msEvent ) {
      tm10000msEvent = 0;
    }
  }
}
