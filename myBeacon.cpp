#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEUtils.h>

#include "myBeacon.h"

bool myBeaconAdvertisedDevice::createByAdvertisedDevice(BLEAdvertisedDevice advertisedDevice) {

  char work[7];
  
  // 16進データをもらう
  String hexString = (String) BLEUtils::buildHexData(nullptr, (uint8_t*)advertisedDevice.getManufacturerData().data(), advertisedDevice.getManufacturerData().length());
  // iBeaconかチェック
  if (hexString.substring(0, 8).equals("4c000215")) {
    // iBeaconのとき
    uuid =  hexString.substring(8, 16) + "-" + 
            hexString.substring(16, 20) + "-" + 
            hexString.substring(20, 24) + "-" + 
            hexString.substring(24, 28) + "-" + 
            hexString.substring(28, 40);  
    ("0x" + hexString.substring(40, 44)).toCharArray(work, 7);
    major = (uint16_t) atof(work);
    ("0x" + hexString.substring(44, 48)).toCharArray(work, 7);
    minor = (uint16_t) atof(work);
    rssi = advertisedDevice.getRSSI();
    return true;
  } else {
    // iBeaconじゃないとき
    return false;
  }
}
