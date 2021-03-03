#ifndef _MY_BEACON_H_
#define _MY_BEACON_H_

class myBeaconAdvertisedDevice
{
  private:
    String    uuid;
    uint16_t  major;
    uint16_t  minor;
    int       rssi;
  public:
    bool      createByAdvertisedDevice(BLEAdvertisedDevice advertisedDevice); // BLEAdvertisedDeviceからデータを組み立てる
    bool      IsAdvertisedDevice(String _uuid) { return _uuid.equalsIgnoreCase(uuid); }
    String    getUUID() { return uuid; }
    uint16_t  getMajor() { return major; }
    uint16_t  getMinor() { return minor; }
    int       getRSSI() { return rssi; }
};

#endif  // _MY_BEACON_H_
