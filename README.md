# giulietta940-smartlock

アルファロメオジュリエッタ向け、簡易スマートロックソフトウェア

ABA-940141

## DEMO

xxxxxx

## Features

bluetoothのiBeaconを利用したス車のマートロックソフトウェアです。

iBeaconのデバイスが車に近づくことでunlockし、離れることでlockします。

## Requirement

1. セントラル　：マイコン

    * ESP-WROOM-32(検証機:ESP32 DEVKITV1)

2. ペリフェラル：スマートフォン等

    * Android(検証機：v10)
    * bluetooth 4.0+(BLE)
    * Beacon Simlator v1.5.1

## Installation

1. [GitHub(sanlike0911)](https://github.com/sanlike0911/giulietta940-smartlock)からソースコードをダウンロードする

2. bluetooth(iBeacon)機器のUUIDをプログラムに書き込む

    ```ino
    /* BLE device uuid */
    #define BLE_ADVERTISED_DEVIECE_UUID   "{{ Set your device UUID }}"
    ```

3. Arduino IDEでビルドする

4. ESP32-WROOM-32にソフトを書き込む

## Usage

## Note

このプログラムを使用してのトラブルには一切責任を負いません。
自己の責任で安全に行ってください。
Good Luck!!

## Author

* sanlike

## License

[MIT license](https://en.wikipedia.org/wiki/MIT_License)
