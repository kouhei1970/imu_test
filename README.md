# imu_test
Raspberry Pi Pico で LSM9DS1 を SPIでアクセスするサンプル

## ビルド手順

念の為

`export PICO_SDK_PATH=../../pico-sdk`


pico-sdkディレクトリがある場所で

```
git clone https://github.com/kouhei1970/imu_test.git
cd imu_test
mkdir build
cd build
cmake ..
make
```

## 接続

Picoとの接続は以下の様にするものとします。

|Pico側|LSM9DS1ボード側|備考|
|---|---|---|
|GPIO 1 (pin 2) SPI0 CSn|CSAG|加速度・ジャイロ選択|
|GPIO 4 (pin 6) MISO/SPI0 RX|SDO（２ピンとも）|センサからデータ出力|
|GPIO 5 (pin 7) SPI00 CSn|CSM|地磁気計選択|
|GPIO 6 (pin 9) SCK/SPI0 SCK|SCL|クロック|
|GPIO 7 (pin 10) MOSI/SPI0 TX|SDA|センサへデータ出力|

