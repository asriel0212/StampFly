/*このコードは、Arduinoプログラムの基本構成で、ドローンやマルチコプターなどの飛行制御システムにおける初期化処理と制御ループの構成を記述しています。



初期化 (setup()):

マルチコプターのハードウェア（センサーやモーター制御ピン）およびソフトウェア（制御ロジック、ゲイン）の初期化を実施します。

制御ループ (loop()):

400Hzの頻度で飛行制御を実行し、リアルタイムの飛行状態を安定化させます。
*/
  
#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"

//VL53L0X_ADDRESS           0x29
//MPU6886_ADDRESS           0x68
//BMP280_ADDRESS            0x76

void setup() {  
  init_copter();
  delay(100);
}

void loop() {
  loop_400Hz();
}
