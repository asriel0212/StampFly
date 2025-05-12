/*(cpp)このコードは、以下の機能を提供するIMU（Inertial Measurement Unit）モジュールです：

BMI270センサーを使用して、マルチコプターやその他の移動体の姿勢や動きを測定。

センサー値を物理単位に変換し、飛行制御やデバッグに利用。

テスト関数を用いてセンサーの動作を確認可能。


(hpp)このヘッダーファイルは、BMI270センサーを利用するモジュールに必要な機能を宣言し、センサーの初期化やデータ取得を可能にします。



BMI270は、Bosch Sensortec社が開発した、ウェアラブルデバイス向けに最適化された超低電力の6軸IMU（慣性測定ユニット）です。

IMUセンサーとは
３次元の慣性運動（直進運動と回転運動）を検知するセンサー
機能
*/

#include <Arduino.h> 
#include "common.h"
#include "bmi2.h"
#include "imu.hpp"
#include <bmi270.h>
// 加速度・ジャイロの値を格納する変数
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z; //加速度センサーのX・Y・Z軸の値を格納（m/s²）。ジャイロセンサーのX・Y・Z軸の値を格納（rad/s）。

// BMI270センサーのデータ構造体
struct bmi2_sens_data imu_data; 

// **IMUの初期化**
void imu_init(void)
{
  int8_t st;
  uint8_t data=0;


  USBSerial.printf("Start IMU Initialize!\n\r");

  // **SPI通信の設定（CSピンをHIGHに設定）**
  pinMode(46, OUTPUT);//CSを設定
  digitalWrite(46, 1);//CSをHIGH
  pinMode(12, OUTPUT);//CSを設定
  digitalWrite(12, 1);//CSをHIGH
  delay(5); //短時間の待機を挿入。
  USBSerial.printf("SPI Initilize status:%d\n\r",spi_init()); //spi_init() を呼び出して、SPI通信を開始。

  //BMI270 Init
  bmi270_dev_init();  //BMI270のデバイスを準備。
  st = bmi270_init(pBmi270); //BMI270を初期化し、st にステータスを保存。
  USBSerial.printf("#INIT Status:%d\n\r", st);
  if (st!=0)
  {
    USBSerial.printf("BMI270 INIT Fail!\n\r");
    while(1);
  }
  // **BMI270のチップ情報を取得**
  USBSerial.printf("#Chip ID DEV:%02X\n\r", Bmi270.chip_id);
  USBSerial.printf("#APP_STATUS:%02X\n\r", Bmi270.aps_status); //BMI270のチップIDとアプリケーションステータスをシリアル出力。

   // **初期化ステータス確認**
  USBSerial.printf("#INIT_STATUS Read:%d\n\r",bmi2_get_regs(0x21, &data, 1, pBmi270));  
  USBSerial.printf("#INIT_STATUS:%02X\n\r", data);
  //IMU Config
  // **加速度・ジャイロセンサーの設定**
  USBSerial.printf("#Config Status:%d\n\r", set_accel_gyro_config(pBmi270));
  uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO }; //IMUのセンサー有効化
  USBSerial.printf("#Sensor enable Status:%d\n\r", bmi2_sensor_enable(sensor_list, 2, pBmi270));
   //加速度・ジャイロセンサーを有効化。
}

// **IMUデータの更新（最新のセンサー値を取得）**
void imu_update(void)
{
    bmi2_get_sensor_data(&imu_data, pBmi270);//使って最新のセンサーデータを取得し、imu_data に保存。
}
// **各軸の加速度取得（m/s²）**
float imu_get_acc_x(void)
{
    return lsb_to_mps2(imu_data.acc.x, 8.0, 16)/GRAVITY_EARTH; //BMI270のLSB値を物理単位に変換（m/s²）。
    //GRAVITY_EARTH で割ることで、重力加速度との比較が可能。
}

float imu_get_acc_y(void)
{
    return lsb_to_mps2(imu_data.acc.y, 8.0, 16)/GRAVITY_EARTH; //lsb_to_rps() を使って角速度をラジアン毎秒（rad/s）に変換。
}

float imu_get_acc_z(void)
{
    return lsb_to_mps2(imu_data.acc.z, 8.0, 16)/GRAVITY_EARTH;
}

// **各軸のジャイロ値取得（rad/s）**
float imu_get_gyro_x(void)
{
    return lsb_to_rps(imu_data.gyr.x, DPS20002RAD, 16);
}

float imu_get_gyro_y(void)
{
    return lsb_to_rps(imu_data.gyr.y, DPS20002RAD, 16);
}

float imu_get_gyro_z(void)
{
    return lsb_to_rps(imu_data.gyr.z, DPS20002RAD, 16);
}

// **IMUのテスト（データを取得して出力）**
void imu_test(void)
{
  u_long st, now, old, end;
  uint16_t count;
  uint8_t ret;
  st=micros(); //現在のマイクロ秒を取得し、タイムスタンプを設定。
  now = st;
  old = st;
  struct bmi2_sens_data imu_data;
  usleep(1000*5000); //5秒間スリープ。
  
 //データ取得とシリアル出力
  while(1)
  {
    old = now;
    now = micros();
    
     // **IMUデータ取得**
    ret = bmi2_get_sensor_data(&imu_data, pBmi270); //bmi2_get_sensor_data() を呼び出してIMUデータを取得。
    //USBSerial.printf("%d\n\r", ret);
     // **加速度・ジャイロデータを変換**
    acc_x = lsb_to_mps2(imu_data.acc.x, 8.0, 16); //加速度データを m/s² に変換。
    acc_y = lsb_to_mps2(imu_data.acc.y, 8.0, 16);
    acc_z = lsb_to_mps2(imu_data.acc.z, 8.0, 16);
    gyro_x = lsb_to_rps(imu_data.gyr.x, DPS10002RAD, 16); //ジャイロデータを rad/s に変換。
    gyro_y = lsb_to_rps(imu_data.gyr.y, DPS10002RAD, 16);
    gyro_z = lsb_to_rps(imu_data.gyr.z, DPS10002RAD, 16);
    #if 1
    
    // **シリアルモニタへデータ出力**
    USBSerial.printf("%8.4f %7.5f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %d\n\r", 
      (float)(now-st)*1.0e-6, // 開始からの経過時間
      (float)(now - old)*1.0e-6, // 前回取得からの時間
      acc_x, // 加速度
      acc_y,
      acc_z,
      gyro_x, // ジャイロ
      gyro_y,
      gyro_z,
      ret); // 取得ステータス
  #endif
  }  
}

