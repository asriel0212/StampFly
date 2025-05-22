/*マルチコプターやロボットなどの動的システムで使用されるセンサー操作を管理するためのヘッダーファイルです。各センサーのデータを収集・処理し、飛行制御のための基盤を提供します。

各センサー（IMU、電圧監視センサー、距離測定センサー）のデータを処理。

姿勢推定や高度制御のためのデータを準備。

飛行制御システムに統合される。



ドローン: 飛行中の高度、姿勢、速度をリアルタイムで監視・制御。

自律ロボット: 動作中の加速度、角速度、電圧状況を管理。

IoTデバイス: センサーを用いた環境データの収集。
*/

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp" 
#include "flight_control.hpp"

Madgwick Drone_ahrs;      //姿勢推定アルゴリズム
Alt_kalman EstimatedAltitude;    //不確実な情報を統合して、より正確な推定を行うアルゴリズム

//INA3221（電圧監視IC）のインスタンスを生成し、I2Cアドレスを指定して初期化
INA3221 ina3221(INA3221_ADDR40_GND);//I2Cアドレスを0x40にセットする (A0 pin -> GND)
Filter acc_filter;
Filter az_filter;
Filter voltage_filter;
Filter raw_ax_filter;
Filter raw_ay_filter;
Filter raw_az_filter;
Filter raw_az_d_filter;
Filter raw_gx_filter;
Filter raw_gy_filter;
Filter raw_gz_filter;
Filter alt_filter;

//センサーデータ
volatile float Roll_angle=0.0f, Pitch_angle=0.0f, Yaw_angle=0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Roll_rate_offset=0.0f, Pitch_rate_offset=0.0f, Yaw_rate_offset=0.0f;
volatile float Accel_z_d;
volatile float Accel_z_offset=0.0f;
volatile float Accel_x_raw,Accel_y_raw,Accel_z_raw;
volatile float Accel_x,Accel_y,Accel_z;
volatile float Roll_rate_raw,Pitch_rate_raw,Yaw_rate_raw;
volatile float Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
volatile float Altitude = 0.0f;
volatile float Altitude2 = 0.0f;
volatile float Alt_velocity = 0.0f;
volatile uint8_t Alt_control_ok = 0;
volatile float Az=0.0;
volatile float Az_bias=0.0;
int16_t deltaX,deltaY;

volatile uint16_t Offset_counter = 0;

volatile float Voltage;
float Acc_norm=0.0f;
//quat_t Quat;
float Over_g=0.0f, Over_rate=0.0f;
uint8_t OverG_flag = 0;
volatile uint8_t Under_voltage_flag = 0;
//volatile uint8_t ToF_bottom_data_ready_flag;
volatile uint16_t Range=1000;

//I2Cデバイスをスキャンし、接続されているI2Cアドレスを表示
uint8_t scan_i2c()
{
  USBSerial.println ("I2C scanner. Scanning ...");
  delay(50);
  byte count = 0;
  for (uint8_t i = 1; i < 127; i++)
  {
    Wire1.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire1.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      USBSerial.print ("Found address: ");
      USBSerial.print (i, DEC);
      USBSerial.print (" (0x");
      USBSerial.print (i, HEX); 
      USBSerial.println (")");
      count++;
    }
  }
  USBSerial.print ("Found ");      
  USBSerial.print (count, DEC);        // numbers of devices
  USBSerial.println (" device(s).");
  return count;
}

//ジャイロや加速度センサーのオフセット値をリセットし、カウンターを初期化
void sensor_reset_offset(void)
{
  Roll_rate_offset = 0.0f;
  Pitch_rate_offset = 0.0f;
  Yaw_rate_offset = 0.0f;
  Accel_z_offset = 0.0f;
  Offset_counter = 0;
}

//オフセットの平均の計算
void sensor_calc_offset_avarage(void)
{
  Roll_rate_offset = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
  Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
  Yaw_rate_offset = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
  Accel_z_offset = (Offset_counter * Accel_z_offset + Accel_z_raw) / (Offset_counter + 1);

  Offset_counter++;
}

//INA3221を使って1000回電圧を取得して表示
void test_voltage(void)
{
  for (uint16_t i=0; i<1000; i++)
  {
    USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
  }
}

//Madgwickフィルタをリセットし、姿勢データを初期化
void ahrs_reset(void)
{
  Drone_ahrs.reset();
}

//I2C通信を設定し、センサーを初期化
void sensor_init()
{
  //beep_init();　<-　意味不
 
  Wire1.begin(SDA_PIN, SCL_PIN,400000UL);

  //I2C通信が確認できない場合、エラーメッセージを出し、システムを停止
  if(scan_i2c()==0)
  {
    USBSerial.printf("No I2C device!\r\n");
    USBSerial.printf("Can not boot AtomFly2.\r\n");
    while(1);
  }

  tof_init();
  imu_init();
  Drone_ahrs.begin(400.0);
  ina3221.begin(&Wire1);
  ina3221.reset();  
  voltage_filter.set_parameter(0.005, 0.0025);
  
  uint16_t cnt=0;
  while(cnt<3)//三回ループ
  {
    if(ToF_bottom_data_ready_flag)  //もしtof準備フラグが１だったら
    {
      ToF_bottom_data_ready_flag = 0;
      cnt++;
      USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());//下のtofの距離の取得
    }
  }
  delay(10);
 
  //Acceleration filter
  acc_filter.set_parameter(0.005, 0.0025);

  raw_ax_filter.set_parameter(0.003, 0.0025);
  raw_ay_filter.set_parameter(0.003, 0.0025);
  raw_az_filter.set_parameter(0.003, 0.0025);
  
  raw_gx_filter.set_parameter(0.003, 0.0025);
  raw_gy_filter.set_parameter(0.003, 0.0025);
  raw_gz_filter.set_parameter(0.003, 0.0025);

  raw_az_d_filter.set_parameter(0.1, 0.0025);//alt158
  az_filter.set_parameter(0.1, 0.0025);//alt158
  alt_filter.set_parameter(0.01, 0.0025);
  
}

float sensor_read(void)
{
  float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
  float filterd_v;
  static float dp, dq, dr; 
  static uint16_t dcnt=0u;
  static uint16_t dist=0;
  int16_t deff;
  static uint16_t old_dist[4] = {0};
  static float alt_time = 0.0f;
  static float sensor_time = 0.0f;
  static float old_alt_time = 0.0f;
  static uint8_t first_flag = 0;
  const uint8_t interval = 400/30+1;
  float old_sensor_time = 0.0;
  uint32_t st;
  float sens_interval;
  float h;
  static float opt_interval =0.0;

  st = micros();
  old_sensor_time = sensor_time;
  sensor_time = (float)st*1.0e-6;
  sens_interval = sensor_time - old_sensor_time;
  opt_interval = opt_interval + sens_interval;

  //以下では航空工学の座標軸の取り方に従って
  //X軸：前後（前が正）左肩上がりが回転の正
  //Y軸：右左（右が正）頭上げが回転の正
  //Z軸：下上（下が正）右回りが回転の正
  //となる様に軸の変換を施しています
  //BMI270の座標軸の撮り方は
  //X軸：右左（右が正）頭上げが回転の正
  //Y軸：前後（前が正）左肩上がりが回転の正
  //Z軸：上下（上が正）左回りが回転の正

  //IMUセンサーの値を更新
  imu_update();//IMUの値を読む前に必ず実行、

  //acc -> 加速度 (Acceleration)
  acc_x = imu_get_acc_x();
  acc_y = imu_get_acc_y();
  acc_z = imu_get_acc_z();

  //gyro -> 角速度 (Gyroscope / Angular Velocity)
  gyro_x = imu_get_gyro_x();
  gyro_y = imu_get_gyro_y();
  gyro_z = imu_get_gyro_z();

  //USBSerial.printf("%9.6f %9.6f %9.6f\n\r", Elapsed_time, sens_interval, acc_z);

  //IMUの座標を航空座標系に変換
  Accel_x_raw =  acc_y;
  Accel_y_raw =  acc_x;
  Accel_z_raw = -acc_z;
  //ジャイロデータも適切な座標に変換
  Roll_rate_raw  =  gyro_y;
  Pitch_rate_raw =  gyro_x;
  Yaw_rate_raw   = -gyro_z;

  if(Mode > AVERAGE_MODE)
  {
    //それぞれの軸の加速度のフィルタリング
    Accel_x    = raw_ax_filter.update(Accel_x_raw, Interval_time);
    Accel_y    = raw_ay_filter.update(Accel_y_raw , Interval_time);
    Accel_z    = raw_az_filter.update(Accel_z_raw , Interval_time);
    Accel_z_d  = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);//Z軸の変化量を計算.Accel_z_offset を引くことで、センサーの初期誤差を取り除き、純粋な高度変化だけを取得 できる

    //それぞれのジャイロ(角速度)データをフィルタリング
    Roll_rate  = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, Interval_time);
    Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, Interval_time);
    Yaw_rate   = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, Interval_time);

    //Madgwickを更新し、IMUのデータから姿勢を推定
    Drone_ahrs.updateIMU( (Pitch_rate)*(float)RAD_TO_DEG, 
                          (Roll_rate)*(float)RAD_TO_DEG,
                         -(Yaw_rate)*(float)RAD_TO_DEG,
                            Accel_y, Accel_x, -Accel_z);
    //ドローンの現在のロール、ピッチ、ヨー角度を取得し、ラジアンに変換
    Roll_angle  =  Drone_ahrs.getPitch()*(float)DEG_TO_RAD;
    Pitch_angle =  Drone_ahrs.getRoll()*(float)DEG_TO_RAD;
    Yaw_angle   = -Drone_ahrs.getYaw()*(float)DEG_TO_RAD;


    //デバッグ用
    //USBSerial.printf("%6.3f %7.4f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n\r", 
    //  Elapsed_time, Interval_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

    //加速度のZ軸データをフィルタリング (30Hz)
    Az = az_filter.update(-Accel_z_d, sens_interval);

    //ToFセンサーを使用し、高度の測定とノイズ除去を行う
    if (dcnt>interval)//dcnt（カウント） の値が interval（間隔） を超えたときに処理を実行
    {
      if(ToF_bottom_data_ready_flag)//tof bottom flgが1の場合起動
      {
        //micros() で現在の時刻を取得し、前回の測定時刻 old_alt_time と比較して、測定間隔 h を算出
        dcnt=0u;
        old_alt_time = alt_time;
        alt_time = micros()*1.0e-6;
        h = alt_time - old_alt_time;
        ToF_bottom_data_ready_flag = 0;

        //距離の値の更新
        //old_dist[0] = dist;
        dist=tof_bottom_get_range();
                
        //外れ値処理。toFセンサーのデータに突然の急激な変化（ノイズや外れ値）がある場合、それを補正
        deff = dist - old_dist[1];
        if ( deff > 500 )//deffが 500mm以上の変化なら、前回の値を基に補正して異常値を抑える
        {
          dist = old_dist[1] + (old_dist[1] - old_dist[2])/2;
        }
        else if ( deff < -500 )//deffが -500mm以下の変化なら、同様に補正
        {
          dist = old_dist[1] + (old_dist[1] - old_dist[3])/2;
        }
        else//問題なければ old_dist に値を保存して、次回の測定に備える
        {
          old_dist[3] = old_dist[2];
          old_dist[2] = old_dist[1];
          old_dist[1] = dist;
        }
        //USBSerial.printf("%9.6f, %9.6f, %9.6f, %9.6f, %9.6f\r\n",Elapsed_time,Altitude/1000.0,  Altitude2, Alt_velocity,-(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));
      }
    }
    else dcnt++;//dcnt を増加させて、次の測定タイミングを管理

    //高度データをフィルタリングして更新
    Altitude = alt_filter.update((float)dist/1000.0, Interval_time);

    //Alt_control_ok = 1;
    if(first_flag == 1) EstimatedAltitude.update(Altitude, Az, Interval_time);//first_flag == 1 のとき、高度 (Altitude) と加速度 (Az) のデータをカルマンフィルタに入力
    else first_flag = 1;//フラグを1にセットして次回起動するようにする
    //推定された高度・垂直速度・加速度のバイアスを変数に格納
    Altitude2 = EstimatedAltitude.Altitude;//ToFセンサーの値よりノイズが少なく、より正確な高さを得られる
    Alt_velocity = EstimatedAltitude.Velocity;//高度の変化率（上昇・下降の速度）
    Az_bias = EstimatedAltitude.Bias;//加速度センサの誤差（オフセット）を補正
    //USBSerial.printf("Sens=%f Az=%f Altitude=%f Velocity=%f Bias=%f\n\r",Altitude, Az, Altitude2, Alt_velocity, Az_bias);

    //float Roll_angle = Roll_angle;
    //float tht = Pitch_angle;
    //float Yaw_angle = Yaw_angle;
    //float sRoll_angle = sin(Roll_angle);
    //float cRoll = cos(Roll_angle);
  // float stht = sin(tht);
    //float cPitch = cos(Pitch_angle);
    //float sYaw_angle = sin(Yaw_angle);
    //float sYaw_angle = cos(Yaw_angle);

    //float r33 =  cRoll*cPitch;
    //Altitude2 = r33 * Altitude;
    //EstimatedAltitude.update(Altitude2, r33*Accel_z_raw)

  }

  //Accel fail safe
  acc_norm = sqrt(Accel_x*Accel_x + Accel_y*Accel_y + Accel_z_d*Accel_z_d);//加速度の大きさ（ベクトルのノルム）を計算.3軸の加速度を統合し、ドローンがどれくらい強く加速しているかを算出
  Acc_norm = acc_filter.update(acc_norm ,Control_period);//加速度の値をフィルタリングし、ノイズを減らす.フィルタを適用して値を安定化することで より正確な過剰加速判定 ができる
  if (Acc_norm>2.0)//フィルタ後の加速度が「2.0G」以上なら、過剰加速と判定 .「2.0G以上」は、ドローンの物理的な耐久性を考慮した制限値
  {
    OverG_flag = 1;
    if (Over_g == 0.0)Over_g = acc_norm;//もし Over_g がまだ設定されていなければ、現在の acc_norm を保存
  }

  //INA3221を使いバッテリー電圧を確認 
  Voltage = ina3221.getVoltage(INA3221_CH2);//バッテリーの電圧を取得,値を格納
  filterd_v = voltage_filter.update(Voltage, Control_period);//電圧のノイズ除去。電圧測定値には 一時的なノイズ や 急変動 が含まれるため、フィルタを適用して より正確な値 にする。

  if(Under_voltage_flag != UNDER_VOLTAGE_COUNT)//Under_voltage_flag は バッテリー低電圧を検知するフラグ
  {
    if (filterd_v < POWER_LIMIT) Under_voltage_flag ++;//filterd_v（フィルタ後の電圧）が POWER_LIMIT（定められた閾値）より低い場合 → 低電圧フラグを増加。バッテリー電圧が 一瞬だけ 低くなることがあるため、連続して閾値を超えた場合のみ低電圧と判断 する。
    else Under_voltage_flag = 0;//filterd_v が POWER_LIMIT 以上なら → 低電圧フラグをリセット
    if ( Under_voltage_flag > UNDER_VOLTAGE_COUNT) Under_voltage_flag = UNDER_VOLTAGE_COUNT;//Under_voltage_flag が UNDER_VOLTAGE_COUNT を超えた場合、最大値に制限（フラグが際限なく増えないようにする）。一定回数 POWER_LIMIT を下回ると、ドローンは低電圧警告を発する
  }
  /*
  if (opt_interval > 0.1)
  {
    opt_interval = 0.0;
    flow.readMotionCount(&deltaX, &deltaY);
    USBSerial.printf("%f %d %d %f\r\n", Elapsed_time, deltaX, deltaY, Accel_z_raw);
  }
  */

  uint32_t et =micros();//現在の処理時間（マイクロ秒単位）を取得
  //USBSerial.printf("Sensor read %f %f %f\n\r", (mt-st)*1.0e-6, (et-mt)*1e-6, (et-st)*1.0e-6);

  return (et-st)*1.0e-6;//開始時間 (st) からの経過時間（マイクロ秒） を計算。1.0e-6 を掛けることで、秒単位（浮動小数点）に変換
}


#if 0

//初期で仮距離を1mと仮定し宣言
float range = 1.0f;

//姿勢角の初期化
float Roll_angle = 0.0f;
float tht = 0.0f;
float Yaw_angle = 0.0f;

//角度からサインとコサインを求める.これは回転行列を作るために必要な値
float sRoll_angle = sin(Roll_angle);
float cRoll_angle = cos(Roll_angle);
float stht = sin(tht);
float ctht = cos(tht);
float sYaw_angle = sin(Yaw_angle);
float sYaw_angle = cos(Yaw_angle);//多分うちミス？でsYawになっている。もしかしたらエラーが起きる可能性もある部分

//回転行列の構成(1行目:x軸)
float r11 =  ctht*cYaw_angle;                                          //r11 → ピッチとヨーの影響を考慮した X軸の変換
float r12 =  sRoll_angle*stht*cYaw_angle - cRoll_angle*sYaw_angle;     //r12, r13 → ロールの影響を加味した変換
float r13 =  cRoll_angle*stht*cYaw_angle + sRoll_angle*sYaw_angle;

//回転行列の構成(2行目:y軸)
float r21 =  ctht*sYaw_angle;
float r22 =  sRoll_angle*stht*sYaw_angle + cRoll_angle*cYaw_angle;
float r23 =  cRoll_angle*stht*sYaw_angle - sRoll_angle*cYaw_angle;

//回転行列の構成(3行目:z軸)
float r31 = -stht;
float r32 =  sRoll_angle*ctht;
float r33 =  cRoll_angle*ctht;

//回転後の座標計算
float x = r13*range;//元の座標系で表された range の位置を、回転後の座標系での x 座標に変換
float y = r23*range;//元の座標系での range の位置を、回転後の座標系での y 座標に変換
float z = r33*range;//元の座標系での range の位置を、回転後の座標系での z 座標に変換
#endif
