/*(cpp)このコードは、マルチコプターの飛行を管理し、センサーの読み取り、PID制御、モータードライバ操作、モード変更、飛行安定化などを実現する制御システムを構築しています。

(hpp)このヘッダーファイルは、マルチコプターの制御ロジックや状態変数を宣言し、他のファイルで使用できるようにしています。PID制御や各モーターの操作、高度・角度制御を円滑に行うために必要な基盤を提供します。

ドローンやRCヘリコプターなどの安定飛行制御。

各種センサー（IMU、バロメーターなど）を用いた高度な制御ロジックの実装。
*/
//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//

#include "flight_control.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"

//モータPWM出力Pinのアサイン
//Motor PWM Pin
const int pwmFrontLeft  = 5;//左前のモーター
const int pwmFrontRight = 42;//右前
const int pwmRearLeft   = 10;//左後ろ
const int pwmRearRight  = 41;//右後ろ

//モータPWM周波数 
//Motor PWM Frequency
const int freq = 150000;// PWMの周波数は 150kHz

//PWM分解能
//PWM Resolution
const int resolution = 8;// PWMの分解能は 8ビット

//モータチャンネルのアサイン   4つのモーターにそれぞれ異なるチャンネルを割り当てている
//Motor Channel
const int FrontLeft_motor  = 0;
const int FrontRight_motor = 1;
const int RearLeft_motor   = 2;
const int RearRight_motor  = 3;

//制御周期 、これにより制御周期（1回の処理にかかる時間） が 0.0025秒（2.5ミリ秒） となり、1秒間に400回の計算が実行 されることになる
//Control period
float Control_period = 0.0025f;//400Hz

//PID Gain　　ロール（横回転）　　ピッチ（前後回転）　　ヨー（機体を上からみたときの回転方向の傾き）
//Rate control PID gain　角速度制御
//p（比例）i（積分）d（微分）
const float Roll_rate_kp = 0.6f;//横回転の速さを制御　　kp:比例ゲイン
const float Roll_rate_ti = 0.7f;//積分時間
const float Roll_rate_td = 0.01;//微分時間
const float Roll_rate_eta = 0.125f;//微分フィルタ係数　ノイズの影響を減らす（安定さ向上）

const float Pitch_rate_kp = 0.75f;//前後回転の速さ
const float Pitch_rate_ti = 0.7f;
const float Pitch_rate_td = 0.025f;
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp = 3.0f;//左右の向き
const float Yaw_rate_ti = 0.8f;
const float Yaw_rate_td = 0.01f;
const float Yaw_rate_eta = 0.125f;

//Angle control PID gain　角度制御
const float Rall_angle_kp = 8.0f;
const float Rall_angle_ti = 4.0f;
const float Rall_angle_td = 0.04f;
const float Rall_angle_eta = 0.125f;

const float Pitch_angle_kp = 8.0f;
const float Pitch_angle_ti = 4.0f;
const float Pitch_angle_td = 0.04f;
const float Pitch_angle_eta = 0.125f;

//Altitude control PID gain　高度制御
const float alt_kp = 0.65f;
const float alt_ti = 200.0f;
const float alt_td = 0.0f;
const float alt_eta = 0.125f;
const float alt_period = 0.0333;

const float Thrust0_nominal = 0.63;
const float z_dot_kp = 0.15f;
const float z_dot_ti = 13.5f;
const float z_dot_td = 0.005f;
const float z_dot_eta = 0.125f;

//Times　　時間経過を記録し、制御の正確性を確保するための変数群です。前の処理からどのくらいの時間がたったかを記録
volatile float Elapsed_time=0.0f;
volatile float Old_Elapsed_time=0.0f;
volatile float Interval_time=0.0f;
volatile uint32_t S_time=0,E_time=0,D_time=0,Dt_time=0;

//Counter　ドローンの飛行制御での状態管理や処理回数を記録する
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t OffsetCounter=0;

//Motor Duty 四つのモーターにどれだけの回転支持を出すか記録する変数（0.0~1.0（０～１００％））
volatile float FrontRight_motor_duty=0.0f;
volatile float FrontLeft_motor_duty=0.0f;
volatile float RearRight_motor_duty=0.0f;
volatile float RearLeft_motor_duty=0.0f;

//ユーザーの操作（ラジコンのスティック）に応じて変更される指令値。
//制御目標
//PID Control reference
//角速度目標値
//Rate reference
volatile float Roll_rate_reference=0.0f, Pitch_rate_reference=0.0f, Yaw_rate_reference=0.0f;
//角度目標値
//Angle reference
volatile float Roll_angle_reference=0.0f, Pitch_angle_reference=0.0f, Yaw_angle_reference=0.0f;
//舵角指令値
//Commanad
//スロットル指令値
//Throttle
volatile float Thrust_command=0.0f, Thrust_command2 = 0.0f;
//角速度指令値
//Rate command
volatile float Roll_rate_command=0.0f, Pitch_rate_command=0.0f, Yaw_rate_command=0.0f;
//角度指令値
//Angle comannd
volatile float Roll_angle_command=0.0f, Pitch_angle_command=0.0f, Yaw_angle_command=0.0f;

//Offset　ドローンの初期設定や調整を行うための変数を宣言
volatile float Roll_angle_offset=0.0f, Pitch_angle_offset=0.0f, Yaw_angle_offset=0.0f; //角度のオフセット ドローンの初期姿勢の補正
volatile float Elevator_center=0.0f, Aileron_center=0.0f, Rudder_center=0.0f;          //操縦スティックのセンサーの位置　スティックのゼロ位置補正

//Machine state & flag　ドローンの状態やフラグ（状況を示す）変数を管理
float Timevalue=0.0f;               //プログラムが開始してからの時間経過を記録する変数
uint8_t Mode = INIT_MODE;           //初期化モード
uint8_t Control_mode = ANGLECONTROL;//角度制御モード
//フラグ変数
volatile uint8_t LockMode=0;                 // ドローンの操作や制御がロックされている状態を示すフラグ
float Motor_on_duty_threshold = 0.1f;        //モーターが動作を開始するための最低出力値の閾値(いきち)です。この値を超えるとモーターが動き始めます。
float Angle_control_on_duty_threshold = 0.5f;//角度制御を有効にするためのモーターの出力閾値
int8_t BtnA_counter = 0;                     //ボタンAが押された回数や押されている時間のカウント
uint8_t BtnA_on_flag = 0;                    //ボタンが押された瞬間を検出フラグ （ONになったら1にする）
uint8_t BtnA_off_flag =1;                    // ボタンAが「離された瞬間」を検出するためのフラグ。　初期値が１だから最初は何も押されていない
volatile uint8_t Loop_flag = 0;              // メインループの中で、処理のタイミングを制御するためのフラグ。
volatile uint8_t Angle_control_flag = 0;     //ドローンの水平維持や自動姿勢制御をON/OFF。
uint8_t Stick_return_flag = 0;               //スティックが「元の位置に戻ったこと」を検出するフラグ。
uint8_t Throttle_control_mode = 0;　         // スロットル（出力）の制御モードを示す変数。 

//for flip フリップ（宙返り）動作やIMU（姿勢センサー）のリセット
float FliRoll_rate_time = 2.0;      // フリップ動作の際の「ロール軸の回転にかかる時間」を秒単位で指定
uint8_t Flip_flag = 0;              //フリップ動作を行うかどうかのスイッチ（フラグ）
uint16_t Flip_counter = 0;          // フリップ動作のカウンター。処理の中で時間やステップを計測するために使われる。
float Flip_time = 2.0;              //フリップにかける全体の時間（秒）
volatile uint8_t Ahrs_reset_flag=0; // 	IMUの再初期化の指示  AHRS（姿勢推定：Attitude and Heading Reference System）のリセット指示フラグ。
float T_flip;                       //フリップに関する「経過時間」や「開始時刻」を記録するための変数。

//PID object and etc.
PID p_pid; 	   //角速度 p  ロール
PID q_pid; 	   //角速度 q  ピッチ
PID r_pid; 	   //角速度 r  	ヨー
PID phi_pid; 	 //角度 φ  ロール
PID theta_pid; //角度 θ  ピッチ
PID psi_pid; 	 //角度 ψ  	ヨー
//PID alt;
PID alt_pid; //高度
PID z_dot_pid; //上昇/下降速度
Filter Thrust_filtered; //推力	センサーや計算結果の推力値を平滑化
Filter Duty_fr; //	右前モーター	モーターPWM信号のフィルター
Filter Duty_fl; //	左前モーター
Filter Duty_rr; //  右後モーター
Filter Duty_rl; //  左後モーター

//ドローンなどの機体の推力（Thrust）や高度（Altitude）に関する制御
volatile float Thrust0=0.0;  //ベース推力値（高度維持などに使用）
uint8_t Alt_flag = 0;        //	高度制御を行うかのON/OFFフラグ
float Alt_max = 0.5;         //	最大高度のしきい値（例：0.5m）

//速度目標Z
float Z_dot_ref = 0.0f;  //上昇・下降速度を設定（例：0.2 = 上昇） m/s

//高度目標
const float Alt_ref_min = 0.3; //	許容される最低高度
volatile float Alt_ref = 0.5;  // 現在の目標高度  高度制御の目標値（PIDがこの高さに合わせる）

//Function declaration 関数の宣言（プロトタイプ）一覧
void init_pwm();                   //PWM（Pulse Width Modulation）制御の初期化
void control_init();               //PID制御や各種制御用変数・構造体の初期化
void variable_init(void);          //使用している全体の変数を初期値にリセットします
void get_command(void);            //リモコンやシリアルなどからの操作コマンドを受信・処理する関数
void angle_control(void);          //姿勢（角度：ロール・ピッチ・ヨー）をPIDで制御
void rate_control(void);           //角速度（回転の速さ）をPIDで制御
void output_data(void);            //制御結果や状態を外部（PCやログ）に送信
void output_sensor_raw_data(void); //IMUなどのセンサーからの生データをそのまま出力
void motor_stop(void);             //モーター出力を完全に停止
uint8_t judge_mode_change(void);   //モード切り替え（例：マニュアル／自動、ホバリング／フリップ）の判定
uint8_t get_arming_button(void);   //モーター起動（arming）ボタンの状態を取得
uint8_t get_flip_button(void);     //フリップ動作（宙返り）用のボタン状態を取得
void reset_rate_control(void);     //角速度制御（rate PID）の積分項などをリセット
void reset_angle_control(void);    //角度制御に使う積分項や誤差記録などをリセットする

//割り込み関数 タイマー割り込み処理  リアルタイムで処理を繰り返すための重要な仕組み
//一定間隔で「Loop_flag = 1」をセットすることで、メインループ側に「今処理していいよ」と合図する。
//Intrupt function                         全体像  
hw_timer_t * timer = NULL;        //  タイマー（例：100Hzなど）が定期的に onTimer() を呼ぶ。
void IRAM_ATTR onTimer()          //  Loop_flag を 1 にする。
{                                 //  メインループ内でこのフラグが 1 になったら、1回だけ制御処理を行う。
  Loop_flag = 1;                  //  Loop_flag を 0 に戻して、次のタイマー割り込みを待つ。
}

//Initialize Multi copter　　マルチコプター（ドローン）の初期化処理
void init_copter(void)　　　　//init_copter()：マイコンやドローンを電源オンしたときに実行される初期設定関数
{
  //Initialize Mode
  Mode = INIT_MODE;

  //Initialaze LED function      LED初期化と点灯（視覚的に状態表示）
  led_init();　　　　　　　　　　　//： LED制御の準備
  esp_led(0x110000, 1);　　　　　 //： これは RGB LED に赤っぽい色を表示する処理（0x110000 は暗めの赤）。
  onboard_led1(WHITE, 1);　　　　 //： 機体に搭載されたLEDを白色に点灯
  onboard_led2(WHITE, 1);
  led_show();　　　　　　　　　　　//： LEDの設定を反映
 
  //Initialize Serial communication
  USBSerial.begin(115200);                    // **USBシリアル通信の初期化**（115200bps）
  delay(1500);                                // 通信安定のために 1.5 秒待機
  USBSerial.printf("Start StampS3FPV!\r\n");  // 起動メッセージをPC側に送信（デバッグや確認用）。
  
  //Initialize PWM
  init_pwm();                                    // ： モーター制御用のPWM出力を初期化
  sensor_init();                                //  ： IMUや気圧センサなどの初期化
  USBSerial.printf("Finish sensor init!\r\n");    //   初期化完了メッセージを表示

  //PID GAIN and etc. Init       //  **PID制御器（角度・角速度・高度など）の初期化処理**
  control_init();               //     PIDゲインの設定や内部変数のリセットが行われるはずです

  //Initilize Radio control
  rc_init();                     //    リモコン（RC）受信機の初期化    通信の準備やキャリブレーションが含まれる可能性あり

  //割り込み設定
  //Initialize intrupt                         // **タイマーのセットアップ**（ESP32などの環境を想定）
  timer = timerBegin(0, 80, true);            //    タイマー0、分周値80（= 1us単位）、カウントアップ
  timerAttachInterrupt(timer, &onTimer, true);//    割り込み関数 `onTimer()` を登録
  timerAlarmWrite(timer, 2500, true);         //  **2.5msごと（= 400Hz）** に割り込み
  timerAlarmEnable(timer);                      //  タイマー割り込み開始         400Hzの制御ループが実現されていると推測できます
  USBSerial.printf("Finish StampFly init!\r\n"); //  初期化完了メッセージと、「Enjoy Flight!」というフライト開始の合図。
  USBSerial.printf("Enjoy Flight!\r\n");
}

//Main loop     ドローンの制御ループ（メインループ）を400Hzで実行するための中心的な処理uj
void loop_400Hz(void)
{
  static uint8_t led=1;
  float sense_time;
  //割り込みにより400Hzで以降のコードが実行
  while(Loop_flag==0);  //タイマー割り込みによって Loop_flag == 1 になるまで待つ
  Loop_flag = 0;        //フラグが立ったらすぐ処理開始、再び 0 に戻して次の割り込み待機

  //時間の計測と経過時間の記録
  E_time = micros();                                    //  現在のマイクロ秒（μs）単位の時刻を取得している
  Old_Elapsed_time = Elapsed_time;　　　　　　　　　　　　//  前回の経過時間を保存
  Elapsed_time = 1e-6*(E_time - S_time);                //  経過時間（秒単位）を計算   1e-6 をかけて 秒に変換しています
  Interval_time = Elapsed_time - Old_Elapsed_time;      //  今回と前回の Elapsed_time の差を取ることで、**今回ループ内での経過時間（インターバル）**を計算
  Timevalue+=0.0025f;                                   //  Timevalue を毎回 0.0025（秒）だけ加算しています
  
  //Read Sensor Value  センサーの値を読み取る処理
  sense_time = sensor_read();      //sensor_read() 関数を呼び出して、何らかのセンサーからの値を取得
  uint32_t cs_time = micros();    // micros() で現在の時刻（スケッチ開始からのマイクロ秒）を取得し、それを cs_time に格納。

  //LED Drive  LEDの駆動（点灯・消灯・制御）を行う処理
  led_drive();                    // led_drive() という関数を呼び出しています
  
  //Begin Mode select
  if (Mode == INIT_MODE)  // 現在のモードが INIT_MODE（初期化モード）であるかをチェックします
  {
      motor_stop();                //  モーターを停止します
      Elevator_center = 0.0f;       // 各舵面（エレベーター）のニュートラル（中立）位置をリセット
      Aileron_center = 0.0f;        // 各舵面（エルロン、）のニュートラル（中立）位置をリセット
      Rudder_center = 0.0f;         // 各舵面（ラダー）のニュートラル（中立）位置をリセット
      Roll_angle_offset = 0.0f;     // ジャイロまたはIMUの各軸の角度オフセットをリセット
      Pitch_angle_offset = 0.0f;
      Yaw_angle_offset = 0.0f;
      sensor_reset_offset();        // センサーのバイアス（オフセット）をリセットする関数
      Mode = AVERAGE_MODE;          // 初期化が終わったら、モードを AVERAGE_MODE に切り替える
      return;
  }
  else if (Mode == AVERAGE_MODE)    // 現在のモードが AVERAGE_MODE のときにのみ、以下の処理を行います
  {
    motor_stop();                    //モーターを停止します
    //Gyro offset Estimate
    if (OffsetCounter < AVERAGENUM)  //オフセット平均処理を、一定回数（＝AVERAGENUM）繰り返すための条件分岐
    {
      sensor_calc_offset_avarage();  //センサーの現在値を読み、内部的に加算や平均計算
      OffsetCounter++;               //オフセット平均処理を行った回数をカウントアップ
      return;
    }
    //Mode change
    Mode = PARKING_MODE;             //センサーのオフセット（キャリブレーション）が完了したので、次のモード PARKING_MODE に遷移します
    S_time = micros();               //現在の時刻を再記録
    return;
  }
  else if( Mode == FLIGHT_MODE)      //モードが FLIGHT_MODE（飛行モード、制御モード）のときに、機体の制御を開始します  
  {
    Control_period = Interval_time; // 1ループ（1回の制御処理）の経過時間（秒）を記録

    //Judge Mode change
    if (judge_mode_change() == 1) Mode = PARKING_MODE;  //飛行中に何らかの条件を検知して、モードを PARKING_MODE に強制的に戻す処理
    
    //Get command
    get_command(); //操縦者からの入力コマンドを取得

    //Angle Control
    angle_control();// **姿勢制御（角度制御）**を行う関数

    //Rate Control
    rate_control();// 角速度（ジャイロ）を制御する、いわゆる内側の制御ループ（インナーループ）
  }
  else if(Mode == PARKING_MODE) // judge_mode_change() によって、モードを切り替える判断をしています
  {
    //Judge Mode change
    if( judge_mode_change() == 1)Mode = FLIGHT_MODE;
    
    //Parking
    motor_stop();
    OverG_flag = 0;
    Angle_control_flag = 0;
    Thrust0 = 0.0;
    Alt_flag = 0;
    Alt_ref = Alt_ref_min;
    Stick_return_flag = 0;
    Throttle_control_mode = 0;
    Thrust_filtered.reset();
  }

  //// Telemetry
  //telemetry400();
  //telemetry();

  uint32_t ce_time = micros();
  Dt_time = ce_time - cs_time;  
  //End of Loop_400Hz function
}

uint8_t judge_mode_change(void)
{
  //Ariming Button が押されて離されたかを確認
  uint8_t state;
  state = 0;
  if(LockMode == 0)
  {
    if( get_arming_button()==1)
    {
      LockMode = 1;
    }
  }
  else
  {
    if( get_arming_button()==0)
    {
      LockMode = 0;
      state = 1;
    }
  }
  return state;
}

///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//  
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001); 

void control_init(void)
{
  //Rate control
  p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta, Control_period);//Roll rate control gain
  q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta, Control_period);//Pitch rate control gain
  r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);//Yaw rate control gain

  //Angle control
  phi_pid.set_parameter  (Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta, Control_period);//Roll angle control gain
  theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta, Control_period);//Pitch angle control gain

  //Altitude control
  alt_pid.set_parameter(alt_kp, alt_ti, alt_td, alt_eta, alt_period);
  z_dot_pid.set_parameter(z_dot_kp, z_dot_ti, z_dot_td, alt_eta, alt_period);

  Duty_fl.set_parameter(0.003, Control_period);
  Duty_fr.set_parameter(0.003, Control_period);
  Duty_rl.set_parameter(0.003, Control_period);
  Duty_rr.set_parameter(0.003, Control_period);

}
///////////////////////////////////////////////////////////////////

void get_command(void)
{
  static uint16_t stick_count;
  float th,thlo;
  float throttle_limit = 0.7;

  Control_mode = Stick[CONTROLMODE];
  if ( (uint8_t)Stick[ALTCONTROLMODE] == 5)Throttle_control_mode = 0;
  else if((uint8_t)Stick[ALTCONTROLMODE] == 4)Throttle_control_mode = 1;
  else Throttle_control_mode = 0;

  //Thrust control
  thlo = Stick[THROTTLE];
  thlo = thlo/throttle_limit;

  if (Throttle_control_mode == 0)
  {
    //Manual
    if(thlo<0.0)thlo = 0.0;
    if ( (0.2 > thlo) && (thlo > -0.2) )thlo = 0.0f ;//不感帯
    if (thlo>1.0f) thlo = 1.0f;
    if (thlo<-1.0f) thlo =0.0f;
    //Throttle curve conversion　スロットルカーブ補正
    th = (2.97f*thlo-4.94f*thlo*thlo+2.86f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
    Thrust_command = Thrust_filtered.update(th, Interval_time);
  }
  else if (Throttle_control_mode == 1)
  {
    //Altitude Control
    if(Alt_flag==0)
    {
      stick_count = 0;
      //Manual目標高度まではマニュアルで上げる
      if(thlo<0.0)thlo = 0.0;
      if ( (0.2 > thlo) && (thlo > -0.2) )thlo = 0.0f ;
      if (thlo>1.0f) thlo = 1.0f;
      if (thlo<-1.0f) thlo =0.0f;
      th = (2.97f*thlo-4.94f*thlo*thlo+2.86f*thlo*thlo*thlo)*BATTERY_VOLTAGE;
      Thrust_command = Thrust_filtered.update(th, Interval_time);
      
      if (Altitude2 < Alt_ref) 
      {
        Thrust0 = Thrust_command / BATTERY_VOLTAGE;
        alt_pid.reset();
        z_dot_pid.reset();
      }
      else Alt_flag = 1; 
    }
    else
    {
      if(Stick_return_flag == 0)
      {
        if ( (-0.2 < thlo) && (thlo < 0.2) )
        {
          thlo = 0.0f ;//不感帯
          stick_count++;
          if(stick_count>200)Stick_return_flag = 1;
        }
      }
      else
      {
        if ( (-0.2 < thlo) && (thlo < 0.2) )thlo = 0.0f ;//不感帯
        Alt_ref = Alt_ref + thlo*0.001;
        if(Alt_ref<0.05)Alt_ref=0.05;
      }
    } 
  }

  Roll_angle_command = 0.4*Stick[AILERON];
  if (Roll_angle_command<-1.0f)Roll_angle_command = -1.0f;
  if (Roll_angle_command> 1.0f)Roll_angle_command =  1.0f;  
  Pitch_angle_command = 0.4*Stick[ELEVATOR];
  if (Pitch_angle_command<-1.0f)Pitch_angle_command = -1.0f;
  if (Pitch_angle_command> 1.0f)Pitch_angle_command =  1.0f;  

  Yaw_angle_command = Stick[RUDDER];
  if (Yaw_angle_command<-1.0f)Yaw_angle_command = -1.0f;
  if (Yaw_angle_command> 1.0f)Yaw_angle_command =  1.0f;  
  //Yaw control
  Yaw_rate_reference   = 2.0f * PI * (Yaw_angle_command - Rudder_center);

  if (Control_mode == RATECONTROL)
  {
    Roll_rate_reference = 240*PI/180*Roll_angle_command;
    Pitch_rate_reference = 240*PI/180*Pitch_angle_command;
  }

  // flip button check
  if (Flip_flag == 0 && Throttle_control_mode == 0)
  {
    Flip_flag = get_flip_button();
  }
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err, z_dot_err;

  //Control main
  if(rc_isconnected())
  {
    if(Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold)
    { 
      reset_rate_control();
    }
    else
    {
      //Control angle velocity
      p_rate = Roll_rate;
      q_rate = Pitch_rate;
      r_rate = Yaw_rate;

      //Get reference
      p_ref = Roll_rate_reference;
      q_ref = Pitch_rate_reference;
      r_ref = Yaw_rate_reference;

      //Error
      p_err = p_ref - p_rate;
      q_err = q_ref - q_rate;
      r_err = r_ref - r_rate;
      z_dot_err = Z_dot_ref - Alt_velocity;
      
      //Rate Control PID
      Roll_rate_command = p_pid.update(p_err, Interval_time);
      Pitch_rate_command = q_pid.update(q_err, Interval_time);
      Yaw_rate_command = r_pid.update(r_err, Interval_time);
      if (Alt_flag == 1)
      {
        Thrust_command = (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time))*BATTERY_VOLTAGE;
      }

      //Motor Control
      //正規化Duty
      FrontRight_motor_duty = Duty_fr.update((Thrust_command +(-Roll_rate_command +Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE, Interval_time);
      FrontLeft_motor_duty  = Duty_fl.update((Thrust_command +( Roll_rate_command +Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE, Interval_time);
      RearRight_motor_duty  = Duty_rr.update((Thrust_command +(-Roll_rate_command -Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE, Interval_time);
      RearLeft_motor_duty   = Duty_rl.update((Thrust_command +( Roll_rate_command -Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE, Interval_time);
    
      const float minimum_duty=0.0f;
      const float maximum_duty=0.95f;

      if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
      if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;

      if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
      if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;

      if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
      if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;

      if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
      if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;

      //Duty set
      if (OverG_flag==0){
        set_duty_fr(FrontRight_motor_duty);
        set_duty_fl(FrontLeft_motor_duty);
        set_duty_rr(RearRight_motor_duty);
        set_duty_rl(RearLeft_motor_duty);      
      }
      else 
      {
        FrontRight_motor_duty = 0.0;
        FrontLeft_motor_duty = 0.0;
        RearRight_motor_duty = 0.0;
        RearLeft_motor_duty = 0.0;
        motor_stop();
        OverG_flag=0;
        Mode = PARKING_MODE;
      }
    }
  }
  else
  {
    reset_rate_control();
  }
}

void reset_rate_control(void)
{
    motor_stop();
    FrontRight_motor_duty = 0.0;
    FrontLeft_motor_duty = 0.0;
    RearRight_motor_duty = 0.0;
    RearLeft_motor_duty = 0.0;
    Duty_fr.reset();
    Duty_fl.reset();
    Duty_rr.reset();
    Duty_rl.reset();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    Roll_rate_reference = 0.0f;
    Pitch_rate_reference = 0.0f;
    Yaw_rate_reference = 0.0f;
    Rudder_center   = Yaw_angle_command;
    //angle control value reset
    Roll_rate_reference=0.0f;
    Pitch_rate_reference=0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Flip_flag = 0;
    Flip_counter = 0;
    Roll_angle_offset   = 0;
    Pitch_angle_offset = 0;
}

void reset_angle_control(void)
{
    Roll_rate_reference=0.0f;
    Pitch_rate_reference=0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Flip_flag = 0;
    Flip_counter = 0;
    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    Aileron_center  = Roll_angle_command;
    Elevator_center = Pitch_angle_command;
    Roll_angle_offset   = 0;
    Pitch_angle_offset = 0;
    /////////////////////////////////////
}

void angle_control(void)
{
  float phi_err, theta_err, alt_err;
  static uint8_t cnt=0;
  static float timeval=0.0f;
  //flip
  uint16_t flip_delay = 150; 
  uint16_t flip_step;
  float domega;

  if (Control_mode == RATECONTROL) return;

  //PID Control
  if ((Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold))//Angle_control_on_duty_threshold))
  {
    //Initialize
    reset_angle_control();
  }
  else
  {
    //Flip
    if (Flip_flag == 1)
    { 
      Led_color = FLIPCOLOR;

      //PID Reset
      phi_pid.reset();
      theta_pid.reset();
    
      //Flip
      Flip_time = 0.4;
      Pitch_rate_reference= 0.0;
      domega = 0.00225f*8.0*PI/Flip_time/Flip_time;//25->22->23->225
      flip_delay = 150;
      flip_step = (uint16_t)(Flip_time/0.0025f);
      if (Flip_counter < flip_delay)
      {
        Roll_rate_reference = 0.0f;
        Thrust_command = T_flip*1.2;
      }
      else if (Flip_counter < (flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference + domega;
        Thrust_command = T_flip*1.05;
      }
      else if (Flip_counter < (2*flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference + domega;
        Thrust_command = T_flip*1.0;
      }
      else if (Flip_counter < (3*flip_step/4 + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference - domega;
        Thrust_command = T_flip*1.0;
      }
      else if (Flip_counter < (flip_step + flip_delay))
      {
        Roll_rate_reference = Roll_rate_reference - domega;
        Thrust_command = T_flip*1.4;
      }
      else if (Flip_counter < (flip_step + flip_delay + 120) )
      {
        if(Ahrs_reset_flag == 0) 
        {
          Ahrs_reset_flag = 1;
          ahrs_reset();
        }
        Roll_rate_reference = 0.0;
        Thrust_command=T_flip*1.4;
      }
      else
      {
        Flip_flag = 0;
        Ahrs_reset_flag = 0;
      }
      Flip_counter++;
    }
    else
    {
      //flip reset
      //Flip_flag = 1;
      Roll_rate_reference = 0;
      T_flip = Thrust_command;
      Ahrs_reset_flag = 0;
      Flip_counter = 0;

      //Angle Control
      Led_color = RED;
      //Get Roll and Pitch angle ref 
      Roll_angle_reference  = 0.5f * PI * (Roll_angle_command - Aileron_center);
      Pitch_angle_reference = 0.5f * PI * (Pitch_angle_command - Elevator_center);
      if (Roll_angle_reference > (30.0f*PI/180.0f) ) Roll_angle_reference = 30.0f*PI/180.0f;
      if (Roll_angle_reference <-(30.0f*PI/180.0f) ) Roll_angle_reference =-30.0f*PI/180.0f;
      if (Pitch_angle_reference > (30.0f*PI/180.0f) ) Pitch_angle_reference = 30.0f*PI/180.0f;
      if (Pitch_angle_reference <-(30.0f*PI/180.0f) ) Pitch_angle_reference =-30.0f*PI/180.0f;

      //Error
      phi_err   = Roll_angle_reference   - (Roll_angle - Roll_angle_offset );
      theta_err = Pitch_angle_reference - (Pitch_angle - Pitch_angle_offset);
      alt_err = Alt_ref - Altitude2;

      //Altitude COntrol PID
      Roll_rate_reference = phi_pid.update(phi_err, Interval_time);
      Pitch_rate_reference = theta_pid.update(theta_err, Interval_time);
      if(Alt_flag==1)Z_dot_ref = alt_pid.update(alt_err, Interval_time);
      
    } 
  }
}

void set_duty_fr(float duty){ledcWrite(FrontRight_motor, (uint32_t)(255*duty));}
void set_duty_fl(float duty){ledcWrite(FrontLeft_motor, (uint32_t)(255*duty));}
void set_duty_rr(float duty){ledcWrite(RearRight_motor, (uint32_t)(255*duty));}
void set_duty_rl(float duty){ledcWrite(RearLeft_motor, (uint32_t)(255*duty));}

void init_pwm(void)
{
  ledcSetup(FrontLeft_motor, freq, resolution);
  ledcSetup(FrontRight_motor, freq, resolution);
  ledcSetup(RearLeft_motor, freq, resolution);
  ledcSetup(RearRight_motor, freq, resolution);
  ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
  ledcAttachPin(pwmFrontRight, FrontRight_motor);
  ledcAttachPin(pwmRearLeft, RearLeft_motor);
  ledcAttachPin(pwmRearRight, RearRight_motor);
}

uint8_t get_arming_button(void)
{
  static int8_t chatta=0;
  static uint8_t state=0;
  if( (int)Stick[BUTTON_ARM] == 1 )
  { 
    chatta++;
    if(chatta>10)
    {
      chatta=10;
      state=1;
    }
  }
  else
  {
    chatta--;
    if(chatta<-10)
    {    
      chatta=-10;
      state=0;
    }
  }
  return state;
}

uint8_t get_flip_button(void)
{
  static int8_t chatta=0;
  static uint8_t state=0;
  if( (int)Stick[BUTTON_FLIP] == 1 )
  { 
    chatta++;
    if(chatta>10)
    {
      chatta=10;
      state=1;
    }
  }
  else
  {
    chatta--;
    if(chatta<-10)
    {    
      chatta=-10;
      state=0;
    }
  }
  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}
