/*ドローンやマルチコプターのテレメトリー（遠隔監視）データを管理・送信するためのモジュールを構成しています。センサーや制御状態のデータを収集し、一定の形式に加工して送信します。




・利点
柔軟なデータ送信:
ヘッダーデータ、通常データ、高速データを用途に応じて切り替え可能。

汎用性:
任意のセンサー情報や制御状態を送信可能。

効率性:
通信頻度やパケットサイズを調整し、通信の効率を最適化。
*/

#include "telemetry.hpp"

#include "rc.hpp"
#include "led.hpp"
#include "sensor.hpp"
#include "flight_control.hpp"

uint8_t Telem_mode = 0;
uint8_t Telem_cnt = 0;
const uint8_t MAXINDEX=110;
const uint8_t MININDEX=14;

void telemetry_sequence(void);
void telemetry_sequence400(void);
void make_telemetry_header_data(uint8_t* senddata);
void make_telemetry_data(uint8_t* senddata);
void make_telemetry_data400(uint8_t* senddata);
void data2log(uint8_t* data_list, float add_data, uint8_t index);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len);
void data_set(uint8_t* datalist, float value, uint8_t* index);

void telemetry(void)
{
  uint8_t senddata[MAXINDEX]; 

  if(Telem_mode==0)
  {
    //Send header data  <- 送るんじゃなくて生成
    Telem_mode = 1;
    make_telemetry_header_data(senddata);

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }  
  else if(Mode > AVERAGE_MODE)
  {
    const uint8_t N=10;
    //N回に一度送信
    if (Telem_cnt == 0)telemetry_sequence();
    Telem_cnt++;
    if (Telem_cnt>N-1)Telem_cnt = 0;
    //telemetry_sequence();  <-　なぜ消さないのか意味不
  }
}

void telemetry_sequence(void)
{
  uint8_t senddata[MAXINDEX]; 

  switch (Telem_mode)
  {
    case 1:
      make_telemetry_data(senddata);
      //Send !  <-　この下の分岐の中で送信している
      if(telemetry_send(senddata, sizeof(senddata))==1)esp_led(0x110000, 1);//Telemetory Reciver OFF  <-　真偽不明。とりあえずledの値を変えている
      else esp_led(0x001100, 1);//Telemetory Reciver ON <-　真偽不明。とりあえずledの値を変えている

      //Telem_mode = 2;  <-  使ってないのに何であるのかわからん
      break;
  }
}

void make_telemetry_header_data(uint8_t* senddata)
{
  float d_float;
  uint8_t d_int[4];
  uint8_t index=0;  

  index=2;
  for (uint8_t i=0;i<(MAXINDEX-2)/4;i++)
  {
    data2log(senddata, 0.0f, index);
    index = index + 4;
  }
  //Telemetry Header
  senddata[0]=99;
  senddata[1]=99;
  index=2;
  data_set(senddata, Roll_rate_kp, &index);
  data_set(senddata, Roll_rate_ti, &index);
  data_set(senddata, Roll_rate_td, &index);
  data_set(senddata, Roll_rate_eta, &index);
  data_set(senddata, Pitch_rate_kp, &index);
  data_set(senddata, Pitch_rate_ti, &index);
  data_set(senddata, Pitch_rate_td, &index);
  data_set(senddata, Pitch_rate_eta, &index);
  data_set(senddata, Yaw_rate_kp, &index);
  data_set(senddata, Yaw_rate_ti, &index);
  data_set(senddata, Yaw_rate_td, &index);
  data_set(senddata, Yaw_rate_eta, &index);
  data_set(senddata, Rall_angle_kp, &index);
  data_set(senddata, Rall_angle_ti, &index);
  data_set(senddata, Rall_angle_td, &index);
  data_set(senddata, Rall_angle_eta, &index);
  data_set(senddata, Pitch_angle_kp, &index);
  data_set(senddata, Pitch_angle_ti, &index);
  data_set(senddata, Pitch_angle_td, &index);
  data_set(senddata, Pitch_angle_eta, &index);
}

void make_telemetry_data(uint8_t* senddata)
{
  float d_float;
  uint8_t d_int[4];
  uint8_t index=0;  

  //Telemetry Header
  senddata[0]=88;
  senddata[1]=88;
  index = 2;
  data_set(senddata, Elapsed_time, &index);                             //1 Time
  data_set(senddata, Interval_time, &index);                            //2 delta Time
  data_set(senddata, (Roll_angle - Roll_angle_offset)*180/PI, &index);  //3 Roll_angle
  data_set(senddata, (Pitch_angle-Pitch_angle_offset)*180/PI, &index);  //4 Pitch_angle
  data_set(senddata, (Yaw_angle-Yaw_angle_offset)*180/PI, &index);      //5 Yaw_angle
  data_set(senddata, (Roll_rate)*180/PI, &index);                       //6 P
  data_set(senddata, (Pitch_rate)*180/PI, &index);                      //7 Q
  data_set(senddata, (Yaw_rate)*180/PI, &index);                        //8 R
  data_set(senddata, Roll_angle_reference*180/PI, &index);              //9 Roll_angle_reference
  //data_set(senddata, 0.5f * 180.0f *Roll_angle_command, index);
  data_set(senddata, Pitch_angle_reference*180/PI, &index);             //10 Pitch_angle_reference
  //data_set(senddata, 0.5 * 189.0f* Pitch_angle_command, index);
  data_set(senddata, Roll_rate_reference*180/PI, &index);               //11 P ref
  data_set(senddata, Pitch_rate_reference*180/PI, &index);              //12 Q ref
  data_set(senddata, Yaw_rate_reference*180/PI, &index);                //13 R ref
  data_set(senddata, Thrust_command/BATTERY_VOLTAGE, &index);           //14 T ref
  data_set(senddata, Voltage, &index);                                  //15 Voltage
  data_set(senddata, Accel_x_raw, &index);                              //16 Accel_x_raw
  data_set(senddata, Accel_y_raw, &index);                              //17 Accel_y_raw
  data_set(senddata, Accel_z_raw, &index);                              //18 Accel_z_raw
  data_set(senddata, Alt_velocity, &index);                             //19 Alt Velocity
  data_set(senddata, Z_dot_ref, &index);                                //20 Z_dot_ref
  //data_set(senddata, FrontRight_motor_duty, index);
  data_set(senddata, FrontLeft_motor_duty, &index);                     //21 FrontLeft_motor_duty
  data_set(senddata, RearRight_motor_duty, &index);                     //22 RearRight_motor_duty
  //data_set(senddata, RearLeft_motor_duty, index);
  data_set(senddata, Alt_ref, &index);                                  //23 Alt_ref
  data_set(senddata, Altitude2, &index);                                //24 Altitude2
  data_set(senddata, Altitude, &index);                                 //25 Sense_Alt
  data_set(senddata, Az, &index);                                       //26 Az
  data_set(senddata, Az_bias, &index);                                  //27 Az_bias
}

void telemetry400(void)
{
  uint8_t senddata[MAXINDEX]; 

  if(Telem_mode==0)
  {
    //Send header data  <- 送るんじゃなくて生成
    Telem_mode = 1;
    make_telemetry_header_data(senddata);

    //Send !
    telemetry_send(senddata, sizeof(senddata));
  }  
  else if(Mode > AVERAGE_MODE)
  {
    telemetry_sequence400();
  }
}

void telemetry_sequence400(void)
{
  uint8_t senddata[MAXINDEX]; 

  make_telemetry_data400(senddata);
  //Send !  <-　この下の分岐の中で送信されている
  if(telemetry_send(senddata, MININDEX)==1)esp_led(0x110000, 1);//Telemetory Reciver OFF
  else esp_led(0x001100, 1);//Telemetory Reciver ON
}

void make_telemetry_data400(uint8_t* senddata)
{
  float d_float;
  uint8_t d_int[4];
  uint8_t index=0;  

  //Telemetry Header
  senddata[0]=88;
  senddata[1]=88;
  index = 2;
  data_set(senddata, Elapsed_time, &index);//1 Time
  data_set(senddata, Accel_z_raw, &index); //2 Accel_z_raw
  data_set(senddata, Accel_z, &index);     //3 Accel_z
}

void data_set(uint8_t* datalist, float value, uint8_t* index)
{
  data2log(datalist, value, *index);//現在の index の位置に value を格納する。
  *index = *index + 4;//次回 data_set を呼び出す際に datalist の次のデータを記録する位置が 4 バイト分進む仕組み
}

void data2log(uint8_t* data_list, float add_data, uint8_t index)
{
    uint8_t d_int[4]; //d = destination（目的地）
    float d_float = add_data;
    float2byte(d_float, d_int);
    append_data(data_list, d_int, index, 4);
}

void float2byte(float x, uint8_t* dst)
{
  uint8_t* dummy;
  dummy = (uint8_t*)&x;//float のメモリをそのまま uint8_t の配列として参照し、各バイトを格納。
  dst[0]=dummy[0];
  dst[1]=dummy[1];
  dst[2]=dummy[2];
  dst[3]=dummy[3];
}

void append_data(uint8_t* data , uint8_t* newdata, uint8_t index, uint8_t len)
{
  for(uint8_t i=index;i<index+len;i++)
  {
    data[i]=newdata[i-index];//data_list の index 位置に newdata をコピー。これにより、4バイト分のデータ（変換されたfloatの値）が格納される
  }
}
