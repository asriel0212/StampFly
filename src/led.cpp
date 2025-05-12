/*このコードは、LED制御のロジックを実現するプログラムで、FastLEDライブラリを使用して複数のLEDを制御しています。飛行制御システムの一部として、LEDはシステムモードや状態に応じた点灯や点滅を行う役割を果たします。

このコードは、ドローンやロボットシステムのLEDステータス表示を実現します。

モード（例: AVERAGE_MODE, FLIGHT_MODE, PARKING_MODE）に応じてLEDの色や点滅状態を変更。

エラーステータス（例: RCエラー、低電圧など）や制御状態（例: 角速度制御、高度制御）を視覚的にユーザーへフィードバック。


https://lang-ship.com/blog/work/fastled/
定番LED制御ライブラリで、特にNeoPixel的な複数のLEDがつながっているタイプのLED制御ではよく使われています。
*/

#include "led.hpp"
#include "sensor.hpp"
#include "rc.hpp"
#include "flight_control.hpp"

// LEDの色を設定する変数
uint32_t Led_color = 0x000000; // 現在のLEDの色（デフォルトはOFF）
uint32_t Led_color2 = 255;    // 点滅・アニメーション用の色
uint32_t Led_color3 = 0x000000; // 未使用（将来的な拡張）
uint16_t LedBlinkCounter=0;    // LEDの点滅カウンター
CRGB led_esp[1];      // ESP32側のLED（1個）
CRGB led_onboard[2];  // 基板上のLED（2個）

// LED制御関数の宣言
void led_drive(void);
void onboard_led1(CRGB p, uint8_t state);
void onboard_led2(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);

// **LEDの初期化**
void led_init(void)
{
  // 基板上のLED（2個）を初期化
  FastLED.addLeds<WS2812, PIN_LED_ONBORD, GRB>(led_onboard, 2);
   // ESP側のLED（1個）を初期化
  FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(led_esp, 1);
}

// **LEDの表示を適用**
void led_show(void)
{
  FastLED.show();
}

// **LEDの動作制御（飛行モードや状態に応じて変更）**
void led_drive(void)
{
  if (Mode == AVERAGE_MODE)
  {
    // 通常モード（紫色に点灯）
    onboard_led1(PERPLE, 1);
    onboard_led2(PERPLE, 1);
  }
  else if(Mode == FLIGHT_MODE)
  {
    // 飛行中の角度制御モード
    if(Control_mode == ANGLECONTROL)
    {
      if(Flip_flag==0)Led_color=0xffff00; // 通常飛行（黄色）
      else Led_color = 0xFF9933; // フリップ時（オレンジ）
    }
    else Led_color = 0xDC669B; // 通常飛行時の色（ピンク系）

    // 高度維持時（濃い紫）
    if(Alt_flag == 1) Led_color = 0x331155;
    // RC信号エラー時（赤）
    if(Rc_err_flag == 1) Led_color = 0xff0000;

    // **低電圧時の処理**
    if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {onboard_led1(Led_color, 1);onboard_led2(Led_color, 1);}
    else {onboard_led1(POWEROFFCOLOR,1);onboard_led1(POWEROFFCOLOR,1);}
  }

  else if (Mode == PARKING_MODE)
  {
    if(Under_voltage_flag < UNDER_VOLTAGE_COUNT)
    {
      // **イルミネーション処理**
      if(LedBlinkCounter==0){//<10
        if (Led_color2&0x800000)Led_color2 = (Led_color2<<1)|1;
        else Led_color2=Led_color2<<1; 

        if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {onboard_led1(Led_color2, 1);onboard_led2(Led_color2, 1);}
        //else onboard_led(POWEROFFCOLOR,1);
        LedBlinkCounter++;
      }
      LedBlinkCounter++;
      if (LedBlinkCounter>20)LedBlinkCounter=0;
    }
    else
    {
      // **水色の点滅（電圧低下時）**
      if (LedBlinkCounter < 10) { onboard_led1(POWEROFFCOLOR,1);onboard_led2(POWEROFFCOLOR,1);}
      else if (LedBlinkCounter < 200) { onboard_led1(POWEROFFCOLOR,0);onboard_led2(POWEROFFCOLOR,0);}
      else LedBlinkCounter = 0;
      LedBlinkCounter ++;
    }
  }

  //LED show　**LEDの状態を適用**
  FastLED.show();
}

// **基板上のLED1の制御**
void onboard_led1(CRGB p, uint8_t state)
{
  if (state ==1)
  {
    led_onboard[0]=p;　 // LEDを点灯
  } 
  else {
    led_onboard[0]=0;　// 消灯
  }
  return;
}

// **基板上のLED2の制御**
void onboard_led2(CRGB p, uint8_t state)
{
  if (state ==1)
  {
    led_onboard[1]=p;
  } 
  else {
    led_onboard[1]=0;
  }
  return;
}

// **ESP側のLEDの制御**
void esp_led(CRGB p, uint8_t state)
{
  if (state ==1) led_esp[0]=p;　// LEDを点灯
  else led_esp[0]=0;　// 消灯
  return;
}
