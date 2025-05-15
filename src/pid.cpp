/*このコードは、PID（Proportional-Integral-Derivative）制御とフィルタリングのアルゴリズムを実現するためのクラスを提供しています。

PID制御とフィルタリングを利用して、動的システムを安定的に制御することを目的としています。

PIDクラス: 制御対象の位置や速度を調整するための出力を計算。

Filterクラス: センサーのノイズを除去し、滑らかな値を提供。




PID制御（Proportional-Integral-Derivative Controller）は、制御工学におけるフィードバック制御の一種である。出力値と目標値との偏差、その積分、および微分の3つの要素によって、入力値の制御を行う方法である

PID クラス
PID制御は比例(P)、積分(I)、微分(D)の要素を組み合わせて、システムの制御を行うアルゴリズムです。このクラスは、PID制御の計算を実装しています。

PIDフィルター(Filter)クラスとは、PID制御を実装するためのプログラミングにおけるクラス設計を指します。PID制御は、比例（Proportional）、積分（Integral）、微分（Derivative）の3つの要素を組み合わせ、システムを目標値に追従させる制御手法です。このクラスは、PID制御の各要素を管理し、制御ロジックを効率的に実装するために設計されます。
*/

#include <Arduino.h>
#include "pid.hpp"

//PIDの初期値を設定するコンストラクタです。デフォルトのゲインは非常に小さい値になっていますが、後から変更可能です。

PID::PID()
{
  m_kp=1.0e-8f;   // 比例ゲイン
  m_ti=1.0e8f;    // 積分時間
  m_td=0.0f;      // 微分時間
  m_eta = 0.01;    // 微分フィルタ係数
  m_integral=0.0f;   //積分成分の初期化
  m_differential=0.0f;  //微分成分の初期化
  m_err=0.0f;           //直近の誤差の初期化
  m_h=0.01f;      // サンプリング時間
}

// PIDパラメータの設定（制御の特性を変更）
void PID::set_parameter(
    float kp,   //比例ゲイン
    float ti,   //積分時間
    float td,   //微分時間
    float eta,   //微分フィルタ係数 （微分動作のノイズ除去用）
    float h)    //サンプリング時間 これらのパラメータを設定することで、PIDの動作を調整できます。
{
  m_kp=kp;    // 比例ゲイン（P）：大きいと応答が速くなる
  m_ti=ti;    // 積分時間（I）：小さいとズレ補正が強くなる
  m_td=td;    // 微分時間（D）：大きいと急変に反応しやすい
  m_eta=eta;  // 微分フィルタ係数（Dのノイズ除去用）
  m_h=h;      // サンプリング時間（制御の更新頻度）
}

  // 状態変数（過去の制御値を保持する）PID制御の変数をリセット。システムの再起動や状態変更時に使用
void PID::reset(void)
{
  m_integral=0.0f;  // 積分値をゼロに戻す
  m_differential=0.0f;  // 微分値をゼロに戻す
  m_err=0.0f;   // 誤差をゼロに戻す
  m_err2=0.0f;
  m_err3=0.0f;
}

void PID::i_reset(void)
{
  m_integral=0.0f;
}
 //現在のPIDパラメータをシリアルモニターに出力する。デバッグやチューニング用
void PID::printGain(void)
{
  Serial.printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Eta:%8.4f h:%8.4f\r\n", 
   m_kp, m_ti, m_td, m_eta, m_h);
}

void PID::set_error(float err)
{
  m_err = err;
}

//PID制御の更新処理。目標値との差分（誤差）を受け取り、最適な制御信号を計算。
float PID::update(float err, float h)
{
  float d;
  m_h = h;    // サンプリング時間を更新

  // 1. 積分計算（過去の誤差の累積補正）
  m_integral = m_integral + m_h * (err + m_err)/2/m_ti;
  
  // 積分成分の制限（アンチワインドアップ）
  // - 積分値が過剰に大きくなると制御が不安定になるため、適切な範囲に制限。
  if(m_integral> 30000.0f)m_integral = 30000.0f;
  if(m_integral<-30000.0f)m_integral =-30000.0f;
  // 2.不完全微分
  // - 誤差の変化率を計算して微分成分を算出する。
  // - 直接の微分計算はノイズを拾いやすいため、フィルタを適用して平滑化。
  m_differential = (2*m_eta*m_td - m_h)*m_differential/(2*m_eta*m_td + m_h)
                  + 2*m_td*(err - m_err)/(2*m_eta*m_td + m_h);
  // 3. 誤差更新（次回計算用に現在の誤差を保存）
  m_err  = err;
   // 4. PID制御出力を計算
    // - P（比例）：誤差に対して比例的に反応
    // - I（積分）：長期的な誤差を補正
    // - D（微分）：誤差の急変に対応
  return m_kp*(err + m_integral + m_differential); 
}
//コンストラクタ - フィルタの初期値を設定. 目標は、センサーデータを滑らかにし、不要なノイズを除去すること。
Filter::Filter()
{
  m_state = 0.0f;  // 初期状態
  m_T = 0.0025f;    // 時定数（フィルタの強さ）
  m_h = 0.0025f;    // サンプリング時間
}

//フィルタの状態をゼロにリセット
void Filter::reset(void)
{
  m_state = 0.0f;
}

//フィルタの時定数とサンプリング時間を設定
//フィルタの強度を調整可能
void Filter::set_parameter(float T, float h)
{
  m_T = T;
  m_h = h;
}


//フィルタリング処理（ノイズ除去）
//入力値 `u` を受け取り、過去の状態 `m_state` と組み合わせて滑らかな出力を生成。
float Filter::update(float u, float h)
{
  m_h = h;
  
  // 平滑化フィルタの計算（過去の値と新しい値をブレンド）
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}
