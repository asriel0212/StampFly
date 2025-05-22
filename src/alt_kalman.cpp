/*このコードは、Kalmanフィルタを使用してドローンや移動体などのシステムにおいて、高度や速度を推定するために設計されています。センサーからの測定データがノイズを含んでいる場合でも、状態推定を行い、不確実性を減少させます。

ノイズ除去:
センサー測定の不確実性（ノイズ）の影響を大幅に軽減。

リアルタイム推定:
加速度センサーや高度センサーからのデータを即時に処理し、リアルタイムで速度や高度を推定。

動的システムへの適応:
状態遷移、外部入力（加速度）、観測値を組み合わせた動的な推定を実現。
*/

#include "alt_kalman.hpp"    // カルマンフィルタのアルゴリズムを定義したヘッダーファイルをインクルード
//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>
#include <Arduino.h>   // Arduinoの基本ライブラリを使用

// Alt_kalman クラスのコンストラクタ
Alt_kalman::Alt_kalman(){};

// 状態推定を行う関数
// z_sens: 高度センサーの測定値
// accel: 加速度センサーの測定値
// h: 時間ステップ (dt)
void Alt_kalman::update(float z_sens, float accel, float h)
{
  step = h;   // 更新間隔 (dt) を設定

  //x:estimate x_:predict
  

  //Matrix update
  //f13 = -step;
  //f21 = step;
  //f33 = 1 + beta*step;
  //b11 = step;
  //b32 = step;
  
  // --------- 状態の予測 (Predict) --------- //
  // 予測ステップでは、前回の状態から現在の状態を推定します。
  //predict state
  
  velocity_ = velocity + (accel-bias)*step;  // 加速度の影響を考慮して速度を予測
  altitude_ = altitude + velocity*step;  // 速度の影響を考慮して高度を予測
  bias_ = bias*(1+step*beta);   // バイアスの影響を考慮 (センサーのずれを補正)
  //velocity = velocity_;
  //altitude = altitude_; 

  // --------- 誤差共分散行列 (Predict P) --------- //
  // 状態推定の信頼性を示す誤差共分散行列を更新
  //predict P
  p11_ = p11 - step*(p31+p13) + step*step*p33 + step*step*q1;  // 速度の推定誤差更新
  p12_ = step*(p11-p32) - step*step*p31 + p12;  // 速度・高度の誤差相関更新
  p13_ = (1+beta*step)*(p13 - step*p33);  // バイアスとの関連性を考慮した誤差更新
  
  p21_ = step*(p11-p23)-step*step*p13 + p21;   // 速度と高度の誤差共分散更新
  p22_ = step*step*p11 + step*(p21+p12) + p22;   // 高度の誤差推定
  p23_ = (1+beta*step)*(p23 + step*p13);  // バイアスの誤差更新
  
  p31_ = (1+beta*step)*(p31-step*p33);  // バイアスの誤差相関更新
  p32_ = (1+beta*step)*(p32+step*p31);   // バイアスの高度推定誤差
  p33_ = (1+beta*step)*(1+beta*step)*p33 + step*step*q2;  // バイアスの誤差推定
  //USBSerial.printf("%f %f %f  %f %f %f  %f %f %f\n\r",p11_,p12_,p13,p21_,p22_,p23_,p31_,p32_,p33_);

  // --------- カルマンゲインの計算 (Kalman Gain) --------- //
  // 測定データとの誤差を補正するためにカルマンゲインを算出
  //update kalman gain
  float s = p22_ + R;  // 観測値に加わるノイズを考慮
  k1 = p12_ / s;  // 速度補正用のゲイン
  k2 = p22_ / s;  // 高度補正用のゲイン
  k3 = p32_ / s;  // バイアス補正用のゲイン

  // --------- 状態推定の補正 (Update) --------- //
  // センサー値と予測値の差分（誤差）を計算
  //inovation
  float e = z_sens - altitude_;   // センサー測定値との差異

  //estimate state
  velocity = velocity_ + k1 * e;  // 誤差補正後の速度
  altitude = altitude_ + k2 * e;  // 誤差補正後の高度
  bias = bias_ + k3 * e;     // 誤差補正後のバイアス (加速度計のドリフト補正)

  //修正値の出力
  //Estimated state output
  Velocity = velocity;
  Altitude = altitude;
  Bias = bias;
  //printf("%11.3f %11.4f %11.4f %11.4f %11.4f ", t+step, velocity, altitude, z_sens, true_v);

  //誤差共分散行列の更新 (Update P) 
  //estimate P
  p11 = p11_ - k1 * p21_;   // 速度の推定誤差補正
  p12 = p12_ - k1 * p22_;  // 高度との誤差補正
  p13 = p13_ - k1 * p23_;   // バイアスとの誤差補正
  p21 = p21_ * (1 - k2);   // 高度の誤差補正
  p22 = p22_ * (1 - k2);   // 高度の推定誤差更新
  p23 = p23_ * (1 - k2);  // バイアスの推定誤差更新
  p31 = -k3*p21_ + p31_;   // バイアス補正後の誤差更新
  p32 = -k3*p22_ + p32_;  // バイアス補正後の誤差更新
  p33 = -k3*p23_ + p33_;  // バイアス補正後の推定誤差

  //printf("%11.4f %11.4f %11.4f %11.4f\n", p11, p12, p21, p22);
}

// 速度を手動設定する関数
void Alt_kalman::set_vel(float v)
{
  velocity = v;  // 初期速度を設定
}

// 将来的に行列計算用の関数 (未実装)
void mat_times(Mat A, Mat B)
{

}
