/*このコードは、ESP-NOWプロトコルを使用して通信を行うためのモジュールを構築しています。ESP-NOWはESP32デバイス間で短距離での低遅延通信を実現するプロトコルであり、ドローンなどのRC（リモートコントロール）操作に適しています。




ESP32とはEspressif Systems社により開発された、低消費電力のWi-Fi及びBLE通信を搭載したマイクロコントローラ(マイコン)です。 主にIoTデバイスやウェアラブル機器に利用されます。


Espressifが開発した、Wi-Fiの規格を利用した、アクセスポイントを介さずにデバイス同士で直接通信できる、低消費電力で高速な無線通信プロトコル
*/

#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// ========================
//  グローバル変数の定義
// ========================

//esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;   // 接続フラグ（通信が成立しているかを確認）

//Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
//ATOM Lite (C): 4C:75:25:AE:27:FC
//4C:75:25:AD:8B:20
//4C:75:25:AF:4E:84
//4C:75:25:AD:8B:20
//4C:75:25:AD:8B:20 赤水玉テープ　ATOM lite
uint8_t TelemAddr[6] = {0x4C, 0x75, 0x25, 0xAD, 0x8B, 0x20};  // Telemetry（相手側のESP32）のMACアドレス（データを送受信する対象）
//uint8_t TelemAddr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; 通信テスト時に使用可能な**ブロードキャストアドレス**

// **自身のESP32のMACアドレス**
volatile uint8_t MyMacAddr[6];
volatile uint8_t Rc_err_flag=0;   // 受信エラーフラグ 
esp_now_peer_info_t peerInfo;  // 接続する相手のMACアドレス、通信チャンネル、暗号化設定などを保持。

//RC

// **RCスティックの状態を格納する配列**
// 配列の各要素には、RCコントローラーの各軸（スティックの動き）に対応するデータが格納される。
// - Stick[RUDDER]   → ラダー（左右方向の回転）
// - Stick[THROTTLE] → スロットル（上下移動）
// - Stick[AILERON]  → エルロン（左右移動）
// - Stick[ELEVATOR] → エレベーター（前後移動）
// - Stick[BUTTON_ARM] → アームボタン（飛行開始）
// - Stick[BUTTON_FLIP] → フリップボタン（宙返り）
volatile float Stick[16];
// **受信データの送信元MACアドレス**
volatile uint8_t Recv_MAC[3];

// 受信コールバック
// ESP-NOW でデータを受信した際に自動的に呼び出される関数。
// 受信データを解析し、RCコントローラーのスティック情報に反映する。
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) 
{
  // **通信が継続中であることを示すフラグをリセット**
  // データを受信するごとに `Connect_flag` をリセットすることで、
  // 通信が途切れていないことを確認する。
  Connect_flag=0;

  uint8_t* d_int;　  // **データ変換用のポインタ**
  //int16_t d_short;
  float d_float;  // **受信データを格納する変数**

  // **送信元のMACアドレスの一部を取得**
  // 受信データの先頭3バイトに送信元のMACアドレスが格納されている
  Recv_MAC[0]=recv_data[0];
  Recv_MAC[1]=recv_data[1];
  Recv_MAC[2]=recv_data[2];

  // **受信したデータが自身のESP32向けの通信か確認**
  // 受信データに含まれるMACアドレスの一部（下位3バイト）が
  // 自身のMACアドレス（下位3バイト）と一致しているかチェックする。
  if ((recv_data[0]==MyMacAddr[3])&&(recv_data[1]==MyMacAddr[4])&&(recv_data[2]==MyMacAddr[5]))
  {
    Rc_err_flag = 0;  // 正常なデータならエラーを解除
  }
  else 
  {
    Rc_err_flag = 1;  // 異なる送信元からのデータならエラーを設定し処理を中断
    return;
  }

  //**受信データを変換してスティックの状態を更新**
  // 各操作軸（ラダー、スロットル、エルロン、エレベーター）の値を取得する。

  // `d_int`ポインタを用いて `float` 型のデータを変換する
  d_int = (uint8_t*)&d_float;  
   // **ラダー（方向転換）**
  d_int[0] = recv_data[3];
  d_int[1] = recv_data[4];
  d_int[2] = recv_data[5];
  d_int[3] = recv_data[6];
  Stick[RUDDER]=d_float;

   // **スロットル（上昇・下降）**
  d_int[0] = recv_data[7];
  d_int[1] = recv_data[8];
  d_int[2] = recv_data[9];
  d_int[3] = recv_data[10];
  Stick[THROTTLE]=d_float;

  // **エルロン（左右移動）**
  d_int[0] = recv_data[11];
  d_int[1] = recv_data[12];
  d_int[2] = recv_data[13];
  d_int[3] = recv_data[14];
  Stick[AILERON]  = d_float;

  // **エレベーター（前後移動）**
  d_int[0] = recv_data[15];
  d_int[1] = recv_data[16];
  d_int[2] = recv_data[17];
  d_int[3] = recv_data[18];
  Stick[ELEVATOR]  = d_float;

    // **各種ボタンの状態を取得**
  // これらはフリップやアームの操作を担当する。
  Stick[BUTTON_ARM] = recv_data[19];  // アームボタン（飛行開始）
  Stick[BUTTON_FLIP] = recv_data[20];  // フリップボタン（宙返り）
  Stick[CONTROLMODE] = recv_data[21];   // 操作モード（手動・自動切り替え）
  Stick[ALTCONTROLMODE] = recv_data[22];  // 高度制御モード（高度を固定するかどうか）

  // **ログを初期化**
  Stick[LOG] = 0.0;
  
// =====================================================
// データ受信のデバッグ出力（現在コメントアウトされている）
// =====================================================

// `#if 0` のため、このコードブロックは **コンパイル時に無効** になっている。
// `#if 1` に変更すると、USBシリアルモニタに受信データが表示される。
// `printf()` を使って、RCスティックの各軸の値を出力する。
// これにより、ESP-NOWで受信したスティックデータの確認が可能になる。

#if 0
  USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                            Stick[THROTTLE],   // スロットル（上下移動）
                                            Stick[AILERON],  // エルロン（左右移動）
                                            Stick[ELEVATOR],  // エレベーター（前後移動）
                                            Stick[RUDDER],  // ラダー（方向転換）
                                            Stick[BUTTON_ARM],  // アームボタン（飛行開始）
                                            Stick[BUTTON_FLIP],   // フリップボタン（宙返り）
                                            Stick[CONTROLMODE],  // 操作モード（手動・自動切り替え）
                                            Stick[ALTCONTROLMODE],  // 高度制御モード（高度を固定するか）
                                            Stick[LOG]);            // ログ情報（使用されていない場合は0）
#endif
}

// =====================================================
// 送信コールバック関数（データ送信後の状態を取得）
// =====================================================

// ESP-NOWでデータ送信を行った後に呼び出されるコールバック関数。
// `status` を使って送信成功か失敗かを記録する。
uint8_t esp_now_send_status;

void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  esp_now_send_status = status;  // **送信の成功/失敗ステータスを記録**
}
// ESP-NOW通信の初期化処理
void rc_init(void)
{
  //Initialize Stick list 
   // **RCスティックの初期化**
  // 各スティックの値を `0.0` にリセットし、起動時の値を安定させる。
  for (uint8_t i = 0;i<16;i++)Stick[i]=0.0;

 // **ESP-NOWの初期化**
  // ステーションモードに設定し、通常のWi-Fi接続を切断する。
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // **自身のESP32のMACアドレスを取得**
  WiFi.macAddress((uint8_t*)MyMacAddr);
  // **MACアドレスの表示**
  // ESP32が持っているユニークなMACアドレスをUSBシリアルモニタに出力する。
  // これにより、デバイスの識別が容易になる。
  USBSerial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                  MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

  // ESP-NOWの初期化結果を確認
  if (esp_now_init() == ESP_OK) {
    USBSerial.println("ESPNow Init Success");  // 成功時のメッセージ
  } else {
    USBSerial.println("ESPNow Init Failed");  // 失敗時のメッセージ
    ESP.restart();  // **エラー発生時はESP32を再起動**
  }

// =====================================================
// MACアドレスのブロードキャスト設定
// =====================================================

// **ブロードキャストアドレスを設定**
// `0xFF:0xFF:0xFF:0xFF:0xFF:0xFF` は、全デバイスにデータを送信できるアドレス。
// 特定のMACアドレスに限定せず、複数デバイスへ情報を送信する場合に使用する。
  uint8_t addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, addr, 6);   // ピア情報にブロードキャストアドレスを設定
  peerInfo.channel = CHANNEL;  // チャンネル設定（事前定義されている `CHANNEL` を使用）
  peerInfo.encrypt = false;   // 通信の暗号化を無効化（ESP-NOWは通常、暗号化なし）

  // **通信相手を追加**
　// `esp_now_add_peer()` を使用して、ブロードキャスト通信の対象を追加する。
　// 追加に失敗した場合は、シリアルモニタにエラーメッセージを表示して処理を終了。
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        USBSerial.println("Failed to add peer");  // ピア追加失敗時のログ出力
        return;
  }
    
  // **Wi-Fiチャンネルの設定**
　// ESP-NOWではWi-Fiチャンネルを明示的に指定する必要があるため、設定を適用。
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

 // =====================================================
// 自身のMACアドレスを送信
// =====================================================

// **自身のMACアドレスを送信し、通信相手に識別を伝える**
// `send_peer_info()` を呼び出し、自分のMACアドレスを送信する。
// 20回繰り返し送信し、確実に相手側へ通知する。
  for (uint16_t i=0; i<20; i++)
  {
    send_peer_info();  // MACアドレスを送信
    delay(50);    // 50ミリ秒の遅延（安定した送信を保証）

    // デバッグ用の出力（コメントアウトされている）
    //USBSerial.printf("%d\n", i);
  }

　// =====================================================
　// ESP-NOWの再初期化
　// =====================================================
　
　// ESP-NOWの初期化を再度行い、安定した通信を保証する。
　// 一度ブロードキャスト通信を行った後、再設定することで確実に相手と接続できるようにする。
  WiFi.mode(WIFI_STA);  // Wi-Fiをステーションモードに設定
  WiFi.disconnect();  // 通常のWi-Fi通信を切断し、ESP-NOW専用モードに移行

  // ESP-NOWの再初期化（通信スタックをリロード
  if (esp_now_init() == ESP_OK) {
    USBSerial.println("ESPNow Init Success2");  // 成功時のメッセージ
  } else {
    USBSerial.println("ESPNow Init Failed2");  // 失敗時のメッセージ
    ESP.restart();  // エラー時はESP32を再起動
  }

// =====================================================
// 通信相手（Telemetry）とのペアリング
// =====================================================

// **特定のTelemetryデバイスとの通信を確立**
// `peerInfo.peer_addr` に、事前設定された `TelemAddr`（相手のMACアドレス）をコピー。
// これにより、ブロードキャスト通信ではなく特定のデバイスとの直接通信が可能になる。
  
  memcpy(peerInfo.peer_addr, TelemAddr, 6);  // TelemetryデバイスのMACアドレスを設定
  peerInfo.channel = CHANNEL;  // チャンネル設定
  peerInfo.encrypt = false;  // ESP-NOWの暗号化を無効化

  // **通信相手を追加**
// Telemetryデバイスをピアとして登録。
  if (esp_now_add_peer(&peerInfo) != ESP_OK) 
  {
        USBSerial.println("Failed to add peer2");  // 追加に失敗した場合のエラーメッセージ
        return;
  }
  // =====================================================
// ESP-NOWのコールバック関数登録
// =====================================================

// **受信時の処理を登録**
// ESP-NOWでデータを受信した際に `OnDataRecv()` を実行するよう設定。
  
  esp_now_register_recv_cb(OnDataRecv);
  
  // **送信時の処理を登録**
// ESP-NOWでデータを送信した際に `on_esp_now_sent()` を実行するよう設定。
  esp_now_register_send_cb(on_esp_now_sent);
  
  // **通信の準備完了メッセージ**
  // ESP-NOWの全ての設定が完了したことをシリアルモニタに通知。
  USBSerial.println("ESP-NOW Ready.");
}

// =====================================================
// **ESP-NOW 通信のピア情報を送信する関数**
// =====================================================

// ESP-NOWの通信相手（ピア）に自身のMACアドレスとチャンネル情報を送信。
// これにより相手側のデバイスが認識しやすくなる
void send_peer_info(void)
{
  uint8_t data[7];  // 送信データ（チャンネル情報 + 自身のMACアドレス）
  data[0] = CHANNEL;   // **チャンネル番号をセット**

  // **自身のMACアドレスをデータにコピー**
  // `memcpy()` を使って `MyMacAddr` を `data` に格納する
  memcpy(&data[1], (uint8_t*)MyMacAddr, 6);  

  // **ESP-NOWでデータ送信**
  // `peerInfo.peer_addr` のデバイスに対して `data` を送信する
  esp_now_send(peerInfo.peer_addr, data, 7);
}

// =====================================================
// **Telemetryデータの送信関数**
// =====================================================

// ESP-NOWを利用してTelemetryデータを送信する。
// 送信エラーの管理も行い、一定回数エラーが発生するとリカバリー処理を実施。
uint8_t telemetry_send(uint8_t* data, uint16_t datalen)
{
  static uint32_t cnt=0;  // エラーカウント
  static uint8_t error_flag = 0;  // 送信エラーフラグ
  static uint8_t state=0;   // 送信状態管理

  esp_err_t result;  // ESP-NOWの送信結果を格納する変数

  // **送信処理の実行**
  // もしエラーが発生していなければ、データを送信
  if ((error_flag == 0)&&(state==0))
  {
    result = esp_now_send(peerInfo.peer_addr, data, datalen);
    cnt=0;   // エラーカウンタをリセット
  }
  else cnt++;  // 送信エラー時にカウンタを増加

  // 送信の成否を確認
  if (esp_now_send_status == 0)// 送信成功
  {
    error_flag = 0;  // エラーフラグを解除
    //state = 0;
  }
  else
  {
    error_flag = 1;   // エラーフラグをセット
    //state = 1;
  }

  // **エラー発生時のリカバリー処理**
  // 500回以上エラーが続いた場合はフラグをリセット
  if (cnt>500)
  {
    error_flag = 0;
    cnt = 0;
  }
  cnt++;  // カウンタ更新
  //USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

  return error_flag;  // 送信の成功/失敗結果を返す
}

// =====================================================
// **RC通信の終了処理**
// =====================================================

// RC通信を終了するための関数。
// 例として `Ps3.end();` のコードがコメントアウトされているが、
// PlayStation 3のコントローラを終了する場合に利用される。
void rc_end(void)
{
    // Ps3.end(); // PS3コントローラの終了（必要ならば有効化）
}

// =====================================================
// **RC接続状態の確認**
// =====================================================

// RCコントローラの接続が維持されているかを確認する関数。
// `Connect_flag` の値をチェックし、一定回数データが受信されない場合は接続が切れたと判定する。
uint8_t rc_isconnected(void)
{
    bool status;  // 接続状態を保持する変数
  
    Connect_flag++; // カウンタを増加

    // 接続判定
    // もし `Connect_flag` が10未満なら接続は維持されていると判定（status = 1）
    // 10以上になると通信が途絶えたと判断し、status = 0 に設定する。
    if (Connect_flag<10)status = 1;
    else status = 0;
    //USBSerial.printf("%d \n\r", Connect_flag);
    return status;  // 接続状態を返す（1: 接続中 / 0: 切断）
}

void rc_demo()
{
}

