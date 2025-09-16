#pragma once
#include <Arduino.h>

class CommandProcessor {
public:
  CommandProcessor(Stream& serial);

  void receive();  // SET_CMD受信処理
  void send();     // STATE送信処理

  void setStateData(const float* p_vals);  // 送信データ設定
  void getStateData(float* p_out, float* e_out);                // 送信データ取得
  void getReceivedData(float* p_out, float* e_out);             // 受信データ取得
  void resetReceivedData();  // received_p と received_e をリセット

private:
  Stream& serial;

  float received_p[8] = {1.02,-1.02,0,0,0,0,0,0};  // SET_CMDで受信したデータ
  float received_e[8] = {0.0f};

  float state_p[8] = {0.0f};     // STATEで送信するデータ
  float state_e[8] = {0.0f};

  bool parseCommand(const String& line);
};