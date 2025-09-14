#include "FeetechServo.h"
#include "ODriveCAN.h"
#include "C610Controller.h"
#include <M5Stack.h>
#include <mcp_can.h>
#include <SPI.h>
#include <AS5600.h>
#include <Wire.h>

AS5600 as5600;
const int encoder_offset = 319;  // オフセット角度（必要に応じて調整）

const int r_lim = 2; // can id = 3 == right
const int l_lim = 5; // can id = 2 == left

const int Estop = 35; // Emergency stop pin

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -1.7;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = -3.42; // for node ID 5

float rev = 1.2; // strictry positive

// CAN設定
const int csPin = 12; // SPI CS ピン
MCP_CAN can(csPin); // Set CS to pin 12

ODriveCAN odrive(&can);
C610Controller c610(&can);

// サーボ設定
const std::vector<byte> servoIDs = {1, 2, 3}; // 複数のサーボID
const int centerPosition = 2048;
const float maxAmplitude = 1600.0;
const float radiansIncrement = 0.06;

// UART2 設定
HardwareSerial SerialSTS(2); // UART2
ServoController servoController(SerialSTS, servoIDs, centerPosition, maxAmplitude, radiansIncrement);

// C610 電流
uint16_t current1 = 0;
uint16_t current2 = 0;
uint16_t current3 = 0;
uint16_t current4 = 0;

// initial current setup
uint16_t current2_init = 650;
uint16_t current3_init = 650;

float rev1 = 0;
float rev2 = 0; // 目標値は必ずマイナス
float rev3 = 0; // 目標値は必ずマイナス

float rev2_offset = 0;
float rev3_offset = 0;

// 🔧 角度取得関数（引数なし、float型の角度を返す）
float getAS5600Angle() {
  uint16_t raw = as5600.readAngle();  // 0–4095
  float degrees = raw * 360.0 / 4096.0 - encoder_offset;

  // ±180度範囲に収める
  if (degrees > 180.0) {
    degrees -= 360.0;
  } else if (degrees < -180.0) {
    degrees += 360.0;
  }

  return degrees;
}

void CANUpdateTask(void* pvParameters) {
  while (true) {
    c610.update();         // CAN受信処理
    vTaskDelay(1);         // 1 tick ≒ 1ms（CPU負荷軽減）
  }
}
void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on M5Stack
    as5600.begin();  // 初期化（I2Cアドレスはデフォルト0x36）
    
    pinMode(r_lim, INPUT_PULLUP);     // 内部プルアップ有効
    pinMode(l_lim, INPUT_PULLUP);     // 内部プルアップ有効
    pinMode(Estop, INPUT);     // Emergency stop pin


    // CAN通信の初期化
    if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        while (1);
    }
    can.setMode(MCP_NORMAL);
    M5.Lcd.println("CAN Initialized");

    // RTOSタスク起動（スタックサイズ2048、優先度1、Core1で実行）
    xTaskCreatePinnedToCore(
        CANUpdateTask,       // タスク関数
        "CANUpdate",         // タスク名
        2048,                // スタックサイズ（バイト）
        NULL,                // 引数（不要ならNULL）
        1,                   // 優先度（0〜5）
        NULL,                // タスクハンドル（不要ならNULL）
        1                    // 実行するコア（0または1）
    );

    if(!digitalRead(r_lim) || !digitalRead(l_lim)) {
        M5.Lcd.println("Error: Limit switch is active at startup!");
        while(1);
    }
    else {
        current3 = current3_init + 300;
        current2 = current2_init + 300; // 少し強めに
        c610.setCurrents(current1, current2, current3, current4);
        delay(10);
    }

    while(digitalRead(r_lim) || digitalRead(l_lim)) {
        if (digitalRead(r_lim) && digitalRead(l_lim)) {
            current3 = current3_init;
            current2 = current2_init;
            c610.setCurrents(current1, current2, current3, current4);
            delay(10);
        }else if (!digitalRead(r_lim)) {
            current3 = 0;
            current2 = current2_init;
            c610.setCurrents(current1, current2, current3, current4);
            delay(10);
        } else if (!digitalRead(l_lim)) {
            current2 = 0;
            current3 = current3_init;
            c610.setCurrents(current1, current2, current3, current4);
            delay(10);
        }
    }
    current2 = 0;
    current3 = 0;
    
    c610.setCurrents(current1, current2, current3, current4);
    // c610.update();
    delay(1000);
    
    rev2_offset = c610.getAngle(1);
    rev3_offset = c610.getAngle(2);
    M5.Lcd.printf("Initialization done.\n");
    delay(3000);

    // ODriveの初期設定
    odrive.begin(0x8B, 0x87); // CAN IDを指定して初期化,node ID 4
    delay(10);
    odrive.begin(0xAB, 0xA7); // CAN IDを指定して初期化,node ID 5

    odrive.setPosition(0x8C, offset_ax4);  // ノードID 4 を原点に復帰
    delay(2);
    odrive.setPosition(0xAC, offset_ax5);  // ノードID 5 を原点に復帰

    delay(3000); // 少し待つ
    odrive.setPosition(0x8C, offset_ax4 + 1.3);  // ノードID 4 を原点に復帰
    delay(2);
    odrive.setPosition(0xAC, offset_ax5 - 1.3);  // ノードID 5 を原点に復帰
    

    servoController.setup();

    // c610.setCurrents(current1, current2, current3, current4);
}

void loop() {
    // servoController.setAngleWithSpeed(1, 90.0, 100);
    
    float angle = getAS5600Angle();

    M5.Lcd.fillRect(0, 30, 320, 30, BLACK);
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("Angle: %.2f deg", angle);

    // if (abs(angle) > 30.0) {
    //     current1 = 0;
    // }
    // else {
    //     current1 = -400;
    // }
    // c610.setCurrents(current1, current2, current3, current4);

    rev1 = c610.getAngle(0);   // モータ1の積算角度（±∞）
    rev2 = c610.getAngle(1) - rev2_offset;
    rev3 = c610.getAngle(2) - rev3_offset;

    M5.Lcd.setCursor(30, 60);
    M5.Lcd.printf("Rev1: %.2f deg", rev1);
    M5.Lcd.setCursor(30, 90);
    M5.Lcd.printf("Rev2: %.2f deg", rev2);
    M5.Lcd.setCursor(30, 120);
    M5.Lcd.printf("Rev3: %.2f deg", rev3);

    M5.Lcd.setCursor(0, 150);
    if (!digitalRead(Estop)) {
        M5.Lcd.printf("EMERGENCY STOP!");
        current1 = 0;
        current2 = 0;
        current3 = 0;
        c610.setCurrents(current1, current2, current3, current4);
        while(1);
    } else {
        M5.Lcd.printf("No Emergency");
    }

    if (rev2 > -2*M_PI*10) {
        current2 = -650;
    } else {
        current2 = 0;
    }
    if (rev3 > -2*M_PI*10) {
        current3 = -650;
    } else {
        current3 = 0;
    }   
    c610.setCurrents(current1, current2, current3, current4);

    delay(20);  // 応答性向上のため少し短く
    
    //c610.update();


}