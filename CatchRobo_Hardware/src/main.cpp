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

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -1.72;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = -0.82; // for node ID 5

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

void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on M5Stack
    as5600.begin();  // 初期化（I2Cアドレスはデフォルト0x36）


    // CAN通信の初期化
    if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        while (1);
    }
    can.setMode(MCP_NORMAL);
    M5.Lcd.println("CAN Initialized");

    // ODriveの初期設定
    odrive.begin(0x8B, 0x87); // CAN IDを指定して初期化,node ID 4
    delay(10);
    odrive.begin(0xAB, 0xA7); // CAN IDを指定して初期化,node ID 5

    odrive.setPosition(0x8C, offset_ax4);  // ノードID 4 を原点に復帰
    delay(2);
    odrive.setPosition(0xAC, offset_ax5);  // ノードID 5 を原点に復帰

    servoController.setup();
    // delay(6000);
    
    // odrive.setPosition(0xAC, offset_ax5 - rev);  // ノードID 5 に位置2.0fを送信
    // delay(2);
    // odrive.setPosition(0x8C, offset_ax4 + rev);  // ノードID 4 に位置2.0fを送信
    
    // rev = 0.2;
    // delay(6000);
    
    // odrive.setPosition(0xAC, offset_ax5 - rev);  // ノードID 5 に位置2.0fを送信
    // delay(2);
    // odrive.setPosition(0x8C, offset_ax4 + rev);  // ノードID 4 に位置2.0fを送信
    
    // rev = 1.0;
    // delay(6000);
    
    // odrive.setPosition(0xAC, offset_ax5 - rev);  // ノードID 5 に位置2.0fを送信
    // delay(2);
    // odrive.setPosition(0x8C, offset_ax4 + rev);  // ノードID 4 に位置2.0fを送信

    // c610.setCurrents(current1, current2, current3, current4);

    // // サーボID 1 に 90度を速度100で設定
    // servoController.setAngleWithSpeed(1, 90.0, 100);

    // // サーボID 2 に 180度を速度200で設定
    // servoController.setAngleWithSpeed(2, 180.0, 200);

    // // サーボID 3 に 45度を速度150で設定
    // servoController.setAngleWithSpeed(3, 45.0, 150);
}

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

void loop() {
    servoController.setAngleWithSpeed(1, 90.0, 100);
    c610.update();
    
    float angle = getAS5600Angle();

    M5.Lcd.fillRect(0, 30, 320, 30, BLACK);
    M5.Lcd.setCursor(0, 30);
    M5.Lcd.printf("Angle: %.2f deg", angle);

}
