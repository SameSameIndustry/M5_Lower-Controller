#include "FeetechServo.h"
#include "ODriveCAN.h"
#include "C610Controller.h"
#include <M5Stack.h>
#include <mcp_can.h>
#include <SPI.h>

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -0.72;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = 0.22; // for node ID 5

float rev = 1.0;

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

    // CAN通信の初期化
    if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        while (1);
    }
    can.setMode(MCP_NORMAL);
    M5.Lcd.println("CAN Initialized");

    // ODriveの初期設定
    odrive.begin(0x8B, 0x87); // CAN IDを指定して初期化,node ID 4
    odrive.begin(0xAB, 0xA7); // CAN IDを指定して初期化,node ID 5

    odrive.setPosition(0x8C, offset_ax4);  // ノードID 4 を原点に復帰
    odrive.setPosition(0xAC, offset_ax5);  // ノードID 5 を原点に復帰

    servoController.setup();
    delay(1000);
    
    odrive.setPosition(0xAC, offset_ax5 - rev);  // ノードID 5 に位置2.0fを送信
    odrive.setPosition(0x8C, offset_ax4 + rev);  // ノードID 4 に位置2.0fを送信

    c610.setCurrents(current1, current2, current3, current4);

    // // サーボID 1 に 90度を速度100で設定
    // servoController.setAngleWithSpeed(1, 90.0, 100);

    // // サーボID 2 に 180度を速度200で設定
    // servoController.setAngleWithSpeed(2, 180.0, 200);

    // // サーボID 3 に 45度を速度150で設定
    // servoController.setAngleWithSpeed(3, 45.0, 150);
}

void loop() {
    servoController.setAngleWithSpeed(1, 90.0, 100);
    c610.update();
}
