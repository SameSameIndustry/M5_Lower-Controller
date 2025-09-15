#include "FeetechServo.h"
#include "ODriveCAN.h"
#include "C610Controller.h"
#include <M5Stack.h>
#include <mcp_can.h>
#include <SPI.h>
#include <AS5600.h>
#include <Wire.h>
#include "CommandProcessor.h"

CommandProcessor* processor;

AS5600 as5600;
const float encoder_offset = 0.7;  // オフセット角度（必要に応じて調整）

const int r_lim = 2; // can id = 3 == right
const int l_lim = 5; // can id = 2 == left

const int Estop = 35; // Emergency stop pin

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -1.7;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = -3.42; // for node ID 5

float ax4_target = 0.0f;
float ax5_target = 0.0f;

float ax4_lim = -1.02; // limitation for ax4 and ax5

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

uint16_t current_max = 1000; // current limit

// float rev1 = 0;
float rev2 = 0; // 目標値は必ずマイナス
float rev3 = 0; // 目標値は必ずマイナス

float rev2_offset = 0;
float rev3_offset = 0;
float angle = 0; // 目標値は必ずマイナス

static uint16_t ids[] = {0x202, 0x203};

const float one_rev = 36*2*M_PI; // 1回転あたりの角度（36歯ギア×2πラジアン）
const float pitch_offset = 0.4; // ピッチオフセット（必要に応じて調整）

// 🔧 角度取得関数（引数なし、float型の角度を返す）
float getAS5600Angle() {
  uint16_t raw = as5600.readAngle();  // 0–4095
  float degrees = -1 * raw * 2*M_PI / 4096.0 - encoder_offset;

  // ±180度範囲に収める
  if (degrees > M_PI) {
    degrees -= 2*M_PI;
  } else if (degrees < -1*M_PI) {
    degrees += 2*M_PI;
  }

  return degrees;
}

// 受信データをLCDに表示する関数
// void CANUpdateTask(void* pvParameters) {
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   const TickType_t xFrequency = 1; // 1ms周期

//   while (true) {
//     c610.update();
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }

// }

void CANUpdateTask(void* pvParameters) {
  uint16_t target_id = *((uint16_t*)pvParameters);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 5;  // 3ms周期

  while (true) {
    c610.update(target_id);
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void receiveTask(void* pvParameters) {
  while (true) {
    processor->receive();  // SET_CMD受信

    // 受信データを取得してLCDに表示
    float receivedData_p[8];
    float receivedData_e[8];
    processor->getReceivedData(receivedData_p, receivedData_e);
    
    // displayReceivedData(receivedData_p, 8);

    if (-1*receivedData_p[0]<=0 && ax4_lim <= -1*receivedData_p[0]) {
        ax4_target = -1*receivedData_p[0]; // 目標値は必ずマイナス
        ax5_target = receivedData_p[0]; // 目標値は必ずマイナス
    }else {
        ax4_target = ax4_lim; // 目標値は必ずマイナス
        ax5_target = -1*ax4_lim; // 目標値は必ずマイナス
    }

    if(0 < rev2 && rev2 <= 0.41){
        if (abs(receivedData_e[2]) < current_max*0.5) {
            if (receivedData_e[2] > 0){
                current2 = receivedData_e[2] + 300;
            }else if(receivedData_e[2] < 0){
                current2 = receivedData_e[2] - 300;
            }else{
                current2 = 0;
            }
        }else{
            if (receivedData_e[2] > 0){
                current2 = current_max;
            }else{
                current2 = -1*current_max;
            }
        }
    } else {
        current2 = 0;
    }
    
    if(0 < rev3 && rev3 <= 0.41){
        if (abs(receivedData_e[3]) < current_max*0.5) {
            if (receivedData_e[3] > 0){
                current3 = receivedData_e[3] + 300;
            }else if(receivedData_e[3] < 0){
                current3 = receivedData_e[3] - 300;
            }else{
                current3 = 0;
            }
        }else{
            if (receivedData_e[3] > 0){
                current3 = current_max;
            }else{
                current3 = -1*current_max;
            }
        }
    } else {
        current3 = 0;
    }

    if(abs(angle) < M_PI/2){
        if (abs(receivedData_e[4]) < current_max) {
            current1 = receivedData_e[4];
        }else{
            if(receivedData_e[4] > 0){
                current1 = current_max;
            }else{  
                current1 = -1*current_max;
            }
        }
    } else {
        current1 = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 10
  }
}

void sendTask(void* pvParameters) {
    static float p_data[8] = {0,0,0,0,0,0,0,0};
    //static float e_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    while (true) {
        // rev2, rev3, angleはグローバル変数なので直接参照できる
        p_data[2] = rev2;
        p_data[3] = rev3;
        p_data[4] = angle;

        processor->setStateData(p_data);
        processor->send();  // STATE送信

        vTaskDelay(pdMS_TO_TICKS(10));  // 100ms周期
    }
}

void initialize(){
    if(digitalRead(r_lim) && digitalRead(l_lim)) {
        current3 = current3_init + 300;
        current2 = current2_init + 300; // 少し強めに
    }else if (!digitalRead(r_lim)){
        current3 = 0;
        current2 = current2_init + 300; // 少し強めに
    }else if (!digitalRead(l_lim)){
        current3 = current3_init + 300;
        current2 = 0; // 少し強めに
    }
    c610.setCurrents(current1, current2, current3, current4);
    delay(10);

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
        
    current1 = 0;
    current2 = 0;
    current3 = 0;
    
    c610.setCurrents(current1, current2, current3, current4);
    // c610.update();
    
    delay(1000);
    
    rev2_offset = c610.getAngle(1);
    rev3_offset = c610.getAngle(2);

    

    // ODriveの初期設定
    odrive.begin(0x8B, 0x87); // CAN IDを指定して初期化,node ID 4
    delay(10);
    odrive.begin(0xAB, 0xA7); // CAN IDを指定して初期化,node ID 5
    delay(10);


    // odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // ノードID 4 を原点に復帰
    // delay(10);
    // odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // ノードID 5 を原点に復帰

    ax5_target = 1.02f;
    ax4_target = -1*ax5_target;

    delay(100); // 少し待つ
    odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // ノードID 4 を原点に復帰
    delay(10);
    odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // ノードID 5 を原点に復帰

    M5.Lcd.printf("Initialization done.\n");
}

void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on M5Stack
    as5600.begin();  // 初期化（I2Cアドレスはデフォルト0x36）
    processor = new CommandProcessor(Serial);
    
    xTaskCreatePinnedToCore(receiveTask, "RX", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(sendTask, "TX", 4096, NULL, 1, NULL, 0);
    
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
    // xTaskCreatePinnedToCore(
    //     CANUpdateTask,       // タスク関数
    //     "CANUpdate",         // タスク名
    //     4096,                // スタックサイズ（バイト）
    //     NULL,                // 引数（不要ならNULL）
    //     1,                   // 優先度（0〜5）
    //     NULL,                // タスクハンドル（不要ならNULL）
    //     0                    // 実行するコア（0または1）
    // );
    for (int i = 0; i < 2; ++i) {
        xTaskCreatePinnedToCore(
        CANUpdateTask,
        "CANUpdateTask",
        2048,
        &ids[i],
        1,
        NULL,
        1
        );
    }


    initialize();
    
    servoController.setup();
    M5.Lcd.fillScreen(BLACK);

    // c610.setCurrents(current1, current2, current3, current4);
}

void loop() {
    // servoController.setAngleWithSpeed(1, 90.0, 100);
    M5.Lcd.setCursor(0, 30);
    if (!digitalRead(Estop)) {
        current1 = 0;
        current2 = 0;
        current3 = 0;
        c610.setCurrents(current1, current2, current3, current4);
        // M5.Lcd.fillScreen(BLACK);
        // M5.Lcd.setTextColor(RED);
        // M5.Lcd.setTextSize(3);
        // M5.Lcd.printf("EMERGENCY STOP!");
        while(!digitalRead(Estop)){
            delay(100);
        }
        initialize();
    } else {
        M5.Lcd.printf("No Emergency");
    }
    
    angle = getAS5600Angle();

    
    // M5.Lcd.fillRect(0, 30, 320, 30, BLACK);
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("Angle: %.4f rad", angle);

    rev2 = pitch_offset + (c610.getAngle(1) - rev2_offset)/one_rev*M_PI/60; // 何回転したか
    rev3 = pitch_offset + (c610.getAngle(2) - rev3_offset)/one_rev*M_PI/60;

    M5.Lcd.setCursor(30, 90);
    M5.Lcd.printf("Rev2: %.4f rad", rev2);
    M5.Lcd.setCursor(30, 120);
    M5.Lcd.printf("Rev3: %.4f rad", rev3);

    // if (rev2 > 0.35) {
    //     current2 = -650;
    // } else {
    //     current2 = 0;
    // }
    // if (rev3 > 0.35) {
    //     current3 = -650;
    // } else {
    //     current3 = 0;
    // }
    // current1 = 0;
    delay(5);
    // c610.setCurrents(current1, current2, current3, current4);
    c610.setCurrents(current1, current2, current3, current4);

    M5.Lcd.setCursor(0, 150);
    M5.Lcd.printf("Cur1: %d mA", current1);
    M5.Lcd.setCursor(0, 180);
    M5.Lcd.printf("Cur2: %d mA", current2);
    M5.Lcd.setCursor(0, 210);
    M5.Lcd.printf("Cur3: %d mA", current3);
    
    delay(10);  // wait for responsiveness
    
    odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // move to target ax4
    delay(5);
    odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // move to target ax5
    delay(5);
}