#include "FeetechServo.h"
#include "ODriveCAN.h"
#include "C610Controller.h"
#include <M5Stack.h>
#include <mcp_can.h>
#include <SPI.h>
#include <AS5600.h>
#include <Wire.h>
#include "CommandProcessor.h"
#include "VescCAN.h"

AS5600 encoder;
const uint8_t TCA_ADDR = 0x70;
const uint8_t muxChannels[] = {0x04, 0x08, 0x10}; // CH0, CH1, CH2

CommandProcessor* processor;

const float encoder_offset = 0.7;  // オフセット角度（必要に応じて調整）

const int r_lim = 2; // can id = 3 == right
const int l_lim = 5; // can id = 2 == left

const int Estop = 35; // Emergency stop pin

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -1.7;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = -3.42; // for node ID 5

const float ax4_lim = -1.02; // limitation for ax4 and ax5

float ax4_target = ax4_lim;
float ax5_target = -1*ax4_target;

float rev = 1.2; // strictry positive

// CAN設定
const int csPin = 12; // SPI CS ピン
MCP_CAN can(csPin); // Set CS to pin 12

ODriveCAN odrive(&can);
C610Controller c610(&can);
VescCAN vesc(&can);

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
uint16_t current2_init = 600;
uint16_t current3_init = 600;

uint16_t current_max = 1000; // current limit

// float rev1 = 0;

const float pitch_offset = 0.4; // ピッチオフセット（必要に応じて調整）
float rev2 = pitch_offset; // 目標値は必ずマイナス
float rev3 = pitch_offset; // 目標値は必ずマイナス

float v_rev2 = 0;
float v_rev3 = 0;
float v_rev1 = 0;

float rev2_offset = 0;
float rev3_offset = 0;
float angle = 0; // 目標値は必ずマイナス

static uint16_t ids[] = {0x202, 0x203, 0x204}; // CAN IDの配列

const float one_rev = 36*2*M_PI; // 1回転あたりの角度（36歯ギア×2πラジアン）

// servoControllerの目標角度
float servo1_angle = 0.0;
float servo2_angle = 0.0;
float servo3_angle = 0.0;

bool rlim_flg = false;
bool llim_flg = false;

int C610_wait = 3000; // C610の初期化待ち時間（ミリ秒）

float now_t = millis();

void selectMuxChannel(uint8_t i) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(2); // 安定化
}

// 🔧 角度取得関数（引数なし、float型の角度を返す）
// float getAS5600Angle() {
//   uint16_t raw = as5600.readAngle();  // 0–4095
//   float degrees = -1 * raw * 2*M_PI / 4096.0 - encoder_offset;

//   // ±180度範囲に収める
//   if (degrees > M_PI) {
//     degrees -= 2*M_PI;
//   } else if (degrees < -1*M_PI) {
//     degrees += 2*M_PI;
//   }

//   return degrees;
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

// void update_C610encoder(){
//     for (int i = 0; i < 3; ++i) {
//         c610.update(ids[i]);  // 各 CAN ID に対してアップデート
//     }
//     rev2 = pitch_offset + (c610.getAngle(1) - rev2_offset)/one_rev*M_PI/60; // 何回転したか
//     rev3 = pitch_offset + (c610.getAngle(2) - rev3_offset)/one_rev*M_PI/60;
    
//     v_rev2 = c610.getSpeed(1)/36*M_PI/60; // rad/s
//     v_rev3 = c610.getSpeed(2)/36*M_PI/60; // rad/s
//     v_rev1 = c610.getSpeed(3)/36/7*M_PI*2; // rad/s
// }

// 受信処理を関数化
void processReceive() {
    processor->receive();  // SET_CMD受信

    // 受信データを取得
    float receivedData_p[8];
    float receivedData_e[8];
    processor->getReceivedData(receivedData_p, receivedData_e);

    // 目標値の更新
    if (-1 * receivedData_p[0] <= -0.05 && ax4_lim <= -1 * receivedData_p[0]) {
        ax4_target = -1 * receivedData_p[0];
        ax5_target = receivedData_p[0];
    } else {
        ax4_target = ax4_lim;
        ax5_target = -1 * ax4_lim;
    }

    // Current and servo angle calculations
    if (0 < rev2) {
        if (abs(receivedData_e[2]) < current2_init + 400) {
            if (receivedData_e[2] > 20) {
                current2 = (rev2 > 0.4) ? 0 : receivedData_e[2];
            } else if (receivedData_e[2] < -20) {
                if(receivedData_e[2] >= -800){
                    current2 = receivedData_e[2];
                }else{
                    current2 = -800;
                }
            } else {
                current2 = 0;
            }
        } else {
            if(receivedData_e[2] > 0){
                if(rev2 > 0.4){
                    current2 = 0;
                }else{
                    current2 = current2_init + 400;
                }
            }else{
                current2 = -800;
            }
        }
    } else {
        current2 = 0;
    }

    if (0 < rev3) {
        if (abs(receivedData_e[3]) < current3_init + 400) {
            if (receivedData_e[3] > 20) {
                current3 = (rev3 > 0.4) ? 0 : receivedData_e[3];
            } else if (receivedData_e[3] < -20) {
                if(receivedData_e[3] >= -800){
                    current3 = receivedData_e[3];
                }else{
                    current3 = -800;
                }
            } else {
                current3 = 0;
            }
        } else {
            if(receivedData_e[3] > 0){
                if(rev3 > 0.4){
                    current3 = 0;
                }else{
                    current3 = current3_init + 400;
                }
            }else{
                current3 = -800;
            }
        }
    } else {
        current3 = 0;
    }

    if(abs(angle) < M_PI*3/4){
        if (abs(receivedData_e[4]) < current_max) {
            current4 = receivedData_e[4];
        }else{
            if(receivedData_e[4] > 0){
                current4 = current_max;
            }else{  
                current4 = -1*current_max;
            }
        }
    } else {
        if (angle > 0){
            if(receivedData_e[4] <=0){
                if (receivedData_e[4] > -1*current_max){
                    current4 = receivedData_e[4];
                }else{
                    current4 = -1*current_max;
                }
            }else{
                current4 = 0;
            }
        }else if (angle < 0){
            if(receivedData_e[4] >=0){
                if (receivedData_e[4] < current_max && receivedData_e[4] >= 0){
                    current4 = receivedData_e[4];
                }else{
                    current4 = current_max;
                }
            }else{
                current4 = 0;
            }
        }else{
            current4 = 0;
        }
    }

    servo1_angle = (receivedData_p[5] > -1.54 && receivedData_p[5] < 1.54) ? receivedData_p[5] : 0.0;
}

// 送信処理を関数化
void processSend() {
    static float p_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    static float e_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // rev2, rev3, angleはグローバル変数なので直接参照
    p_data[2] = rev2;
    p_data[3] = rev3;
    p_data[4] = angle;

    e_data[2] = v_rev2; // rad/s
    e_data[3] = v_rev3; // rad/s
    e_data[4] = v_rev1; // rad/s

    processor->setStateData(p_data, e_data);
    processor->send();  // STATE送信
}

void initialize(){
    
    c610.setCurrents(current1, current2, current3, current4);
    // c610.update();
    // for(int i=0; i<100; i++){
    //     update_C610encoder();
    //     delay(1);
    // }
    
    delay(100);
    
    rev2_offset = c610.getAngle(1);
    rev3_offset = c610.getAngle(2);

    delay(200);
    
    for (int i = 0; i < 5; ++i) {  
        // ODriveの初期設定, C610の初期設定後に行う
        odrive.begin(0x8B, 0x87); // CAN IDを指定して初期化,node ID 4
        delay(5);
        odrive.begin(0xAB, 0xA7); // CAN IDを指定して初期化,node ID 5
        delay(5);
    }
    
    for (int i = 1; i < 3; ++i) {
        odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // ノードID 4 を原点に復帰
        delay(5);
        odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // ノードID 5 を原点に復帰
        delay(5);
    }

    
    servoController.setAngleWithSpeed(1, 0, 100); // angle is always negative
    servoController.setAngleWithSpeed(2, 0, 100); // angle is always negative
    servoController.setAngleWithSpeed(3, 0, 100); // angle is always negative

    M5.Lcd.printf("Initialization done.\n");
    delay(100);
}

void scanI2CDevices(uint8_t channel) {
  delay(2); // 安定化

  M5.Lcd.printf("CH%d:\n", channel);
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      M5.Lcd.printf("  0x%02X found\n", addr);
    }
  }
}


void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on M5Stack
    
    // as5600.begin();  // 初期化（I2Cアドレスはデフォルト0x36）
    processor = new CommandProcessor(Serial);
    
    pinMode(r_lim, INPUT_PULLUP);     // 内部プルアップ有効
    pinMode(l_lim, INPUT_PULLUP);     // 内部プルアップ有効
    pinMode(Estop, INPUT);     // Emergency stop pin


    // CAN通信の初期化
    if (can.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        while (1);
    }
    can.setMode(MCP_NORMAL);
    M5.Lcd.println("CAN Initialized");
    
    servoController.setup();

    if(!digitalRead(Estop)){
        M5.Lcd.println("EMERGENCY STOPPED!");
        while(!digitalRead(Estop)){
            delay(5);
            // angle = getAS5600Angle();
            processSend();
        }
        M5.Lcd.fillScreen(BLACK);
    }
    delay(C610_wait); // C610の初期化待ち
    initialize();
    vesc.begin();


    M5.Lcd.fillScreen(BLACK);
    // c610.setCurrents(current1, current2, current3, current4);
}

void loop() {
    // servoController.setAngleWithSpeed(1, 90.0, 100);
    if (digitalRead(r_lim)){
        rlim_flg = false; // リミットスイッチが押されていない状態
    } else {
        if (!rlim_flg){
            rev3_offset = c610.getAngle(2); // リミットスイッチが押された瞬間の角度をオフセットとして保存
            rlim_flg = true; // リミットスイッチが押された状態
        }
    }

    if (digitalRead(l_lim)){
        llim_flg = false; // リミットスイッチが押されていない状態
    } else {
        if (!llim_flg){
            rev2_offset = c610.getAngle(1); // リミットスイッチが押された瞬間の角度をオフセットとして保存
            llim_flg = true; // リミットスイッチが押された状態
        }
    }
    // rev2_offset = c610.getAngle(1);
    // rev3_offset = c610.getAngle(2);

    
    if (!digitalRead(Estop)) {
        M5.Lcd.setCursor(0, 0);
        current1 = 0;
        current2 = 0;
        current3 = 0;
        current4 = 0;
        c610.setCurrents(current1, current2, current3, current4);
        // M5.Lcd.fillScreen(BLACK);
        // M5.Lcd.setTextColor(RED);
        // M5.Lcd.setTextSize(3);
        processor->resetStateData();  // データをリセット
        rev2 = pitch_offset; // 目標値は必ずマイナス
        rev3 = pitch_offset; // 目標値は必ずマイナス
        v_rev2 = 0;
        v_rev3 = 0;
        v_rev1 = 0;
        M5.Lcd.printf("EMERGENCY STOP!");
        while(!digitalRead(Estop)){
            // angle = getAS5600Angle();
            processSend();
            delay(5);
        }
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.printf("RESTART!");
        ax4_target = ax4_lim;
        ax5_target = -1*ax4_lim;
        processor->resetReceivedData();  // データをリセット
        processor->resetStateData();  // データをリセット
        delay(C610_wait); // C610の初期化待ち
        M5.Lcd.fillScreen(BLACK);
        initialize();
        M5.Lcd.fillScreen(BLACK);
    } else {
        // M5.Lcd.printf("No Emergency");
    }
    
    // delay(5);
    c610.setCurrents(current1, current2, current3, current4);
    delay(2);
    
    odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // move to target ax4
    // delay(1);
    // delay(1);
    odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // move to target ax5
    delay(2);
    // update_C610encoder();
    // delay(1);
    servoController.setAngleWithSpeed(1, servo1_angle, 100); // angle is always negative
    // delay(1);
    // delay(1);

    servo2_angle = M_PI/2 - angle;
    if(servo2_angle < 0 && servo2_angle > 3.14){
        if(servo2_angle > 0){
            servo2_angle = 3.14;
        }else{
            servo2_angle = 0;
        }
    }

    servoController.setAngleWithSpeed(2, servo2_angle, 100); // angle is always negative
    // delay(1);
    // delay(1);
    servoController.setAngleWithSpeed(3, servo3_angle, 100); // angle is always negative
    // delay(2);
    // update_C610encoder();
    // delay(2);
    // M5.Lcd.setCursor(0, 210);
    //M5.Lcd.printf("S1: %.2f rad, %.2f rad, %.2f rad", servo1_angle, servo2_angle, servo3_angle);
    // M5.Lcd.printf("%.4f Hz", 1000/(millis() - now_t));
    // now_t = millis();
    processReceive();
    processSend();
    delay(2);
    // update_C610encoder();

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);

    for (uint8_t i = 2; i < 5; i++) {
        selectMuxChannel(i);
        encoder.begin();
        delay(100);
        uint16_t raw = encoder.readAngle();
        float deg = raw * 360.0 / 4096.0;

        M5.Lcd.printf("CH%d: %6.2f deg\n", i, deg);
    }

    delay(500);  
    //vesc.setERPM(10, 1500.0f);   // 1500 ERPM to VESC ID 1

    vesc.setCurrent(10, 1000.0f);      // 電流指令（Amp）
    delay(1000);

}