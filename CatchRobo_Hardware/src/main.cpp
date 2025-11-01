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

const int Estop = 35; // Emergency stop pin

// ODriveのオフセット値（適宜調整してください）
const float offset_ax4 = -1.7;//-6.76f;// 1.62f; //-6.95f; // for node ID 4
const float offset_ax5 = -3.42; // for node ID 5

const float ax4_lim = -1.02; // limitation for ax4 and ax5

float ax4_target = ax4_lim;
float ax5_target = -1*ax4_target;

// float rev = 1.2; // strictry positive

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
float current2 = 0;
float current3 = 0;
uint16_t current4 = 0;

uint16_t current_max = 1000; // current limit
uint16_t vel_max = 2000; // velocity limit

// AS5600 角度
float rev[] = {0.0, 0.0, 0.0}; // rad
float v_rev[] = {0.0, 0.0, 0.0}; // rad/s
const float rev_offset[] = {-138, -15.7, -228};
const float rev_div[] = {180 / M_PI, 10.0 * 180 / M_PI, 10.0 * 180 / M_PI};
const int rev_inv[] = {-1, 1, -1};
float pre_time[] = {0.0, 0.0, 0.0};
float pre_rev[] = {0.0, 0.0, 0.0};

static uint16_t ids[] = {0x202, 0x203, 0x204}; // CAN IDの配列

const float one_rev = 36*2*M_PI; // 1回転あたりの角度（36歯ギア×2πラジアン）

// servoControllerの目標角度
float servo1_angle = 0.0;
float servo2_angle = 0.0;
float servo3_angle = 0.0;

int C610_wait = 3000; // C610の初期化待ち時間（ミリ秒）

float now_t = millis();

void selectMuxChannel(uint8_t i) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  delay(1); // 安定化
}

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
    if (0 < rev[1]) {
        if (abs(receivedData_e[2]) < vel_max) {
            if (receivedData_e[2] > 20) {
                current2 = (rev[1] > 0.4) ? 0 : receivedData_e[2];
            } else if (receivedData_e[2] < -20) {
                if(receivedData_e[2] >= -1*vel_max){
                    current2 = receivedData_e[2];
                }else{
                    current2 = -1*vel_max;
                }
            } else {
                current2 = 0;
            }
        } else {
            if(receivedData_e[2] > 0){
                if(rev[1] > 0.4){
                    current2 = 0;
                }else{
                    current2 = vel_max;
                }
            }else{
                current2 = -1*vel_max;
            }
        }
    } else {
        current2 = 0;
    }

    if (0 < rev[2]) {
        if (abs(receivedData_e[3]) < vel_max) {
            if (receivedData_e[3] > 20) {
                current3 = (rev[2] > 0.4) ? 0 : receivedData_e[3];
            } else if (receivedData_e[3] < -20) {
                if(receivedData_e[3] >= -1*vel_max){
                    current3 = receivedData_e[3];
                }else{
                    current3 = -1*vel_max;
                }
            } else {
                current3 = 0;
            }
        } else {
            if(receivedData_e[3] > 0){
                if(rev[2] > 0.4){
                    current3 = 0;
                }else{
                    current3 = vel_max;
                }
            }else{
                current3 = -800;
            }
        }
    } else {
        current3 = 0;
    }

    if(abs(rev[0]) < M_PI/2){
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
        if (rev[0] > 0){
            if(receivedData_e[4] <=0){
                if (receivedData_e[4] > -1*current_max){
                    current4 = receivedData_e[4];
                }else{
                    current4 = -1*current_max;
                }
            }else{
                current4 = 0;
            }
        }else if (rev[0] < 0){
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
    p_data[2] = rev[1];
    p_data[3] = rev[2];
    p_data[4] = rev[0];

    e_data[2] = v_rev[1]; // rad/s
    e_data[3] = v_rev[2]; // rad/s
    e_data[4] = v_rev[0]; // rad/s

    processor->setStateData(p_data, e_data);
    processor->send();  // STATE送信
}

void initialize(){
    
    c610.setCurrents(0, 0, 0, current4);

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
    delay(100);
    
    vesc.begin();

    M5.Lcd.printf("Initialization done.\n");
    delay(100);
}

void Update_Encode(){
    for (uint8_t i = 0; i < 3; i++) {
        selectMuxChannel(i + 2); // CH2, CH3, CH4を選択
        encoder.begin();
        delay(1); // 安定化待ち
        float raw = encoder.readAngle();
        float deg = rev_inv[i] * raw * 360.0 / 4096.0 - rev_offset[i];
        deg = deg / rev_div[i];    
        if(deg > M_PI){
            rev[i] = deg - 2*M_PI;
        }else if(deg < -1*M_PI){
            rev[i] = deg + M_PI;
        } else{
            rev[i] = deg;
        }
        v_rev[i] = (rev[i] - pre_rev[i]) / ((millis() - pre_time[i]) / 1000.0);
        pre_rev[i] = rev[i];
        pre_time[i] = millis();
    }
}

void E_Stop_Process(){
    M5.Lcd.setCursor(0, 0);
    current1 = 0;
    current2 = 0;
    current3 = 0;
    current4 = 0;
    c610.setCurrents(0, 0, 0, current4);
    // M5.Lcd.fillScreen(BLACK);
    // M5.Lcd.setTextColor(RED);
    // M5.Lcd.setTextSize(3);
    processor->resetStateData();  // データをリセット
    M5.Lcd.printf("EMERGENCY STOP!");
    while(!digitalRead(Estop)){
        Update_Encode();
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
}

void setup() {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(10, 10);
    Serial.begin(115200);
    Wire.begin();  // SDA=21, SCL=22 on M5Stack
    
    // as5600.begin();  // 初期化（I2Cアドレスはデフォルト0x36）
    processor = new CommandProcessor(Serial);
    
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
            Update_Encode();
            processSend();
        }
        M5.Lcd.fillScreen(BLACK);
    }
    delay(C610_wait); // C610の初期化待ち
    initialize();

    M5.Lcd.fillScreen(BLACK);
}

void loop() {
    if (!digitalRead(Estop)) {
        E_Stop_Process();
    } else {
        Update_Encode();
    }

    c610.setCurrents(0, 0, 0, current4);
    delay(2);
    
    odrive.setPosition(0x8C, ax4_target/M_PI*4 + (offset_ax4+1.3));  // move to target ax4
    odrive.setPosition(0xAC, ax5_target/M_PI*4 + (offset_ax5-1.3));  // move to target ax5
    delay(2);
    servoController.setAngleWithSpeed(1, servo1_angle, 100); // angle is always negative

    servo2_angle = M_PI/2 - rev[0];
    if(servo2_angle < 0 && servo2_angle > 3.14){
        if(servo2_angle > 0){
            servo2_angle = 3.14;
        }else{
            servo2_angle = 0;
        }
    }

    vesc.setERPM(10, -1 * current2);      // 電流指令（Amp）
    vesc.setERPM(20, -1 * current3);      // 電流指令（Amp）

    delay(2);

    servoController.setAngleWithSpeed(2, servo2_angle, 100); // angle is always negative
    servoController.setAngleWithSpeed(3, servo3_angle, 100); // angle is always negative
    processReceive();
    processSend();
    delay(2);
}