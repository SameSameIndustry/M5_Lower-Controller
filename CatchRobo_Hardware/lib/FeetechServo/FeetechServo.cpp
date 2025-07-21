#include "FeetechServo.h"

ServoController::ServoController(HardwareSerial& serial, const std::vector<byte>& servoIDs, int centerPosition, float maxAmplitude, float radiansIncrement)
    : serial(serial), servoIDs(servoIDs), centerPosition(centerPosition), maxAmplitude(maxAmplitude), radiansIncrement(radiansIncrement), radiansVal(0.0) {}

void ServoController::setup() {
    M5.begin();
    Serial.begin(115200);
    serial.begin(1000000, SERIAL_8N1, 36, 26); // TX=26, RX=36

    delay(200);
    Serial.println("サーボスイング制御開始（UART2, 1Mbps）");

    // 画面初期化
    M5.Lcd.setRotation(1);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(40, 20);
    M5.Lcd.print("Servo Swing Monitor");
}

void ServoController::update() {
    radiansVal += radiansIncrement;
    if (radiansVal > TWO_PI) radiansVal -= TWO_PI;

    int targetPos = centerPosition + int(sin(radiansVal) * maxAmplitude);

    // 複数のサーボIDに対して同時に制御を行う
    for (byte id : servoIDs) {
        moveToPos(id, targetPos);
    }

    // デバッグ出力
    Serial.printf("Target Pos: %d\n", targetPos);

    // ディスプレイに表示
    M5.Lcd.fillRect(40, 60, 240, 40, BLACK);  // 前の値を消去
    M5.Lcd.setCursor(40, 60);
    M5.Lcd.printf("Pos: %d", targetPos);

    delay(20);
}

void ServoController::setAngleWithSpeed(byte id, float angle, int speed) {
    // 角度をサーボの位置に変換 (角度は0～300度、位置は0～4095)
    int position = centerPosition + int((angle / 300.0) * 4095.0 - 2048);
    moveToPos(id, position, speed);

    // デバッグ出力
    Serial.printf("Set Angle: %.2f, Position: %d, Speed: %d, ID: %d\n", angle, position, speed, id);
}

void ServoController::moveToPos(byte id, int position, int speed) {
    byte packet[13];

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 9;
    packet[4] = 3;
    packet[5] = 42;
    packet[6] = position & 0xFF;
    packet[7] = (position >> 8) & 0xFF;
    packet[8] = speed & 0xFF;         // 速度の下位バイト
    packet[9] = (speed >> 8) & 0xFF;  // 速度の上位バイト
    packet[10] = 0x00;
    packet[11] = 0x00;

    byte checksum = 0;
    for (int i = 2; i < 12; i++) checksum += packet[i];
    packet[12] = ~checksum;

    for (int i = 0; i < 13; i++) serial.write(packet[i]);
}