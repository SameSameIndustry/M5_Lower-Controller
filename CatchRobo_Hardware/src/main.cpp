#include "FeetechServo.h"

// サーボ設定
const std::vector<byte> servoIDs = {1, 2, 3}; // 複数のサーボID
const int centerPosition = 2048;
const float maxAmplitude = 1600.0;
const float radiansIncrement = 0.06;

// UART2 設定
HardwareSerial SerialSTS(2); // UART2
ServoController servoController(SerialSTS, servoIDs, centerPosition, maxAmplitude, radiansIncrement);

void setup() {
    servoController.setup();

    // サーボID 1 に 90度を速度100で設定
    servoController.setAngleWithSpeed(1, 90.0, 100);

    // サーボID 2 に 180度を速度200で設定
    servoController.setAngleWithSpeed(2, 180.0, 200);

    // サーボID 3 に 45度を速度150で設定
    servoController.setAngleWithSpeed(3, 45.0, 150);
}

void loop() {
    servoController.update();
}