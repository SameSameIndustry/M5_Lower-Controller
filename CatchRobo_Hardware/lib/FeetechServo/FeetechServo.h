#ifndef FEETECH_SERVO_H
#define FEETECH_SERVO_H

#include <Arduino.h>
#include <M5Stack.h>
#include <vector>

class ServoController {
public:
    ServoController(HardwareSerial& serial, const std::vector<byte>& servoIDs, int centerPosition, float maxAmplitude, float radiansIncrement);
    void setup();
    void update();
    void setAngleWithSpeed(byte id, float angle, int speed); // サーボID、角度、速度を指定して制御

private:
    void moveToPos(byte id, int position, int speed = 0);

    HardwareSerial& serial;
    std::vector<byte> servoIDs; // 複数のサーボIDを保持
    int centerPosition;
    float maxAmplitude;
    float radiansIncrement;
    float radiansVal;
};

#endif // FEETECH_SERVO_H