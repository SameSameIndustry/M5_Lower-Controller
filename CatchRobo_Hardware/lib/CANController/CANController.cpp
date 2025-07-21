#include "CANController.h"

CANController::CANController(int csPin, long baudRate)
    : csPin(csPin), baudRate(baudRate), can(csPin) {}

void CANController::begin() {
    // CAN通信の初期化
    while (CAN_OK != can.begin(MCP_ANY, baudRate, MCP_8MHZ)) {
        Serial.println("CAN BUS init failed. Retrying...");
        delay(100);
    }
    Serial.println("CAN BUS init success.");
}

bool CANController::sendPacket(int id, byte data[8], size_t length) {
    if (length > 8) {
        Serial.println("データ長が8バイトを超えています");
        return false;
    }

    if (can.sendMsgBuf(id, 0, length, data)) {
        Serial.printf("CANパケット送信成功: ID=%d, Data=", id);
        for (size_t i = 0; i < length; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
        return true;
    } else {
        Serial.println("CANパケット送信失敗");
        return false;
    }
}