#include "ODriveCAN.h"

ODriveCAN::ODriveCAN(uint8_t csPin) : can(csPin) {}

bool ODriveCAN::begin(uint8_t can_id1, uint8_t can_id2) {
  byte input_mode[8] = {0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00};  // AxisState = 8 (Closed Loop)
  can.sendMsgBuf(can_id1, 0, 8, input_mode);
//   delay(50);
  byte state_data[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  can.sendMsgBuf(can_id2, 0, 8, state_data);
  return true;
}

void ODriveCAN::setClosedLoop(uint8_t can_id) {
  byte data[8] = {0x08, 0, 0, 0, 0, 0, 0, 0};
  can.sendMsgBuf(can_id, 0, 8, data);
  delay(50);
}

void ODriveCAN::sendFloatCommand(uint16_t can_id, float value1, float value2) {
  union { float f; uint8_t b[4]; } u1, u2;
  u1.f = value1;
  u2.f = value2;
  byte data[8] = {
    u1.b[0], u1.b[1], u1.b[2], u1.b[3],
    u2.b[0], u2.b[1], u2.b[2], u2.b[3]
  };
  can.sendMsgBuf(can_id, 0, 8, data);
}

void ODriveCAN::setPosition(uint8_t can_id, float position) {
  sendFloatCommand(can_id, position, 0.00);
}