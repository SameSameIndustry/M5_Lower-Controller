#include "VescCAN.h"

#define VESC_CMD_DUTY     0x00
#define VESC_CMD_CURRENT  0x01
#define VESC_CMD_ERPM     0x03

VescCAN::VescCAN(MCP_CAN* external_can) {
  can = external_can;
}

bool VescCAN::begin() {
  return true; // MCP_CANの初期化はmain側で行う想定
}

void VescCAN::setDuty(uint8_t vesc_id, float duty) {
  sendFloatCommand(vesc_id, VESC_CMD_DUTY, duty, 100000.0f);
}

void VescCAN::setCurrent(uint8_t vesc_id, float current) {
  sendFloatCommand(vesc_id, VESC_CMD_CURRENT, current, 1.0f);
}

void VescCAN::setERPM(uint8_t vesc_id, float erpm) {
  sendFloatCommand(vesc_id, VESC_CMD_ERPM, erpm, 1.0f);
}

void VescCAN::sendFloatCommand(uint8_t vesc_id, uint8_t cmd_id, float value, float scale) {
  uint32_t scaled = value * scale;
  uint8_t data[4];
  data[0] = static_cast<uint8_t>((scaled >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((scaled >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((scaled >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(scaled & 0xFF);

  // uint32_t can_id = (vesc_id << 8) | cmd_id;
  uint32_t can_id = (cmd_id << 8) | vesc_id;
  can->sendMsgBuf(can_id, 1, 4, data);
}