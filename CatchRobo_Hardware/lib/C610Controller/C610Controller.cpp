#include "C610Controller.h"

C610Controller::C610Controller(MCP_CAN* can_ptr) {
  can = can_ptr;
}

void C610Controller::setCurrents(int16_t current1, int16_t current2, int16_t current3, int16_t current4) {
  byte txData[8];
  txData[0] = (current1 >> 8) & 0xFF;
  txData[1] = current1 & 0xFF;
  txData[2] = (current2 >> 8) & 0xFF;
  txData[3] = current2 & 0xFF;
  txData[4] = (current3 >> 8) & 0xFF;
  txData[5] = current3 & 0xFF;
  txData[6] = (current4 >> 8) & 0xFF;
  txData[7] = current4 & 0xFF;

  can->sendMsgBuf(0x200, 0, 8, txData);  // 一括送信（ID 0x200）
}


void C610Controller::update() {
  if (CAN_MSGAVAIL == can->checkReceive()) {
    unsigned long canId;
    byte len;
    byte buf[8];
    can->readMsgBuf(&canId, &len, buf);

    uint16_t base_id = canId & 0x7FF;
    if (base_id >= 0x201 && base_id <= 0x204) {
      uint8_t index = base_id - 0x201;
      uint16_t angle_raw = (buf[0] << 8) | buf[1];
      uint16_t speed_raw = (buf[2] << 8) | buf[3];
      int16_t current_angle = reinterpret_cast<int16_t &>(angle_raw);
      int16_t current_speed_val = reinterpret_cast<int16_t &>(speed_raw);

      int diff = current_angle - last_angle[index];
      if (diff > 4096) {
        diff -= 8192;
      } else if (diff < -4096) {
        diff += 8192;
      }
      if (pre_diff[index] > 2048 && current_speed_val > 10 && diff < 0) { // ノイズ対策
        diff += 8192;
      }
      else if (pre_diff[index] < -2048 && current_speed_val < -10 && diff > 0) { // ノイズ対策
        diff -= 8192;
      }
      total_angle[index] += diff / 8192.0 * 2 * M_PI;
      last_angle[index] = current_angle;
      pre_diff[index] = diff;
    }
  }
}

float C610Controller::getAngle(uint8_t motor_index) {
  if (motor_index < 4) return total_angle[motor_index];
  return 0.0;
}
