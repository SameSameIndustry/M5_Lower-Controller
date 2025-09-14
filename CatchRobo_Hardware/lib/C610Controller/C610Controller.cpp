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

      float angle_deg = angle_raw * 360 / 8192;
      float delta = 0.0;
      if (speed_raw > 30000 && speed_raw < 65135) { // 逆回転
        if (angle_deg > last_angle[index]) {
          delta = (angle_deg - last_angle[index]) - 360;
        } else {
          delta = (angle_deg - last_angle[index]);
        }
      }
      else if(speed_raw >= 400) {
        if (angle_deg < last_angle[index]) {
          delta = (angle_deg - last_angle[index]) + 360;
        } else {
          delta = (angle_deg - last_angle[index]);
        }
      }
      else {
        float delta = angle_deg - last_angle[index];

        if (delta > 180) delta -= 360.0;
        if (delta < -180) delta += 360.0;
      }

      total_angle[index] += delta;
      last_angle[index] = angle_deg;
    }
  }
}

float C610Controller::getAngle(uint8_t motor_index) {
  if (motor_index < 4) return total_angle[motor_index];
  return 0.0;
}
