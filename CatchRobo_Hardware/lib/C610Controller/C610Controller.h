#pragma once
#include <mcp_can.h>
#include <Arduino.h>

class C610Controller {
public:
  C610Controller(MCP_CAN* can_ptr);

  void setCurrents(int16_t current1, int16_t current2, int16_t current3, int16_t current4);
  void update(uint16_t CANID);  // 受信処理（必要なら拡張）
  
  float getAngle(uint8_t motor_index);  // 0〜3
  float getSpeed(uint8_t motor_index);  // 0〜3

private:
  MCP_CAN* can;

  float last_angle[4] = {0, 0, 0, 0};
  float total_angle[4] = {0, 0, 0, 0};
  float current_speed[4] = {0, 0, 0, 0};
  int pre_diff[4] = {0, 0, 0, 0};

};