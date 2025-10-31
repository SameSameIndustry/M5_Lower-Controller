#pragma once
#include <mcp_can.h>

class VescCAN {
public:
  VescCAN(MCP_CAN* external_can);
  bool begin();

  void setDuty(uint8_t vesc_id, float duty);
  void setCurrent(uint8_t vesc_id, float current);
  void setERPM(uint8_t vesc_id, float erpm);

private:
  MCP_CAN* can;

  void sendFloatCommand(uint8_t vesc_id, uint8_t cmd_id, float value, float scale);
};