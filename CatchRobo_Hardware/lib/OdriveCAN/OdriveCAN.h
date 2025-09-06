#pragma once
#include <mcp_can.h>

class ODriveCAN {
public:
  ODriveCAN(uint8_t csPin);
  bool begin(uint8_t can_id1, uint8_t can_id2);
  void setPosition(uint8_t can_id, float position);

private:
  MCP_CAN can;
  void setClosedLoop(uint8_t can_id);
  void sendFloatCommand(uint16_t can_id, float value1, float value2);
};