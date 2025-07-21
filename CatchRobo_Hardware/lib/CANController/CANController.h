#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <M5Stack.h>
#include <mcp_can.h>
#include <SPI.h>

class CANController {
public:
    CANController(int csPin, long baudRate);
    void begin();
    bool sendPacket(int id, byte data[8], size_t length);

private:
    int csPin;
    long baudRate;
    MCP_CAN can;
};

#endif // CAN_CONTROLLER_H