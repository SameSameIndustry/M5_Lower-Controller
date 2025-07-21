#ifndef ODRIVE_CONTROLLER_H
#define ODRIVE_CONTROLLER_H

#include <Arduino.h>
#include <ODriveCAN.h>
#include <mcp_can.h> // MCP2515 CAN通信用ライブラリ

class OdriveCANController {
public:
    OdriveCANController(MCP_CAN& canInterface, int nodeId);
    void begin(long baudRate);
    void setPosition(float position, float velocityFeedforward = 0.0f);
    void setVelocity(float velocity);
    void setTorque(float torque);
    void calibrate();
    void enableClosedLoopControl();
    void disableControl();

private:
    ODriveCAN odrive;
    MCP_CAN& canInterface;
    int nodeId;
};

#endif // ODRIVE_CAN_CONTROLLER_H