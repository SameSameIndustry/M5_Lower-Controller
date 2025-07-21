#include "OdriveController.h"

OdriveCANController::OdriveCANController(MCP_CAN& canInterface, int nodeId)
    : odrive(wrap_can_intf(canInterface), nodeId), canInterface(canInterface), nodeId(nodeId) {}

void OdriveCANController::begin(long baudRate) {
    if (canInterface.begin(MCP_ANY, baudRate, MCP_8MHZ) != CAN_OK) {
        Serial.println("CAN initialization failed!");
        while (true); // 停止
    }
    Serial.println("CAN initialized successfully.");
}

void OdriveCANController::setPosition(float position, float velocityFeedforward) {
    odrive.setPosition(position, velocityFeedforward);
    Serial.printf("Set position: %.2f, velocity feedforward: %.2f\n", position, velocityFeedforward);
}

void OdriveCANController::setVelocity(float velocity) {
    odrive.setVelocity(velocity);
    Serial.printf("Set velocity: %.2f\n", velocity);
}

void OdriveCANController::setTorque(float torque) {
    odrive.setTorque(torque);
    Serial.printf("Set torque: %.2f\n", torque);
}

void OdriveCANController::calibrate() {
    Serial.println("Starting calibration...");
    odrive.setState(ODriveAxisState::AXIS_STATE_MOTOR_CALIBRATION);
    delay(1000);
    odrive.setState(ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
    delay(1000);
    Serial.println("Calibration completed.");
}

void OdriveCANController::enableClosedLoopControl() {
    odrive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    Serial.println("Enabled closed-loop control.");
}

void OdriveCANController::disableControl() {
    odrive.setState(ODriveAxisState::AXIS_STATE_IDLE);
    Serial.println("Disabled control.");
}