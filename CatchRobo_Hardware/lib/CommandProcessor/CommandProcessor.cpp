#include "CommandProcessor.h"

CommandProcessor::CommandProcessor(Stream& serialRef) : serial(serialRef) {}

void CommandProcessor::receive() {
  if (serial.available()) {
    String line = serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("SET_CMD")) {
      parseCommand(line);
    }
  }
}

bool CommandProcessor::parseCommand(const String& line) {
  float values[16];
  int index = 0;
  int last = 0;

  for (int i = 0; i < line.length(); ++i) {
    if (line[i] == ',' || i == line.length() - 1) {
      String token = line.substring(last, (i == line.length() - 1) ? i + 1 : i);
      token.trim();
      if (index >= 1 && index <= 16) {
        values[index - 1] = token.toFloat();
      }
      index++;
      last = i + 1;
    }
  }

  if (index != 17) return false;

  for (int i = 0; i < 8; ++i) {
    received_p[i] = values[i];
    received_e[i] = values[i + 8];
  }
  return true;
}

void CommandProcessor::send() {
  serial.print("STATE,8");
  for (int i = 0; i < 8; ++i) {
    serial.print(",");
    serial.print(state_p[i], 4);
  }
  // for (int i = 0; i < 8; ++i) {
  //   serial.print(",");
  //   serial.print(state_e[i], 4);
  // }
  serial.println();
}

void CommandProcessor::setStateData(const float* p_vals) {
  for (int i = 0; i < 8; ++i) {
    state_p[i] = p_vals[i];
  }
}

void CommandProcessor::getStateData(float* p_out, float* e_out) {
  for (int i = 0; i < 8; ++i) {
    p_out[i] = state_p[i];
    e_out[i] = state_e[i];
  }
}

void CommandProcessor::getReceivedData(float* p_out, float* e_out) {
  for (int i = 0; i < 8; ++i) {
    p_out[i] = received_p[i];
    e_out[i] = received_e[i];
  }
}