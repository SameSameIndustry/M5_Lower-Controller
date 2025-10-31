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
  float values[24];
  int index = 0;
  int last = 0;

  for (int i = 0; i < line.length(); ++i) {
    if (line[i] == ',' || i == line.length() - 1) {
      String token = line.substring(last, (i == line.length() - 1) ? i + 1 : i);
      token.trim();
      if (index >= 1 && index <= 24) {
        values[index - 1] = token.toFloat();
      }
      index++;
      last = i + 1;
    }
  }

  if (index != 25) return false;

  for (int i = 0; i < 8; ++i) {
    received_p[i] = values[i];
    received_e[i] = values[i + 8];
    received_v[i] = values[i + 16];  // v_vals are not provided in SET_CMD
  }
  return true;
}

void CommandProcessor::send() {
  serial.print("STATE_FULL,8");
  for (int i = 0; i < 8; ++i) {
    serial.print(",");
    serial.print(state_p[i], 4);
  }
  for (int i = 0; i < 8; ++i) {
    serial.print(",");
    serial.print(state_e[i], 4);
  }
  for (int i = 0; i < 8; ++i) {
    serial.print(",");
    serial.print(state_v[i], 4);
  }
  serial.println();
}

void CommandProcessor::setStateData(const float* p_vals, const float* e_vals, const float* v_vals) {
  for (int i = 0; i < 8; ++i) {
    state_p[i] = p_vals[i];
  }
  for (int i = 0; i < 8; ++i) {
    state_e[i] = e_vals[i];
  }
  for (int i = 0; i < 8; ++i) {
    state_v[i] = v_vals[i];
  }
}

void CommandProcessor::getStateData(float* p_out, float* e_out, float* v_out) {
  for (int i = 0; i < 8; ++i) {
    p_out[i] = state_p[i];
    e_out[i] = state_e[i];
    v_out[i] = state_v[i];
  }
}

void CommandProcessor::getReceivedData(float* p_out, float* e_out, float* v_out) {
  for (int i = 0; i < 8; ++i) {
    p_out[i] = received_p[i];
    e_out[i] = received_e[i];
    v_out[i] = received_v[i];
  }
}

void CommandProcessor::resetReceivedData() {
  // received_p を初期値にリセット
  float default_p[8] = {1.02, -1.02, 0, 0, 0, 0, 0, 0};
  memcpy(received_p, default_p, sizeof(received_p));

  // received_e を初期値にリセット
  memset(received_e, 0, sizeof(received_e));
  // received_v を初期値にリセット
  memset(received_v, 0, sizeof(received_v));
}
void CommandProcessor::resetStateData() {
  // received_p を初期値にリセット
  float default_p[8] = {0,0,0.4,0.4,0,0,0,0};
  memcpy(state_p, default_p, sizeof(received_p));

  // received_e を初期値にリセット
  memset(state_e, 0, sizeof(received_e));
  // received_v を初期値にリセット
  memset(state_v, 0, sizeof(received_v));
}