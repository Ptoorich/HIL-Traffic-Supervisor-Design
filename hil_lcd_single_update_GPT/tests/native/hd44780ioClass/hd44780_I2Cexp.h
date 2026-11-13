#pragma once
#include "Arduino.h"
#include "Wire.h"
#include <algorithm>
class hd44780_I2Cexp {
public:
  uint8_t address;
  int initStatus = 0;
  bool initTimeout = false, failNext = false, timeoutNext = false, visible = true;
  unsigned calls = 0, displays = 0;
  uint32_t costMs = 1;
  uint8_t row = 0, col = 0;
  char screen[2][16];
  explicit hd44780_I2Cexp(uint8_t addr) : address(addr) {
    std::fill(&screen[0][0], &screen[0][0] + 32, ' ');
  }
  int begin(int cols, int rows) {
    if (cols != 16 || rows != 2) return -10;
    Wire.timeout = initTimeout; return initStatus;
  }
  bool op(const char *name) {
    ++calls; trace.push_back(std::string("lcd:") + name); fakeMs += costMs;
    if (timeoutNext) { Wire.timeout = true; timeoutNext = false; return false; }
    if (failNext) { failNext = false; return false; }
    return true;
  }
  int noDisplay() { if (!op("hide")) return -1; visible = false; return 0; }
  int display() { if (!op("show")) return -1; visible = true; ++displays; return 0; }
  int setCursor(uint8_t c, uint8_t r) {
    if (!op("cursor")) return -1;
    col = c; row = r; return 0;
  }
  size_t write(uint8_t v) {
    if (!op("char")) return 0;
    if (row >= 2 || col >= 16) return 0;
    screen[row][col++] = (char)v; return 1;
  }
};
