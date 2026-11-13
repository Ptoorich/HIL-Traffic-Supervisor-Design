#pragma once
#include <cstdint>
class FakeWire {
public:
  bool timeout = false, resetOnTimeout = false;
  uint32_t limit = 0, hz = 0;
  void begin() {}
  void setClock(uint32_t value) { hz = value; }
  void setWireTimeout(uint32_t value, bool reset) { limit = value; resetOnTimeout = reset; timeout = false; }
  bool getWireTimeoutFlag() { return timeout; }
  void clearWireTimeoutFlag() { timeout = false; }
};
inline FakeWire Wire;
