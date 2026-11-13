#pragma once
#include <cstdint>
#include <cstring>
class FakeEEPROM {
public:
  uint8_t bytes[32] = {};
  template<class T> void get(int at, T &value) { std::memcpy(&value, bytes + at, sizeof(T)); }
  template<class T> void put(int at, const T &value) { std::memcpy(bytes + at, &value, sizeof(T)); }
};
inline FakeEEPROM EEPROM;
