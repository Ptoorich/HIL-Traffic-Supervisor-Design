#pragma once
#include <cstdint>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>
#include <utility>

constexpr int LOW = 0, HIGH = 1, OUTPUT = 1;
inline uint32_t fakeMs = 0;
inline int pins[20] = {};
inline int forcedReadPin = -1;
inline unsigned pinWrites = 0;
inline std::vector<std::string> trace;
inline uint32_t millis() { return fakeMs; }
inline void digitalWrite(int pin, int value) { pins[pin] = value; ++pinWrites; }
inline void pinMode(int, int) {}
inline int digitalRead(int pin) { return pin == forcedReadPin ? !pins[pin] : pins[pin]; }
class FakeSerial {
public:
  std::deque<char> rx;
  std::string tx;
  void begin(unsigned long) {}
  int available() const { return (int)rx.size(); }
  int read() { char c = rx.front(); rx.pop_front(); return (unsigned char)c; }
  void print(const char *s) { tx += s; trace.push_back(std::string("serial:") + s); }
};
inline FakeSerial Serial;
