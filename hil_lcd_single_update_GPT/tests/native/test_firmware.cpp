#include <cassert>
#include <iostream>
#include <sstream>
#include "../../arduino/asta_display_GPT/asta_display_GPT.ino"

static std::string frame(const std::string &payload) {
  char tail[8]; snprintf(tail, sizeof(tail), "*%04X\n", crc16(payload.c_str()));
  return payload + tail;
}
static void feed(const std::string &bytes) {
  for (char c : bytes) Serial.rx.push_back(c);
  for (unsigned n = 0; Serial.available() && n < 100; ++n) {
    unsigned before = lcd.calls;
    loop();
    assert(lcd.calls - before <= 1);
  }
  assert(!Serial.available());
}
static void send(const std::string &payload) { feed(frame(payload)); }
static void paint() {
  for (unsigned n = 0; n < 100 && lcdOnline && lcdStage != LCD_IDLE; ++n) {
    unsigned before = lcd.calls;
    loop();
    assert(lcd.calls - before <= 1);
  }
  assert(!lcdOnline || lcdStage == LCD_IDLE);
}
static std::string row(unsigned r) { return std::string(lcd.screen[r], 16); }
static std::string padded(const std::string &text) { return text + std::string(16 - text.size(), ' '); }
static void rows(const std::string &a, const std::string &b) {
  paint(); assert(lcd.visible); assert(row(0) == padded("TL1: " + a));
  assert(row(1) == padded("TL2: " + b));
}
static void expectAck(uint32_t seq, uint16_t mask, char kind) {
  std::ostringstream p; p << "1,A,123," << seq << ',' << mask << ',' << kind;
  assert(Serial.tx == frame(p.str())); Serial.tx.clear();
}
static void start() {
  setup(); assert(Serial.tx == frame("1,R,1")); Serial.tx.clear();
  assert(Wire.limit == 3000 && Wire.resetOnTimeout && Wire.hz == 100000);
  assert(lcd.address == 0x27);
  send("1,H,123,1"); expectAck(0, ALL_RED, 'H');
  send("1,P,123,1"); expectAck(1, ALL_RED, 'P');
}
static void state(uint32_t seq, uint32_t sim, unsigned a, unsigned b,
                  unsigned p1 = 2, unsigned p2 = 2) {
  std::ostringstream p; p << "1,S,123," << seq << ',' << sim << ',' << p1 << ',' << p2 << ',' << a << ',' << b;
  send(p.str());
}

int main(int argc, char **argv) {
  assert(argc == 2); std::string test = argv[1];
  assert(crc16("123456789") == 0x29B1);
  if (test == "startup") {
    setup(); assert(Serial.tx == frame("1,R,1"));
    rows("WAITING", "WAITING"); assert(readMask() == ALL_RED);
    Serial.tx.clear(); send("1,H,123,1"); expectAck(0, ALL_RED, 'H');
    rows("READY", "READY");
  } else if (test == "modes") {
    start(); rows("PAUSED", "PAUSED");
    const char *names[] = {"ASTA", "FTSC", "FTSC TRANS"};
    uint32_t seq = 1;
    for (unsigned a = 0; a < 3; ++a) for (unsigned b = 0; b < 3; ++b) {
      ++seq; state(seq, seq * 1000, a, b); expectAck(seq, ALL_RED, 'S');
      rows(names[a], names[b]);
    }
    ++seq; state(seq, seq * 1000, 0, 0); expectAck(seq, ALL_RED, 'S'); rows("ASTA", "ASTA");
    send("1,P,123,12"); expectAck(12, ALL_RED, 'P'); rows("PAUSED", "PAUSED");
    fakeMs += 10000; loop(); assert(!faulted && !watching);
    send("1,X,123,13"); expectAck(13, ALL_RED, 'X'); rows("STOPPED", "STOPPED");
    state(14, 14000, 1, 1); assert(Serial.tx.empty()); rows("STOPPED", "STOPPED");
  } else if (test == "phases") {
    start(); uint32_t seq = 1;
    const uint16_t masks[] = {33, 34, 36, 12, 20, 36};
    for (unsigned a = 0; a < 6; ++a) for (unsigned b = 0; b < 6; ++b) {
      ++seq; state(seq, seq * 1000, 0, 1, a, b);
      uint16_t expected = masks[a] | (masks[b] << 6);
      expectAck(seq, expected, 'S'); assert(readMask() == expected);
    }
    rows("ASTA", "FTSC");
  } else if (test == "duplicate") {
    start(); state(2, 0, 1, 0); expectAck(2, ALL_RED, 'S'); rows("FTSC", "ASTA");
    unsigned writes = pinWrites, calls = lcd.calls; uint32_t accepted = lastAccepted;
    fakeMs += 100;
    state(2, 0, 1, 0); expectAck(2, ALL_RED, 'S');
    assert(pinWrites == writes && lcd.calls == calls && lastAccepted == accepted);
    state(2, 0, 0, 0); assert(Serial.tx.empty());
    assert(pinWrites == writes && lcd.calls == calls && lastAccepted == accepted);
    state(3, 1000, 1, 0); expectAck(3, ALL_RED, 'S');
    assert(lcd.calls == calls); // Same labels on a fresh sample do not repaint.
  } else if (test == "invalid") {
    start(); state(2, 1000, 1, 0); expectAck(2, ALL_RED, 'S'); rows("FTSC", "ASTA");
    uint32_t accepted = lastAccepted; unsigned writes = pinWrites, calls = lcd.calls;
    for (const char *p : {"1,S,123,3,2000,2,2,3,0", "1,S,123,3,2000,6,2,0,0",
                          "1,S,123,3,999,2,2,0,0"}) {
      send(p); assert(Serial.tx == frame("1,N,123,2,STATE")); Serial.tx.clear();
    }
    send("1,S,456,3,2000,2,2,0,0"); assert(Serial.tx.empty());
    std::string bad = frame("1,S,123,3,2000,2,2,0,0"); bad[0] = '2'; feed(bad);
    assert(Serial.tx.empty() && lastAccepted == accepted && pinWrites == writes && lcd.calls == calls);
    rows("FTSC", "ASTA");
  } else if (test == "watchdog" || test == "wrap") {
    start(); if (test == "wrap") fakeMs = UINT32_MAX - 1000;
    state(2, 0, 0, 1, 0, 3); expectAck(2, 801, 'S'); rows("ASTA", "FTSC");
    fakeMs = lastAccepted + 2999U; loop(); assert(!faulted);
    fakeMs = lastAccepted + 3000U; loop(); assert(faulted && readMask() == ALL_RED);
    assert(Serial.tx == frame("1,E,123,2,WATCHDOG")); Serial.tx.clear();
    rows("FAULT", "FAULT");
    fakeMs = flashAt + 500U; loop(); assert(readMask() == 0);
    fakeMs = flashAt + 500U; loop(); assert(readMask() == ALL_RED);
    send("1,H,123,1"); state(3, 1000, 0, 0);
    assert(Serial.tx.empty() && faulted); rows("FAULT", "FAULT");
  } else if (test == "host_fault") {
    start(); state(2, 0, 0, 0); expectAck(2, ALL_RED, 'S'); rows("ASTA", "ASTA");
    send("1,F,123,3"); assert(Serial.tx == frame("1,E,123,3,HOST"));
    assert(faulted && readMask() == ALL_RED); rows("FAULT", "FAULT");
  } else if (test == "gpio_fault") {
    start(); forcedReadPin = 2; state(2, 0, 0, 0);
    assert(faulted && Serial.tx == frame("1,E,123,2,GPIO"));
    forcedReadPin = -1; assert(readMask() == ALL_RED); rows("FAULT", "FAULT");
  } else if (test == "lcd_absent" || test == "lcd_init_timeout") {
    lcd.initStatus = test == "lcd_absent" ? -6 : 0;
    lcd.initTimeout = test == "lcd_init_timeout";
    start(); assert(!lcdOnline);
    state(2, 0, 0, 1, 0, 3); expectAck(2, 801, 'S');
    fakeMs = lastAccepted + 3000U; loop();
    assert(faulted && readMask() == ALL_RED && lcd.calls == 0);
  } else if (test == "lcd_error" || test == "lcd_timeout") {
    start(); rows("PAUSED", "PAUSED");
    lcd.failNext = test == "lcd_error"; lcd.timeoutNext = test == "lcd_timeout";
    trace.clear(); state(2, 0, 1, 0); expectAck(2, ALL_RED, 'S');
    assert(!lcdOnline && !faulted);
    assert(trace[0].find("serial:1,A") == 0); // GPIO ACK precedes LCD I/O.
    unsigned calls = lcd.calls;
    state(3, 1000, 0, 1); expectAck(3, ALL_RED, 'S');
    for (unsigned i = 0; i < 100; ++i) loop();
    assert(lcd.calls == calls);
    fakeMs = lastAccepted + 3000U; loop(); assert(faulted && readMask() == ALL_RED);
  } else if (test == "supersede") {
    start(); rows("PAUSED", "PAUSED");
    state(2, 0, 1, 0); expectAck(2, ALL_RED, 'S'); assert(!lcd.visible);
    for (int i = 0; i < 10; ++i) loop();
    state(3, 1000, 0, 2); expectAck(3, ALL_RED, 'S'); assert(!lcd.visible);
    rows("ASTA", "FTSC TRANS");
    state(4, 2000, 1, 1); expectAck(4, ALL_RED, 'S');
    send("1,F,123,5"); rows("FAULT", "FAULT");
  } else if (test == "serial_priority") {
    start(); rows("PAUSED", "PAUSED");
    state(2, 0, 1, 0); expectAck(2, ALL_RED, 'S');
    unsigned calls = lcd.calls;
    for (int i = 0; i < 90; ++i) Serial.rx.push_back('z');
    loop(); assert(Serial.available() && lcd.calls == calls);
    while (Serial.available()) loop();
    assert(used != 0 && lcd.calls == calls);
    fakeMs = frameAt + 200; loop(); assert(used == 0 && dropping);
    feed("\n"); rows("FTSC", "ASTA");
  } else if (test == "watchdog_guard") {
    start(); rows("PAUSED", "PAUSED");
    state(2, 0, 0, 0); expectAck(2, ALL_RED, 'S');
    fakeMs = lastAccepted + 2950; unsigned calls = lcd.calls; loop();
    assert(lcd.calls == calls && !faulted);
    fakeMs = lastAccepted + 3000; loop(); assert(faulted && readMask() == ALL_RED);
    rows("FAULT", "FAULT");
  } else { assert(false && "unknown scenario"); }
  std::cout << "PASS " << test << '\n';
}
