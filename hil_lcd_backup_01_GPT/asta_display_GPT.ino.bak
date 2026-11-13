// UNO R3 logical-phase display. Protocol and bench instructions: ARDUINO_HIL_GUIDE_GPT.md.
// EEPROM bytes 0..7 store a signature and boot counter for stale-session rejection.
#include <Arduino.h>
#include <EEPROM.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint16_t ALL_RED = (1U << 2) | (1U << 5) | (1U << 8) | (1U << 11);
static const uint32_t WATCHDOG_MS = 3000, FRAME_MS = 200, FLASH_MS = 500;
static const size_t CAPACITY = 160;
static char input[CAPACITY], previous[CAPACITY];
static size_t used = 0;
static bool dropping = false, faulted = false, flashing = true, redOn = true;
static bool acceptedHello = false, watching = false, closed = false;
static uint32_t bootId = 0, sessionId = 0, sequence = 0, lastAccepted = 0;
static uint32_t flashAt = 0, frameAt = 0, lastSim = 0;
static bool haveSim = false;
static uint16_t lastMask = ALL_RED;
static char lastKind = 'H';

static uint16_t crc16(const char *s) {
  uint16_t crc = 0xFFFF;
  while (*s) {
    crc ^= (uint16_t)(uint8_t)*s++ << 8;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void sendPayload(const char *payload) {
  char tail[7];
  snprintf(tail, sizeof(tail), "*%04X\n", crc16(payload));
  Serial.print(payload);
  Serial.print(tail);
}

static void errorReply(const char *code, char kind = 'N') {
  char reply[90];
  snprintf(reply, sizeof(reply), "1,%c,%lu,%lu,%s", kind,
           (unsigned long)sessionId, (unsigned long)sequence, code);
  sendPayload(reply);
}

// Break before make across ALL twelve outputs, including old red lamps.
static void applyMask(uint16_t mask) {
  for (uint8_t i = 0; i < 12; ++i) digitalWrite(i + 2, LOW);
  for (uint8_t i = 0; i < 12; ++i)
    if (mask & (1U << i)) digitalWrite(i + 2, HIGH);
}

static uint16_t readMask() {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < 12; ++i)
    if (digitalRead(i + 2) == HIGH) mask |= 1U << i;
  return mask;
}

static void enterFault(const char *code) {
  if (faulted) return;
  faulted = true; watching = false; flashing = true; redOn = true;
  flashAt = millis();
  applyMask(ALL_RED);
  errorReply(code, 'E');
  // Latched until board reset. HELLO cannot silently rearm a fault.
}

static void serviceTimers() {
  uint32_t now = millis();
  if (watching && (uint32_t)(now - lastAccepted) >= WATCHDOG_MS) enterFault("WATCHDOG");
  if (flashing && (uint32_t)(now - flashAt) >= FLASH_MS) {
    redOn = !redOn; flashAt = now;
    applyMask(redOn ? ALL_RED : 0);
  }
  if (used && (uint32_t)(now - frameAt) >= FRAME_MS) {
    used = 0; dropping = true;  // Discard through LF, never accept a timed-out fragment.
  }
}

static bool number(const char *s, uint32_t &out) {
  if (!*s) return false;
  uint32_t value = 0;
  for (; *s; ++s) {
    if (*s < '0' || *s > '9') return false;
    uint8_t digit = *s - '0';
    if (value > (UINT32_MAX - digit) / 10UL) return false;
    value = value * 10 + digit;
  }
  out = value; return true;
}

static void acknowledge() {
  char reply[90];
  uint16_t actual = readMask();
  if (actual != lastMask) { enterFault("GPIO"); return; }
  snprintf(reply, sizeof(reply), "1,A,%lu,%lu,%u,%c",
           (unsigned long)sessionId, (unsigned long)sequence, actual, lastKind);
  sendPayload(reply);
}

static uint16_t junctionMask(uint8_t phase) {
  // G/Y/R pin order for A then B. Both all-red stages remain distinct on the wire.
  static const uint8_t masks[6] = {33, 34, 36, 12, 20, 36};
  return masks[phase];
}

static void handleLine() {
  char *star = strchr(input, '*');
  if (!star || strlen(star + 1) != 4) return;
  uint16_t expected = 0;
  for (char *p = star + 1; *p; ++p) {
    if (!((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F'))) return;
    expected = (expected << 4) | (*p <= '9' ? *p - '0' : *p - 'A' + 10);
  }
  *star = 0;
  if (crc16(input) != expected) return;
  if (faulted) return;  // Discard traffic after a fault; do not block flashing with replies.
  char work[CAPACITY]; strcpy(work, input);
  char *fields[10]; uint8_t count = 1;
  fields[0] = work;
  for (char *p = work; *p; ++p) {
    if (*p == ',') {
      if (count >= 10) return;
      *p = 0; fields[count++] = p + 1;
    }
  }
  if (count < 4 || strcmp(fields[0], "1") || strlen(fields[1]) != 1) return;
  uint32_t sid, seq;
  if (!number(fields[2], sid) || !sid || !number(fields[3], seq)) return;
  char kind = fields[1][0];
  if (kind == 'H') {
    if (count != 4 || seq != bootId) return;
    if (acceptedHello) {
      if (!closed && sid == sessionId && sequence == 0 && !strcmp(previous, input)) acknowledge();
      return;
    }
    sessionId = sid; sequence = 0; lastKind = 'H'; lastMask = ALL_RED;
    acceptedHello = true; watching = true; flashing = false;
    lastAccepted = millis(); applyMask(ALL_RED);
    strcpy(previous, input); acknowledge(); return;
  }
  if (!acceptedHello || sid != sessionId) return;
  if (seq == sequence && !strcmp(previous, input)) {
    acknowledge(); return;  // No output reapplication and no watchdog refresh.
  }
  if ((closed && kind != 'F') || seq <= sequence) return;
  uint16_t mask = ALL_RED;
  uint32_t sim = lastSim;
  if (kind == 'S') {
    uint32_t p1, p2, m1, m2;
    if (count != 9 || !number(fields[4], sim) || !number(fields[5], p1) ||
        !number(fields[6], p2) || !number(fields[7], m1) || !number(fields[8], m2) ||
        p1 > 5 || p2 > 5 || m1 > 2 || m2 > 2 || (haveSim && sim < lastSim)) {
      errorReply("STATE"); return;
    }
    mask = junctionMask(p1) | (junctionMask(p2) << 6);
  } else if ((kind != 'P' && kind != 'X' && kind != 'F') || count != 4) {
    errorReply("COMMAND"); return;
  }
  sequence = seq;
  if (kind == 'F') { enterFault("HOST"); return; }
  applyMask(mask);
  if (readMask() != mask) { enterFault("GPIO"); return; }
  lastMask = mask; lastKind = kind; flashing = false;
  lastAccepted = millis(); watching = kind == 'S';
  if (kind == 'S') { lastSim = sim; haveSim = true; }
  if (kind == 'X') closed = true;
  strcpy(previous, input); acknowledge();
}

void setup() {
  for (uint8_t pin = 2; pin <= 13; ++pin) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  applyMask(ALL_RED); flashAt = millis();
  Serial.begin(115200);
  const uint32_t signature = 0x41535441UL;
  uint32_t storedSignature, storedBoot;
  EEPROM.get(0, storedSignature); EEPROM.get(4, storedBoot);
  if (storedSignature != signature) storedBoot = 0;
  if (storedBoot == UINT32_MAX) { enterFault("BOOT_COUNTER"); return; }
  bootId = storedBoot + 1;
  EEPROM.put(4, bootId); EEPROM.put(0, signature);
  EEPROM.get(4, storedBoot); EEPROM.get(0, storedSignature);
  if (storedBoot != bootId || storedSignature != signature) {
    enterFault("EEPROM"); return;
  }
  char reply[40];
  snprintf(reply, sizeof(reply), "1,R,%lu", (unsigned long)bootId);
  sendPayload(reply);
}

void loop() {
  serviceTimers();
  // Bound each pass so input floods cannot starve watchdog/flash service.
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    serviceTimers();
    int byte = Serial.read();
    if (byte == '\n') {
      if (!dropping && used) { input[used] = 0; handleLine(); }
      used = 0; dropping = false;
    } else if (!dropping) {
      if (byte < 32 || byte > 126 || used >= CAPACITY - 1) {
        used = 0; dropping = true;
      } else {
        if (!used) frameAt = millis();
        input[used++] = (char)byte;
      }
    }
  }
}
