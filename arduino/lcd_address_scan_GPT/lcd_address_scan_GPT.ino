// Temporary UNO I2C scanner. Holds the four traffic heads at red.
// This is NOT the HIL serial-protocol sketch; do not run Python HIL with it.
#include <Arduino.h>
#include <Wire.h>



void setup() {
  for (uint8_t pin = 2; pin <= 13; ++pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  const uint8_t reds[] = {4, 7, 10, 13};
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(reds[i], HIGH);

  Serial.begin(115200);
  delay(1500);
  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(25000UL, true);

  Serial.println(F("Scan ONE LCD at a time. Traffic outputs: all red."));
  uint8_t found = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(address);
    uint8_t status = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag() || status == 5) {
      Serial.println(F("I2C TIMEOUT: disconnect USB and check wiring."));
      return;
    }
    if (status == 0) {
      Serial.print(F("Found I2C device at 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    } else if (status != 2) {
      Serial.print(F("I2C error "));
      Serial.print(status);
      Serial.print(F(" at 0x"));
      Serial.println(address, HEX);
    }
  }
  Serial.print(F("Devices found: "));
  Serial.println(found);
  if (found == 0) Serial.println(F("Disconnect USB and check GND, 5V, A4 and A5."));
  Serial.println(F("Scan complete. Press RESET to scan again."));
}

void loop() {
  // Keep the traffic outputs steady red. No HIL commands are processed here.
}
