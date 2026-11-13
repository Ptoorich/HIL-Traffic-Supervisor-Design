// Standalone test: one 16x2 LCD at 0x27, one row per junction.
// Install "hd44780" by Bill Perry from Arduino Library Manager.
// These are TEST labels, not live supervisor modes. All traffic heads stay red.
// This replaces the running HIL sketch temporarily; leave Python HIL stopped.
#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd(0x27);

void setup() {
  for (uint8_t pin = 2; pin <= 13; ++pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  const uint8_t reds[] = {4, 7, 10, 13};
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(reds[i], HIGH);

  Serial.begin(115200);
  delay(1500);
  Serial.println(F("Standalone LCD test. All traffic heads remain red."));

  Wire.begin();
  Wire.setClock(100000UL);
  Wire.setWireTimeout(25000UL, true);

  int status = lcd.begin(16, 2);
  if (status != 0 || Wire.getWireTimeoutFlag()) {
    Serial.print(F("LCD initialization failed. Status: "));
    Serial.println(status);
    Serial.print(F("Wire timeout: "));
    Serial.println(Wire.getWireTimeoutFlag() ? F("yes") : F("no"));
    return;
  }

  bool ok = (lcd.backlight() == 0);
  ok &= (lcd.setCursor(0, 0) == 0);
  ok &= (lcd.print(F("TL1: ASTA [TEST]")) == 16);
  ok &= (lcd.setCursor(0, 1) == 0);
  ok &= (lcd.print(F("TL2: FTSC [TEST]")) == 16);

  if (!ok || Wire.getWireTimeoutFlag()) {
    Serial.println(F("LCD write failed. Check wiring with USB disconnected."));
    return;
  }
  Serial.println(F("Test text sent. Please verify both rows on the LCD."));
}

void loop() {
  // No simulation, mode switching, or HIL serial commands in this test.
}
