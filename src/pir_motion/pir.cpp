// pir.cpp
#include "pir.h"

static int pirPin;
static bool motionDetected = false;
static unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 500;  // Check every 500 ms

void setupPIR(int pin) {
  pirPin = pin;
  pinMode(pirPin, INPUT);
  delay(10000);  // Warm-up delay
  Serial.println("<PIR> Sensor ready after warm-up.");
}

bool isMotionDetected() {
  if (millis() - lastCheckTime >= checkInterval) {
    lastCheckTime = millis();
    motionDetected = digitalRead(pirPin) == HIGH;
    if (motionDetected) {
      // Serial.println("<PIR> Motion detected!");
    } else {
      Serial.println("<PIR> No motion.");
    }
  }
  return motionDetected;
}
