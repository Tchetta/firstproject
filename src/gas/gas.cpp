#include <MQUnifiedsensor.h>
#include "gas.h"
#include "../gsm/gsm.h"
#include "../aside/aside.h"

#define Board "ESP-32"
#define Voltage_Resolution 5
#define ADC_Bit_Resolution 12
#define MQ2_PIN 34
#define RatioMQ2CleanAir 9.83

// GAS NAMES (used in alerts)
const char* gasNames[] = { "LPG", "CO", "Smoke" };

// GAS READ FUNCTIONS
float (*gasReaders[])() = { readGasLPG, readGasCO, readGasSmoke };

// ALERT THRESHOLDS in ppm
const float gasThresholds[] = { 300.0, 50.0, 100.0 };

// ALERTING STATE to avoid spam
bool lastAlerted[] = { false, false, false };

// Number of gases
const int gasCount = sizeof(gasNames) / sizeof(gasNames[0]);

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ2_PIN, "MQ-2");

// ========== Initialization ========== //
void initGasSensor() {
  MQ2.setRegressionMethod(1); // Exponential
  MQ2.init();

  float calcR0 = 0;
  for (int i = 0; i < 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    delay(500);
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("✅ Gas sensor ready!");
  // myPrint("✅ Gas sensor ready!");
}

// ========== Gas Reading Functions ========== //
float readGasLPG() {
  MQ2.setA(574.25); MQ2.setB(-2.222); // LPG
  MQ2.update();
  return MQ2.readSensor();
}

float readGasCO() {
  MQ2.setA(36974); MQ2.setB(-3.109); // CO
  MQ2.update();
  return MQ2.readSensor();
}

float readGasSmoke() {
  MQ2.setA(1000000); MQ2.setB(-3.333); // Smoke
  MQ2.update();
  return MQ2.readSensor();
}

// ========== SMS Helper ========== //
void sendGasAlert(int gasIndex, float ppm, bool isHigh) {
  char message[120];
  if (isHigh) {
    snprintf(message, sizeof(message),
             "[GAS ALERT] %s level high: %.2f ppm", gasNames[gasIndex], ppm);
  } else {
    snprintf(message, sizeof(message),
             "[GAS] %s level normalized: %.2f ppm", gasNames[gasIndex], ppm);
  }
  // sendSMS(message);
}

// ========== Main Detection Handler ========== //
void handleGasDetection() {
  Serial.println("");
  for (int i = 0; i < gasCount; i++) {
    float ppm = gasReaders[i]();

    Serial.print(gasNames[i]);
    Serial.print(": ");
    Serial.print(ppm);
    Serial.print(" ppm");

    bool isAboveThreshold = ppm > gasThresholds[i];

    if (isAboveThreshold && !lastAlerted[i]) {
      sendGasAlert(i, ppm, true);
      lastAlerted[i] = true;
      Serial.print(" [ALERT]");
    } else if (!isAboveThreshold && lastAlerted[i]) {
      sendGasAlert(i, ppm, false);
      lastAlerted[i] = false;
    }

    Serial.println();
    delay(200); // Avoid ADC noise overlap
  }
}

