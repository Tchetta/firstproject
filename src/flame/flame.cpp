#include <Arduino.h>

void setupFlame() {
  analogSetAttenuation(ADC_11db);
}

int analogFlamePercent(int AO_PIN) {
  int infrared_value = analogRead(AO_PIN);


  int percent = map(infrared_value, 700, 4005, 100, 0);

  return percent;

}