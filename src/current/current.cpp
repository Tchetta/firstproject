
#include <Arduino.h>
#include "current.h"
#include "../aside/aside.h"

float R1 = 6800.0;
float R2 = 12000.0;

void setCurrentPin(int pin) {
    pinMode(pin, INPUT);
    // myPrint("Current sensor connects on GPIO" + pin);
}

float currentValue(int pin) {
    int adc = analogRead(pin);
    float adc_voltage = adc * (3.3 / 4096.0);
    float current_voltage = (adc_voltage * (R1+R2)/R2);
    float current = (current_voltage - 2.5) / 0.100;

    return current;
}