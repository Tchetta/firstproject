#ifndef HTTP_H
#define HTTP_H

#include <Arduino.h>

void setupESPApp(const char* ssid, const char* password);
void handleClientESPApp();
void checkLEDStateFromServer();
void sendSerialLogToServer(String message);
void setLedState(String state);
void sendSensorReadings(bool motion, int fire, float temperature, float humidity, float gasLPG, float gasCO, float gasSmoke);

#endif
