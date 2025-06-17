#ifndef GSM_H
#define GSM_H

#include <Arduino.h>

// Forward declarations for existing functions
void setupGSM();
bool sendSMS(const String& message, const String& number = "653997220");
bool isGSMReady();
bool callNumber(const String& phoneNumber);
void waitForNetwork(unsigned long timeout = 15000);
void updateSerial();
void checkForIncomingSMS();
void deleteLastSMS();
void deleteAllSMS();
void handUPCall();

// New function prototypes for time and location
String getNetworkTime(); // Returns time/date string
String getGsmLocation(); // Returns location string (latitude, longitude)

#endif