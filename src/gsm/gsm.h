#ifndef GSM_H
#define GSM_H

#include <Arduino.h>

// void powerOnGSM();
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

#endif
