#include "gsm.h"
#include "../aside/aside.h"

const char simPIN[]   = "";
String SMS_Message;

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024

#include <Wire.h>
#include <TinyGsmClient.h>

// TTGO T-Call pin definitions
#define MODEM_RST        5
#define MODEM_PWKEY      4
#define MODEM_POWER_ON   23
#define MODEM_TX         27
#define MODEM_RX         26
#define I2C_SDA          21
#define I2C_SCL          22

#define SerialAT Serial1
#define SerialMon Serial

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool callActive = false;
unsigned long callStartTime = 0;
const unsigned long callDuration = 30000; // 30 seconds

char smsBuffer[250]; // Buffer to store SMS messages

bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  Wire.write(en ? 0x37 : 0x35);
  return Wire.endTransmission() == 0;
}

void powerOnGSM() {
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
}

void setupGSM() {
  Wire.begin(I2C_SDA, I2C_SCL);
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  powerOnGSM();

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  SerialMon.println("Initializing modem...");
  modem.restart();
  SerialMon.println("Modem restart finished!");

  int simStatus = modem.getSimStatus();
  if (simStatus != 3) {
    modem.simUnlock(""); // Add your PIN if needed
  }

  waitForNetwork();

  SerialMon.println("SIM status: " + String(simStatus));
  SerialMon.print("Signal quality: ");
  SerialMon.println(modem.getSignalQuality());

  SerialMon.println("Setting SMS mode to text...");
  modem.sendAT("+CMGF=1");
  updateSerial();
  delay(100);
  if (!modem.waitResponse(1000)) {
    SerialMon.println("Failed to set SMS mode.");
    return;
  }

  SerialAT.println("AT+CNMI=2,2,0,0,0"); // Decides how newly arrived SMS messages should be handled
  updateSerial();

  SerialMon.println("Selecting SMS storage...");
  modem.sendAT("+CPMS=\"SM\",\"SM\",\"SM\"");
  if (!modem.waitResponse(1000)) {
    SerialMon.println("Failed to select SMS storage.");
    return;
  }

  SerialMon.println("Deleting all messages...");
  deleteAllSMS();  // 🧹 Clean all messages at boot

  SerialMon.println("Done!");

  // modem.sendAT("+CLIP=1"); // Enable Caller ID
  // modem.waitResponse(1000); // Not critical if it fails
}

void waitForNetwork(unsigned long timeout) {
  SerialMon.print("Connecting");
  unsigned long start = millis();
  while (!modem.isNetworkConnected() && millis() - start < timeout) {
    delay(1000);
    SerialMon.print(".");
  }
  SerialMon.println();

  if (modem.isNetworkConnected()) {
    SerialMon.println("Connected to network!");
    blink_internal_led(3, 200);
  } else {
    SerialMon.println("Still not connected.");
  }
}


bool sendSMS(const String& message, const String& number) {
  return modem.sendSMS(number, message);
}

bool isGSMReady() {
  return modem.isNetworkConnected();
}

bool callNumber(const String& phoneNumber) {
  SerialMon.print("Calling " + phoneNumber + "...");
  // myPrint("Calling " + phoneNumber + "...");

  SerialMon.println(phoneNumber);
  if (modem.callNumber(phoneNumber)) {
    SerialMon.println("Call initiated.");
    callActive = true;
    callStartTime = millis();
    return true;
  } else {
    SerialMon.println("Failed to initiate call.");
    sendSMS("Failed to initiate call.");
    // myPrint("Failed to initiate call.");
    return false;
  }
}

void handUPCall() {
  modem.callHangup();
  SerialMon.println("Call ended.");
  sendSMS("Call ended.");
  // myPrint("Call ended.");
}

void updateSerial()
{
  while (Serial.available())
  {
    SerialAT.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (SerialAT.available())
  {
    Serial.write(SerialAT.read());//Forward what Software Serial received to Serial Port
  }
}

void checkForIncomingSMS() {
  static String currentLine = "";
  static String senderNumber = "";
  static bool expectingMessage = false;

  while (SerialAT.available()) {
    char c = SerialAT.read();

    // Debugging: Print all characters as they arrive
    // Serial.println(c); 

    if (c == '\n') {
      currentLine.trim();

      if (expectingMessage) {
        String receivedMessage = currentLine;
        Serial.println("Sender: " + senderNumber);
        Serial.println("Message: " + receivedMessage);
        receivedMessage.toCharArray(smsBuffer, sizeof(smsBuffer));
        handleIncomingCommand(senderNumber, receivedMessage);
        deleteLastSMS(); // Delete after handling
        expectingMessage = false;
      } else if (currentLine.startsWith("+CMT:")) {
        // Extract the sender number
        int firstQuote = currentLine.indexOf('"');
        int secondQuote = currentLine.indexOf('"', firstQuote + 1);
        senderNumber = currentLine.substring(firstQuote + 1, secondQuote);
        Serial.println("Expecting message from: " + senderNumber); // Debugging
        expectingMessage = true;
      }

      currentLine = "";
    } else {
      currentLine += c;
    }
  }
}


void deleteLastSMS() {
  // Deletes SMS at index 1 (works in this setup since new SMS always lands at index 1 in direct mode)
  SerialAT.println("AT+CMGD=1");
  delay(100);
}

void deleteAllSMS() {
  // Loop over a reasonable range to ensure cleanup
  for (int i = 1; i <= 5; i++) {
    SerialAT.print("AT+CMGD=");
    SerialAT.println(i);
    delay(100);
  }
}

/**
 * @brief Retrieves the current date and time from the GSM network using TinyGSM's built-in function.
 *
 * @return String containing the network time, or "N/A" if unable to retrieve.
 */
String getNetworkTime() {
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Cannot get network time: Not connected to GSM network.");
    return "N/A";
  }

  SerialMon.println("Attempting to get network time using modem.getGSMDateTime(DATE_FULL)...");
  // Use TinyGSMDateTimeFormat::DATE_FULL for full date and time string
  String timeStr = modem.getGSMDateTime(TinyGSMDateTimeFormat::DATE_FULL);

  if (!timeStr.isEmpty()) { // Check if the returned string is not empty
    SerialMon.println("Network Time: " + timeStr);
    return timeStr;
  } else {
    SerialMon.println("Failed to get network time.");
    return "N/A";
  }
}

/**
 * @brief Retrieves the approximate location (latitude and longitude) using
 * GSM cell tower information (LBS - Location Based Services) via TinyGSM's built-in function.
 * Note: This is NOT precise GPS data from satellites. Accuracy varies greatly.
 * Requires modem to be connected to network.
 *
 * @return String containing location details (e.g., "Lat: X.XXXX, Lng: Y.YYYY") or "N/A" if not available.
 * The format depends on what the 0-argument getGsmLocation() function returns.
 */
String getGsmLocation() {
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Cannot get GSM location: Not connected to GSM network.");
    return "N/A";
  }

  SerialMon.println("Attempting to get GSM location using modem.getGsmLocation() (0-argument version)...");
  // The compiler feedback clearly showed String getGsmLocation() as a valid overload.
  // This function typically returns a formatted string like "+CIPGSMLOC: <stat>,<lon>,<lat>,<date>,<time>"
  // or a more user-friendly string depending on the TinyGSM version.
  String locStr = modem.getGsmLocation();

  if (!locStr.isEmpty()) {
    SerialMon.println("GSM Location: " + locStr);
    // You might need to parse 'locStr' further if you want individual lat/lon/accuracy/timestamp values.
    // For now, we'll return the raw string provided by the modem.
    return locStr;
  } else {
    SerialMon.println("Failed to get GSM location or LBS not supported by network.");
    return "N/A";
  }
}