#include <Arduino.h>
#include "aside.h"
#include "../http/http_app.h"
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
// const char* ssid = "Redmi 13C";
// const char* password = "Rodel2.0";

int setup_wifi(const char* ssid, const char* password) {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    return 1;
  } else {
    return 0;
  }
}

void blink_internal_led(int count, int delayTIme) {
    pinMode(13, OUTPUT);

    int i = 0;
    while (i != count) {
        digitalWrite(13, HIGH);
        delay(delayTIme);
        digitalWrite(13, LOW);
        delay(delayTIme);

        i++;
    }
}

void setBlinkPin() {
    pinMode(13, OUTPUT);
}
void setInternalBlinkPin() {
    pinMode(13, OUTPUT);
}

void setSystemMode(SystemMode newMode) {
  currentSystemMode = newMode;
  preferences.begin("smart_home", false); // Open NVS namespace
  preferences.putUChar(MODE_KEY, (uint8_t)newMode); // Save as uint8_t
  preferences.end();
  Serial.print("System mode set to: ");
  if (newMode == MODE_GSM_ONLY) Serial.println("GSM_ONLY");
  else if (newMode == MODE_WIFI_ONLY) Serial.println("WIFI_ONLY");
  else Serial.println("BOTH");
  // Optionally reboot here or prompt user to reboot for full effect
  ESP.restart();
}

  void handleIncomingCommand(const String &sender, const String &message) {
    // This is a placeholder for `handleIncomingCommand`
    // You'd ideally make this a global function or pass a function pointer
    // to your GSM and MQTT modules.
    // For now, let's just show how it would affect the mode.

    // Define trusted numbers (if applicable for SMS, less so for MQTT unless authenticated)
    const char* trustedNumbers[] = {
      "+237653997220",
      "653997220",
      "+237620637397",
      "620637397"
    };
    bool authorized = false;
    // Check for authorization if from SMS
    if (sender != "MQTT_CLIENT") { // A simple way to differentiate for now
        for (const char* number : trustedNumbers) {
            if (sender == String(number)) {
                authorized = true;
                break;
            }
        }
        if (!authorized) {
            Serial.println("Unauthorized sender. Ignoring command.");
            return;
        }
    } else { // Assume MQTT commands are authorized if they reach here (e.g. from your dashboard)
        authorized = true;
    }

    Serial.print("Received Command (from ");
    Serial.print(sender);
    Serial.print("): ");
    Serial.println(message);

    if (message.equalsIgnoreCase("ALARM ON")) {
        Serial.println("Turning ALARM ON...");
        // Call your existing setLedState("on") or similar function
        // setLedState("on"); // Ensure this function is globally accessible or passed
        digitalWrite(13, HIGH);
    } else if (message.equalsIgnoreCase("ALARM OFF")) {
        Serial.println("Turning ALARM OFF...");
        // setLedState("off");
        digitalWrite(13, LOW);
    } else if (message.equalsIgnoreCase("REBOOT")) {
      Serial.println("Rebooting...");
      ESP.restart();
    } else if (message.equalsIgnoreCase("MODE GSM")) {
      setSystemMode(MODE_GSM_ONLY);
      myPrint("Mode set to GSM only. Rebooting...");
      ESP.restart();
    } else if (message.equalsIgnoreCase("MODE WIFI")) {
      setSystemMode(MODE_WIFI_ONLY);
      myPrint("Mode set to WiFi only. Rebooting...");
      ESP.restart();
    } else if (message.equalsIgnoreCase("MODE BOTH")) {
      setSystemMode(MODE_BOTH);
      myPrint("Mode set to Both. Rebooting...");
      ESP.restart();
    } else {
      Serial.println("Unknown command.");
    }
    // Also, remember to send feedback via SMS or MQTT as appropriate
}

void myPrint(String text) {
  Serial.println(text);
  sendSerialLogToServer(text);
}