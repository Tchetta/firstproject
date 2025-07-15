#include <Arduino.h>
#include "aside.h"
#include "../http/http_app.h"
#include "../gsm/gsm.h"
#include "../mqtt/mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_now/esp_now_handler.h"

extern bool manualAlarmOverride;
extern SensorMuteStatus mutedSensors;
extern bool motionManualOverride;
extern bool motionManuallyEnabled;
extern bool alarmManualOverride;
extern bool alarmManuallyEnabled;

const int alertPin = 2;

// ──────────── WiFi Setup ─────────────
int setup_wifi(const char* ssid, const char* password) {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected.");
    return 0;
  } else {
    Serial.println("WiFi connected.");
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.localIP());
    return 1;
  }
}

// ──────────── LED Blink ─────────────
void blink_internal_led(int count, int delayTime) {
    pinMode(alertPin, OUTPUT);
    for (int i = 0; i < count; i++) {
        digitalWrite(alertPin, HIGH);
        delay(delayTime);
        digitalWrite(alertPin, LOW);
        delay(delayTime);
    }
}

void setBlinkPin() {
    pinMode(alertPin, OUTPUT);
}
void setInternalBlinkPin() {
    pinMode(alertPin, OUTPUT);
}

// ──────────── System Mode Change ─────────────
void setSystemMode(SystemMode newMode) {
  currentSystemMode = newMode;
  preferences.begin("smart_home", false);
  preferences.putUChar(MODE_KEY, (uint8_t)newMode);
  preferences.end();
  Serial.print("System mode set to: ");
  if (newMode == MODE_GSM_ONLY) Serial.println("GSM_ONLY");
  else if (newMode == MODE_WIFI_ONLY) Serial.println("WIFI_ONLY");
  else Serial.println("BOTH");
  ESP.restart();
}

// ──────────── Command Handling ─────────────
void handleIncomingCommand(const String &sender, const String &message) {
    const char* trustedNumbers[] = {
      "+237653997220", "653997220",
      "+237620637397", "620637397"
    };

    bool authorized = false;
    bool fromMQTT = sender == "MQTT_CLIENT";

    if (!fromMQTT) {
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
    }

    String msg = message;
    msg.trim();
    msg.toUpperCase();

    Serial.print("Command (from ");
    Serial.print(sender);
    Serial.print("): ");
    Serial.println(msg);

    // ───── Alarm Control ─────
    if (msg == "ALARM ON") {
        alarmManualOverride = true;
        alarmManuallyEnabled = true;
        digitalWrite(alertPin, HIGH);
        ESPNowHandler_sendCommand("ALARM_ON");
        if (!fromMQTT) sendSMS("ALARM manually ON");
    }
    else if (msg == "ALARM OFF") {
        alarmManualOverride = true;
        alarmManuallyEnabled = false;
        digitalWrite(alertPin, LOW);
        ESPNowHandler_sendCommand("ALARM_OFF");
        if (!fromMQTT) sendSMS("ALARM manually OFF");
    }
    else if (msg == "ALARM AUTO") {
        alarmManualOverride = false;
        if (!fromMQTT) sendSMS("ALARM set to AUTO mode");
    }

    // ───── Sensor Muting ─────
    else if (msg == "MUTE MOTION") mutedSensors.motion = true;
    else if (msg == "UNMUTE MOTION") mutedSensors.motion = false;
    else if (msg == "MUTE FIRE") mutedSensors.fire = true;
    else if (msg == "UNMUTE FIRE") mutedSensors.fire = false;
    else if (msg == "MUTE GAS") mutedSensors.gas = true;
    else if (msg == "UNMUTE GAS") mutedSensors.gas = false;
    else if (msg == "MUTE VOLTAGE") mutedSensors.voltage = true;
    else if (msg == "UNMUTE VOLTAGE") mutedSensors.voltage = false;
    else if (msg == "MUTE TEMPERATURE") mutedSensors.temperature = true;
    else if (msg == "UNMUTE TEMPERATURE") mutedSensors.temperature = false;

    // ───── Motion Override ─────
    else if (msg == "MOTION OFF") {
        motionManualOverride = true;
        motionManuallyEnabled = false;
        if (!fromMQTT) sendSMS("Motion detection turned OFF manually.");
    }
    else if (msg == "MOTION ON") {
        motionManualOverride = true;
        motionManuallyEnabled = true;
        if (!fromMQTT) sendSMS("Motion detection turned ON manually.");
    }
    else if (msg == "MOTION AUTO") {
        motionManualOverride = false;
        if (!fromMQTT) sendSMS("Motion detection back to auto schedule.");
    }

    // ───── Fire System & Mains Control ─────
    else if (msg == "FIRE ON") {
        ESPNowHandler_sendCommand("FIRE_SYSTEM_ON");
        if (!fromMQTT) sendSMS("Fire system activated.");
    }
    else if (msg == "FIRE OFF") {
        ESPNowHandler_sendCommand("FIRE_SYSTEM_OFF");
        if (!fromMQTT) sendSMS("Fire system deactivated.");
    }
    else if (msg == "MAINS OFF") {
        ESPNowHandler_sendCommand("OFF_APPLIANCE");
        if (!fromMQTT) sendSMS("Main supply turned OFF.");
    }
    else if (msg == "MAINS ON") {
        ESPNowHandler_sendCommand("ON_APPLIANCE");
        if (!fromMQTT) sendSMS("Main supply turned ON.");
    }

    // ───── System Mode Switch ─────
    else if (msg == "MODE GSM") {
        setSystemMode(MODE_GSM_ONLY);
        if (!fromMQTT) sendSMS("System mode: GSM_ONLY. Restarting...");
    }
    else if (msg == "MODE WIFI") {
        setSystemMode(MODE_WIFI_ONLY);
        if (!fromMQTT) sendSMS("System mode: WIFI_ONLY. Restarting...");
    }
    else if (msg == "MODE BOTH") {
        setSystemMode(MODE_BOTH);
        if (!fromMQTT) sendSMS("System mode: BOTH. Restarting...");
    } else if (msg == "MODE") {
        if (currentSystemMode == MODE_GSM_ONLY) setDashboardMode("GSM_ONLY_ACTIVE");
        else if (currentSystemMode == MODE_WIFI_ONLY) setDashboardMode("WIFI_ONLY_ACTIVE");
        else setDashboardMode("BOTH_ACTIVE");
        if (!fromMQTT) sendSMS("System mode: BOTH. Restarting...");
    }

    // ───── Restart ─────
    else if (msg == "REBOOT") {
        if (!fromMQTT) sendSMS("Restarting ESP...");
        ESP.restart();
    }

    else {
        Serial.println("Unknown or unsupported command.");
        if (!fromMQTT) sendSMS("Unknown command.");
    }
}

// Optional forwarding to HTTP log server
void myPrint(String text) {
  Serial.println(text);
  sendSerialLogToServer(text);
}
