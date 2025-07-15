#include "mqtt.h"
#include "../aside/aside.h" // For myPrint, if used
#include "esp_now/esp_now_handler.h"
// Make sure to include the file where the mode management function will reside,
// or define a global variable for the current mode.
// For now, we'll assume a global 'currentSystemMode' can be updated or a function call.

// Assuming manualAlarmOverride is declared as extern in aside.h
extern bool manualAlarmOverride;

// MQTT Broker details - moved to .cpp and made flexible
const char* mqtt_server = "192.168.179.240"; // e.g., "192.168.1.100"
const int mqtt_port = 1883;

// MQTT Topics
const char* MQTT_PUBLISH_TOPIC_SENSORS = "smart_home/sensors";
const char* MQTT_SUBSCRIBE_TOPIC_COMMANDS = "smart_home/commands";
const char* MQTT_PUBLISH_TOPIC_MODE = "smart_home/mode_status"; // New topic for mode updates
const char* MQTT_PUBLISH_TOPIC_INFO = "smart_home/info"; // New topic for info updates

WiFiClient espClient;
PubSubClient client(espClient);

// Global for MQTT connection state (optional, can be checked via client.connected())
static bool mqttConnected = false;
static unsigned long lastReconnectAttempt = 0;
const long RECONNECT_INTERVAL = 5000; // Try to reconnect every 5 seconds

// Forward declaration for local use
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void handleMqttCommand(const String& message); // New handler for MQTT commands

const int alertPin = 2;

/* // Function to initialize MQTT client
void mqttSetup(const char* server, int port) {
  client.setServer(server, port);
  client.setCallback(mqtt_callback); // Use the new callback name
  Serial.println("MQTT client setup complete.");
} */

// Function to initialize MQTT client - MODIFIED TO ACCEPT IPAddress
void mqttSetup(IPAddress serverIp, int port) {
  client.setServer(serverIp, port); // Use the IPAddress object directly
  client.setCallback(mqtt_callback); // Use the new callback name
  Serial.print("MQTT client setup complete for broker IP: ");
  Serial.println(serverIp);
}

void mqttReconnect() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL) {
      lastReconnectAttempt = now;
      Serial.print("Attempting MQTT connection...");
      String clientId = "ESP32-SmartHome-" + String(random(0xffff), HEX);

      if (client.connect(clientId.c_str())) {
        Serial.println("connected!");
        mqttConnected = true;
        // Publish initial status and subscribe
        client.publish(MQTT_PUBLISH_TOPIC_MODE, "ESP32 online!"); // Publish that we are online
        client.subscribe(MQTT_SUBSCRIBE_TOPIC_COMMANDS);
        Serial.println("Subscribed to " + String(MQTT_SUBSCRIBE_TOPIC_COMMANDS));

        // --- IMPORTANT: Inform dashboard about current mode ONLY AFTER MQTT IS CONNECTED ---
        // Access currentSystemMode from main.cpp via extern declared in aside.h
        extern SystemMode currentSystemMode; // Re-declare extern if not already globally accessible
        if (currentSystemMode == MODE_GSM_ONLY) setDashboardMode("GSM_ONLY_ACTIVE");
        else if (currentSystemMode == MODE_WIFI_ONLY) setDashboardMode("WIFI_ONLY_ACTIVE");
        else setDashboardMode("BOTH_ACTIVE");

      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" - will retry...");
        mqttConnected = false;
      }
    }
  } else {
    mqttConnected = true; // Ensure state is true if already connected
  }
}

 // This function must be called continuously in your main loop()
void mqttLoop() {
  mqttReconnect(); // Ensure connection is maintained
  client.loop();   // Process incoming/outgoing MQTT messages
}

// Callback function for incoming MQTT messages
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Msg arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  if (strcmp(topic, MQTT_SUBSCRIBE_TOPIC_COMMANDS) == 0) {
    handleMqttCommand(message);
  }
}

// New function to handle incoming MQTT commands
void handleMqttCommand(const String& message) {
  // This is where you will add logic for commands from the dashboard
  // For example:
  handleIncomingCommand("MQTT_CLIENT", message);
    /* if (message.equalsIgnoreCase("ALARM ON")) {
        Serial.println("MQTT: Turning ALARM ON...");
        manualAlarmOverride = true; // Set manual override
        digitalWrite(alertPin, HIGH);
    } else if (message.equalsIgnoreCase("ALARM OFF")) {
        Serial.println("MQTT: Turning ALARM OFF...");
        manualAlarmOverride = false; // Clear manual override
        digitalWrite(alertPin, LOW);
    } else if (message.equalsIgnoreCase("MAINS ON")) {
        Serial.println("MQTT: Turning MAIN ON...");
        // digitalWrite(alertPin, LOW); // Still turning off pin 4 here if mains is on.
        ESPNowHandler_sendCommand("ON_APPLIANCE");
    } else if (message.equalsIgnoreCase("MAINS OFF")) {
        Serial.println("MQTT: Turning MAINS OFF...");
        digitalWrite(alertPin, LOW); // Still turning off pin 4 here if mains is off.
        ESPNowHandler_sendCommand("OFF_APPLIANCE");

    } else if (message.equalsIgnoreCase("MODE GSM")) {
        ("MQTT: Setting mode to GSM only...");
        setSystemMode(MODE_GSM_ONLY);
        Serial.println("Mode set to GSM only. Rebooting...");
        ESP.restart();
    } else if (message.equalsIgnoreCase("MODE WIFI")) {
        Serial.println("MQTT: Setting mode to WiFi only...");
        setSystemMode(MODE_WIFI_ONLY);
        ESP.restart();
    } else if (message.equalsIgnoreCase("MODE BOTH")) {
        Serial.println("MQTT: Setting mode to Both GSM & WiFi...");
        setSystemMode(MODE_BOTH);
        ESP.restart();
    } else if (message.equalsIgnoreCase("MODE")) {
        Serial.println("MQTT: Telling mode to dashboard...");
        if (currentSystemMode == MODE_GSM_ONLY) setDashboardMode("GSM_ONLY_ACTIVE");
        else if (currentSystemMode == MODE_WIFI_ONLY) setDashboardMode("WIFI_ONLY_ACTIVE");
        else setDashboardMode("BOTH_ACTIVE");
    } else if (message.equalsIgnoreCase("REBOOT")) {
        Serial.println("MQTT: Rebooting...");
        ESP.restart();
    } else {
        Serial.println("MQTT: Unknown command: " + message);
    } */
}


// Function to publish sensor data via MQTT
bool publishSensorData(bool motion, int fire, float temperature, float humidity, float gasLPG, float gasCO, float gasSmoke, float rms_Voltage) {
  if (!client.connected()) {
    Serial.println("MQTT: Not connected, cannot publish sensor data.");
    return false;
  }

  // Use a JSON format for structured data, it's very flexible for dashboards
  // ArduinoJson library is excellent for this if you prefer, but snprintf is fine for simple flat data
  char jsonBuffer[256]; // Adjust size as needed

  // Example JSON structure: {"motion":1,"fire":50,"temp":25.50,"hum":60.20,"lpg":10.00,"co":2.00,"smoke":500.00}
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"motion\":%d,\"fire\":%d,\"temp\":%.2f,\"hum\":%.2f,\"lpg\":%.2f,\"co\":%.2f,\"smoke\":%.2f,\"Vrms\":%.2f}",
           motion ? 1 : 0, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage); // Use temp_val and hum_val

  Serial.print("Publishing to ");
  Serial.print(MQTT_PUBLISH_TOPIC_SENSORS);
  Serial.print(": ");
  Serial.println(jsonBuffer);

  return client.publish(MQTT_PUBLISH_TOPIC_SENSORS, jsonBuffer);
}

bool publishInfo(const char* info) {
  return client.publish(MQTT_PUBLISH_TOPIC_INFO, info);
}

// Function to update the dashboard with current operational mode
void setDashboardMode(String mode) {
  if (client.connected()) {
    client.publish(MQTT_PUBLISH_TOPIC_MODE, mode.c_str());
    Serial.println("Published mode: " + mode + " to dashboard.");
  } else {
    Serial.println("Not connected!");
  }
}