#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <WiFi.h> // Include WiFi.h if not already included here

// Define your MQTT broker details here (replace with your PC's IP)
extern const char* mqtt_server; // Declared extern, defined in .cpp
extern const int mqtt_port;

// Function prototypes
// void mqttSetup(const char* server, int port); // Added parameters for flexibility
void mqttSetup(IPAddress serverIp, int port);
void mqttLoop(); // New function to be called frequently in main loop
bool publishSensorData(bool motion, int fire, float temperature, float humidity, float gasLPG, float gasCO, float gasSmoke);
void setDashboardMode(String mode); // Function to update dashboard mode via MQTT
// You might want to declare the client object extern as well if other files need it,
// but usually it's encapsulated within mqtt.cpp
// extern PubSubClient client; // Uncomment if needed elsewhere

// ============================================================
// --- Forward Declarations ---
void publishSensorData();
void publishModeStatus(const char* mode);
void publishAlertAcknowledgement(const char* alertType, const char* status);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMqtt();

// Flags to indicate if an alert has been acknowledged by the dashboard
extern bool motionAcknowledged;
extern bool fireAcknowledged;
extern bool tempAcknowledged;
extern bool smokeAcknowledged;
extern bool lpgAcknowledged;
extern bool coAcknowledged;

#endif