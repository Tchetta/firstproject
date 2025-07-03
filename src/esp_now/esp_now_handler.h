// src/esp_now/esp_now_handler.h
#ifndef ESP_NOW_HANDLER_H
#define ESP_NOW_HANDLER_H

#include <Arduino.h>
#include <esp_now.h> // Include esp_now.h here, as this module directly deals with it

// --- Data Structures ---
// Structure for ESP-NOW command (Master to Sensor)
typedef struct {
    char command[32]; // e.g., "GET_VOLTAGE"
} command_message;

// Structure for ESP-NOW data (Sensor to Master)
typedef struct {
    float voltage;
    // Add other sensor data if needed
} sensor_data_message;

// --- External Variables ---
// Declare a global function pointer that main.cpp can assign to.
// This will be called when sensor data is received via ESP-NOW.
extern void (*onSensorDataReceivedCallback)(float voltage);

// --- Function Declarations ---

/**
 * @brief Initializes ESP-NOW, registers callbacks, and adds the peer device.
 * @param peerMacAddr Pointer to the MAC address of the peer device (Sensor).
 * @return True if ESP-NOW initialization and peer addition are successful, false otherwise.
 */
bool ESPNowHandler_init(const uint8_t *peerMacAddr);

/**
 * @brief Sends a command message to the peer device via ESP-NOW.
 * @param commandString The command string (e.g., "GET_VOLTAGE").
 * @return True if the command was sent successfully, false otherwise.
 */
bool ESPNowHandler_sendCommand(const char* commandString);

#endif // ESP_NOW_HANDLER_H