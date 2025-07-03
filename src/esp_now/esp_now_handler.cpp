// src/esp_now/esp_now_handler.cpp
#include "esp_now_handler.h"
#include <WiFi.h> // Required for WiFi.mode() which is used with ESP-NOW

// Define the global function pointer declared in the header
void (*onSensorDataReceivedCallback)(float voltage) = nullptr;

// Internal variable for the peer MAC address
static uint8_t s_peerMacAddress[6];

// --- Internal ESP-NOW Callbacks ---

// Callback when data is sent
// This function is automatically called by the ESP-NOW library
// when a packet send operation completes.
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("\r\nESP-NOW: Last Packet Sent to: ");
    Serial.println(macStr);
    Serial.print("ESP-NOW: Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
// This function is automatically called by the ESP-NOW library
// when an ESP-NOW packet is received.
static void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("ESP-NOW: Packet received from: ");
    Serial.println(macStr);
    Serial.print("ESP-NOW: Length: ");
    Serial.println(len);

    if (len == sizeof(sensor_data_message)) {
        sensor_data_message incomingSensorData;
        memcpy(&incomingSensorData, incomingData, sizeof(incomingSensorData));
        Serial.print("ESP-NOW: Received Voltage: ");
        Serial.println(incomingSensorData.voltage, 3); // Print with 3 decimal places

        // Call the external callback function if it's set (to notify main.cpp)
        if (onSensorDataReceivedCallback != nullptr) {
            onSensorDataReceivedCallback(incomingSensorData.voltage);
        }
    } else {
        Serial.println("ESP-NOW: Received data of unexpected size.");
    }
}

// --- Public Functions Implementation ---

bool ESPNowHandler_init(const uint8_t *peerMacAddr) {
    // Store the peer MAC address internally
    memcpy(s_peerMacAddress, peerMacAddr, 6);

    // Set WiFi to Station mode (required for ESP-NOW)
    // WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW: Error initializing ESP-NOW");
        return false;
    }

    // Register send callback
    esp_now_register_send_cb(OnDataSent);

    // Register receive callback
    esp_now_register_recv_cb(OnDataRecv);

    // Add peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize to zeros
    memcpy(peerInfo.peer_addr, s_peerMacAddress, 6);
    peerInfo.channel = 1; // Use current WiFi channel. If no WiFi, it defaults to channel 1.
    peerInfo.encrypt = false; // No encryption for simplicity

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("ESP-NOW: Failed to add peer");
        return false;
    }
    Serial.println("ESP-NOW: Peer added successfully.");
    return true;
}

bool ESPNowHandler_sendCommand(const char* commandString) {
    command_message cmd;
    strncpy(cmd.command, commandString, sizeof(cmd.command) - 1);
    cmd.command[sizeof(cmd.command) - 1] = '\0'; // Ensure null termination

    esp_err_t result = esp_now_send(s_peerMacAddress, (uint8_t *) &cmd, sizeof(cmd));

    if (result == ESP_OK) {
        Serial.print("ESP-NOW: Sent command: ");
        Serial.println(commandString);
        return true;
    } else {
        Serial.print("ESP-NOW: Error sending command: ");
        Serial.println(commandString);
        return false;
    }
}