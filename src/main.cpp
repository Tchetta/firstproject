#include <Arduino.h>
#include "dht/dht.h"
#include "aside/aside.h"
#include "gsm/gsm.h"
#include "gas/gas.h"
#include "current/current.h"
#include "http/http_app.h"
#include "flame/flame.h"
#include "pir_motion/pir.h"
#include "mqtt/mqtt.h" // <--- Include the new MQTT header
#include <Preferences.h> // For NVS (Non-Volatile Storage)
#include <nvs_flash.h> // <--- THIS IS THE MISSING INCLUDE!

#define NUMBER_TO_CALL "653997220"

// digital pins
#define dhtPin 19
#define CURRENT_PIN 32
#define FLAME_PIN 35
#define PIR_SENSOR_PIN 2
#define ALERT_PIN 4

// analog pins
// #define MQ2_PIN 34

// Global WiFi credentials
const char* wifi_ssid = "Redmi 13C"; // Use a distinct name to avoid confusion
const char* wifi_password = "Rodel2.0";

// --- Global variables for Operational Mode Management (DEFINED ONCE) ---
// These are the actual variable definitions, allocating memory
Preferences preferences; // Define the object
// enum SystemMode { MODE_GSM_ONLY, MODE_WIFI_ONLY, MODE_BOTH }; // Enum can be defined in .h
SystemMode currentSystemMode = MODE_BOTH; // Define and initialize
const char* MODE_KEY = "sys_mode"; // Define and initialize

// GSM & Call related
static bool isCalling = false;
static unsigned long callStart = 0;
const unsigned long callDuration = 60000; // 30 seconds

// Global cooldown variables for alerts
unsigned long lastMotionAlert = 0;
const unsigned long motionAlertInterval = 60000; // 60 seconds cooldown
unsigned long lastFireAlert = 0;
const unsigned long fireAlertInterval = 60000; // 60 seconds
unsigned long lastHeatAlert = 0;
const unsigned long heatAlertInterval = 60000; // 60 seconds
unsigned long lastSmokeAlert = 0;
const unsigned long smokeAlertInterval = 60000; // 60 seconds

// --- Sensor Reading Interval ---
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000; // Read sensors every 5 seconds

// In gsm.cpp, ensure checkForIncomingSMS calls this global handleIncomingCommand
// In mqtt.cpp, ensure handleMqttCommand calls this global handleIncomingCommand


void setup()
{
  Serial.begin(115200);
  Serial.println();
  String thisBoard = ARDUINO_BOARD;
  Serial.println("Board: " + thisBoard);

  // --- NVS Initialization (CRITICAL FIX) ---
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("NVS: Partition was corrupted or new version found, erasing...");
    ESP_ERROR_CHECK(nvs_flash_erase()); // Erase if corrupted/new version
    ret = nvs_flash_init(); // Re-initialize after erase
  }
  ESP_ERROR_CHECK(ret); // Check if init was successful

  preferences.begin("smart_home", true); // Open NVS in read-only mode first
  currentSystemMode = (SystemMode)preferences.getUChar(MODE_KEY, MODE_BOTH); // Read saved mode, default to BOTH
  preferences.end(); // Close NVS

  Serial.print("Starting in mode: ");
  if (currentSystemMode == MODE_GSM_ONLY) Serial.println("GSM_ONLY");
  else if (currentSystemMode == MODE_WIFI_ONLY) Serial.println("WIFI_ONLY");
  else Serial.println("BOTH WiFi and GSM");


  setupPIR(PIR_SENSOR_PIN);
  dhtSetup(dhtPin);
  setupFlame();
  setCurrentPin(CURRENT_PIN);
  initGasSensor();
  pinMode(ALERT_PIN, OUTPUT);


  if (currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) {
    if (setup_wifi(wifi_ssid, wifi_password) == 0) { // Check if WiFi connected successfully
      Serial.println("WiFi connected!");
      myPrint("WiFi - Successfully connected via wifi!");
      
      // --- DYNAMIC IP DISCOVERY FOR MQTT BROKER ---
      IPAddress ip = WiFi.localIP();
      // Construct the broker's IP using the ESP32's current subnet and the known last octet (.240)
      IPAddress mqttBrokerIp(ip[0], ip[1], ip[2], 240); // Create an IPAddress object directly
      Serial.print("Derived MQTT Broker IP: ");
      Serial.println(mqttBrokerIp); // Print the IPAddress object
      mqttSetup(mqttBrokerIp, mqtt_port); // Setup MQTT client with dynamic IP
      // setDashboardMode("WIFI_CONNECTED"); // Inform dashboard
    } else {
      // Serial.println("WiFi NOT connected!");
      Serial.println("WiFi - NOT connected!");
      // Fallback or error handling
    }
  }

  if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
    setupGSM(); // This can be blocking
    if (isGSMReady()) {
      Serial.println("GSM connected!");
      myPrint("🌍 ESP32 successfully connected via gsm!");
      sendSMS("ESP32 successfully connected via gsm!");
    } else {
      Serial.println("GSM NOT connected!");
      myPrint("⚠️ ESP32 NOT connected via gsm!");
    }
  }
   // After initial setup, inform dashboard about the mode
  if (currentSystemMode == MODE_GSM_ONLY) setDashboardMode("GSM_ONLY_ACTIVE");
  else if (currentSystemMode == MODE_WIFI_ONLY) setDashboardMode("WIFI_ONLY_ACTIVE");
  else setDashboardMode("BOTH_ACTIVE");
}

void loop()
{
  // --- Handle MQTT (if WiFi enabled) ---
  if (currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) {
    mqttLoop(); // This calls client.loop() and non-blocking reconnect
  }

  // --- Handle GSM SMS reception ---
  if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
    checkForIncomingSMS(); // Check for SMS commands
  }

  // --- Sensor Readings and Publishing/Alerting ---
  unsigned long now = millis();
  if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = now;

    float temperature = getTemperatureDEG();
    float humidity = getHumidity();
    float current = currentValue(CURRENT_PIN);
    float gasLPG = readGasLPG();
    float gasCO = readGasCO();
    float gasSmoke = readGasSmoke();
    int fire = analogFlamePercent(FLAME_PIN);
    bool motion = isMotionDetected(); // This is non-blocking with its own internal timer

    temperature = isnan(temperature) ? -1.0F : temperature;
    humidity = isnan(humidity) ? -1.0F : humidity;
    current = currentValue(CURRENT_PIN);
    gasLPG = (0 > gasLPG || gasLPG > 100 || isnan(gasLPG)) ? -1.0F : gasLPG;
    gasCO = (0 > gasCO || gasCO > 100 || isnan(gasCO)) ? -1.0F : gasCO;
    gasSmoke = (0 > gasSmoke || gasSmoke > 100 || isnan(gasSmoke)) ? -1.0F : gasSmoke;
    fire = fire == 47 ? -1.0F : fire;
    motion = isMotionDetected(); // This is non-blocking with its own internal timer

    // Publish sensor data via MQTT if WiFi is enabled and connected
    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
      publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke);
    }

    // Send sensor data via HTTP if WiFi is enabled and connected AND MQTT is not preferred or fails
    // You can remove this if you exclusively want MQTT for dashboard updates
    // if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
    //   if (!isnan(temperature) && !isnan(humidity)) {
    //     sendSensorReadings(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke);
    //   }
    // }

    // --- Alerting Logic (via GSM or Serial/MQTT if GSM not ready) ---
    // Make sure these calls don't block for too long.
    // sendSMS is blocking, so we need to be careful.
    // If GSM is not ready, consider publishing to MQTT for alerts if WiFi is active.

    // Motion Alert
    if (motion && now - lastMotionAlert > motionAlertInterval) {
      lastMotionAlert = now;
      if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        if (isGSMReady()) {
          sendSMS("🚶 Motion detected in the house!");
          digitalWrite(ALERT_PIN, HIGH);
          myPrint("🚶 Motion detected in the house!"); // This also calls HTTP
        } else {
          // Fallback if GSM not ready in BOTH mode, or for WIFI_ONLY mode
          Serial.println("🚶 Motion detected (GSM not ready, or WiFi only).");
          if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            if (publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke)) { // Re-publish all data with motion
               // Optionally publish a specific alert message
               // client.publish("smart_home/alerts", "Motion Detected!");
            }
          }
          digitalWrite(ALERT_PIN, LOW); // Or HIGH depending on alert logic
        }
    }


    // Fire Alert
    if (fire > 10 && now - lastFireAlert > fireAlertInterval) { // Assuming 'fire > 10' means fire detected
      lastFireAlert = now;
      if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        if (isGSMReady()) {
          sendSMS("🔥Fire detected!");
          myPrint("🔥Fire detected!");
          digitalWrite(ALERT_PIN, HIGH);
        } else {
          Serial.println("🔥 Fire detected (GSM not ready, or WiFi only).");
          if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            // client.publish("smart_home/alerts", "Fire Detected!");
          }
          digitalWrite(ALERT_PIN, LOW);
        }
      }
    }

    // High Temperature Alert (45-65 degC)
    if (temperature >= 45.0 && temperature < 65.0 && now - lastHeatAlert > heatAlertInterval) {
      lastHeatAlert = now;
      if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        if (isGSMReady()) {
          sendSMS("🔥Home is too hot");
          digitalWrite(ALERT_PIN, HIGH);
        } else {
          myPrint("🔥 Home is too hot (GSM not ready, or WiFi only).");
          if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            // client.publish("smart_home/alerts", "High Temperature!");
          }
          digitalWrite(ALERT_PIN, LOW);
        }
      }
    }

    // Smoke Alert
    if (gasSmoke >= 1000.0 && now - lastSmokeAlert > smokeAlertInterval) { // Assuming 1000.0 is a threshold
      lastSmokeAlert = now;
      if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        if (isGSMReady()) {
          sendSMS("🔥Smoke detected");
          digitalWrite(ALERT_PIN, HIGH);
        } else {
          myPrint("🔥 Smoke detected (GSM not ready, or WiFi only).");
          if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            // client.publish("smart_home/alerts", "Smoke Detected!");
          }
          digitalWrite(ALERT_PIN, LOW);
        }
      }
    }
  } // End of SENSOR_READ_INTERVAL check

  // --- Critical High Temperature for Calling ---
  // This part is still blocking due to callNumber/handUPCall.
  // In a non-blocking `loop()`, you'd need to manage calling state more carefully.
  // For now, it stays as is, but be aware it will block other activities while a call is active.
  if (temperature >= 65.0 && !isCalling && (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) && isGSMReady()) {
    if(callNumber(NUMBER_TO_CALL)) { // This initiates a call, which is blocking until the modem responds
      isCalling = true;
      callStart = now;
    }
  }
  if (isCalling && now - callStart >= callDuration) {
    handUPCall(); // This also involves AT commands and might block briefly
    isCalling = false;
  }

  // No delay(500) here to keep loop non-blocking for MQTT/Sensors
  // The delays in some sensor setup functions (like DHT, PIR warm-up, Gas calibration)
  // and GSM setup/SMS sending still exist, but they are generally one-time or infrequent.
  // The GSM waitResponse in checkForIncomingSMS might also be a slight concern.
}

}
// In aside.cpp and gsm.cpp, ensure `handleIncomingCommand` is defined or called correctly.
// For now, I've defined a local one in main.cpp to demonstrate.
// The `mqtt_callback` also needs to call this `handleIncomingCommand`.