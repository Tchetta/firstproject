#include <Arduino.h>
#include <WiFi.h>
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
#include "esp_now/esp_now_handler.h"
#include <ESP_Mail_Client.h>
#include <HTTPClient.h>

/** The smtp host name e.g. smtp.gmail.com for GMail or smtp.office365.com for Outlook or smtp.mail.yahoo.com */
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "beforen21@gmail.com"
#define AUTHOR_PASSWORD "kxatcbpuqopzvjjk"

bool esp_now = false;
/* Recipient's email*/
#define RECIPIENT_EMAIL "tchetzzgreat@gmail.com"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);
bool sendHtmlEmail(const char* recipient, const char* subject, const char* htmlContent);

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
const unsigned long callDuration = 60000; // 60 seconds


// --- Sensor Reading Interval ---
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_READ_INTERVAL = 5000; // Read sensors every 5 seconds
// Global cooldown variables for alerts
unsigned long lastMotionAlert = 0;
const unsigned long motionAlertInterval = 60000; // 60 seconds cooldown
unsigned long lastFireAlert = 0;
const unsigned long fireAlertInterval = 60000; // 60 seconds
unsigned long lastHeatAlert = 0;
const unsigned long heatAlertInterval = 60000; // 60 seconds
unsigned long lastSmokeAlert = 0;
const unsigned long smokeAlertInterval = 60000; // 60 seconds
unsigned long lastRms_VoltageAlert = 0;
const unsigned long rms_VoltageAlertInterval = 60000; // 60 seconds

// --- New: Global variables for Time/Location Check Interval ---
unsigned long lastTimeLocationCheck = 0;
const unsigned long TIME_LOCATION_CHECK_INTERVAL = 30000; // Check every 30 seconds

// --- NEW GLOBAL FOR EMAIL ALERT INTERVAL ---
unsigned long lastEmailAlertTime = 0;
const unsigned long EMAIL_ALERT_INTERVAL = 5 * 60 * 1000; // 5 minutes in milliseconds

// In gsm.cpp, ensure checkForIncomingSMS calls this global handleIncomingCommand
// In mqtt.cpp, ensure handleMqttCommand calls this global handleIncomingCommand

// ================== ESP_NOW =======================
// MAC address of the Sensor device (replace with your sensor's actual MAC)
// This is now defined in main and passed to the ESPNowHandler_init
uint8_t SENSOR_PEER_MAC_ADDRESS[] = {0x44, 0x17, 0x93, 0xf9, 0x71, 0x04};

// Global variable to store the last received sensor voltage
volatile float g_lastReceivedVoltage = 0.0;

/**
 * @brief Callback function to store received sensor data from ESP-NOW into a global variable.
 * This function is assigned to `onSensorDataReceivedCallback` in setup.
 * @param voltage The received voltage value.
 */
void storeReceivedSensorData(float voltage) {
    Serial.print("MAIN: Storing received sensor data: ");
    Serial.println(voltage, 3);
    g_lastReceivedVoltage = voltage; // Store the received voltage globally

    // You can add logic here to flag that new data is available
    // For example: volatile bool newDataReady = true;
    // And then use newDataReady in loop() to trigger further actions.
}

// --- Function to check for actual internet connectivity (for external services like email) ---
bool checkInternetConnection() {
    Serial.print("Checking internet connection (HTTP GET to google.com)...");
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(" WiFi not connected, so no internet.");
        return false;
    }

    HTTPClient http;
    // Use a light-weight, highly available service for connection check
    // "http://connectivitycheck.gstatic.com/generate_204" is often used by Android for this purpose.
    // It returns a 204 No Content, which is perfect for just checking connectivity.
    // Alternatively, "http://www.google.com" is also good, though it returns more data.
    http.begin("http://connectivitycheck.gstatic.com/generate_204"); // HTTP GET Request

    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) { // Check for a positive response code
        Serial.printf(" Internet connected! HTTP Response code: %d\n", httpResponseCode);
        http.end(); // Free resources
        return true;
    } else {
        Serial.printf(" No internet connection. HTTP Error: %s\n", http.errorToString(httpResponseCode).c_str());
        http.end(); // Free resources
        return false;
    }
}


// ------------------- Setup WiFi -------------------
void setup_wifi() {
    WiFi.begin(wifi_ssid, wifi_password);
    Serial.print("Connecting to WiFi");
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) { // Limit connection attempts
        delay(500);
        Serial.print(".");
        retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
    } else {
        Serial.println("\nFailed to connect to WiFi.");
    }
}

// ------------------- Setup and Loop -------------------
void setup()
{
    pinMode(4, OUTPUT);
    Serial.begin(115200);
    Serial.println();
    String thisBoard = ARDUINO_BOARD;
    String output = "Board: ";
    output += thisBoard;
    Serial.println(output);

    WiFi.mode(WIFI_STA);
    setup_wifi();

    Serial.print("WiFi Channel: ");
    Serial.println(WiFi.channel());

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

    Serial.println("Starting esp_now");

    // Initialize ESP-NOW handler with the sensor's MAC address
    if (!ESPNowHandler_init(SENSOR_PEER_MAC_ADDRESS)) {
        Serial.println("Failed to initialize ESP-NOW handler. Halting.");
        esp_now = false;
    } else {
        Serial.print("ESP-NOW successfully initialised on Channel.");
        Serial.println(WiFi.channel());
        esp_now = true;
    }

    // Assign the callback for received ESP-NOW data to our new function
    onSensorDataReceivedCallback = storeReceivedSensorData;
    Serial.println("esp_now started");

    setupPIR(PIR_SENSOR_PIN);
    dhtSetup(dhtPin);
    setupFlame();
    setCurrentPin(CURRENT_PIN);
    initGasSensor();
    pinMode(ALERT_PIN, OUTPUT);

    // If WiFi is part of the current mode
    if (currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) {
        if (WiFi.status() == WL_CONNECTED) { // Only check if WiFi is connected locally
            Serial.println("");
            Serial.println("WiFi Channel: ");
            Serial.println(WiFi.channel());
            Serial.println("WiFi connected!");
            myPrint("WiFi - Successfully connected via wifi!");

            // --- DYNAMIC IP DISCOVERY FOR MQTT BROKER ---
            IPAddress ip = WiFi.localIP();
            // Construct the broker's IP using the ESP32's current subnet and the known last octet (.100)
            IPAddress mqttBrokerIp(ip[0], ip[1], ip[2], 241); // Create an IPAddress object directly
            Serial.print("Derived MQTT Broker IP: ");
            Serial.println(mqttBrokerIp); // Print the IPAddress object
            mqttSetup(mqttBrokerIp, mqtt_port); // Setup MQTT client with dynamic IP

            // =============================== EMAIL =====================================
            // ONLY attempt email setup if there is an internet connection
            if (checkInternetConnection()) {
                Serial.println("Internet available, setting up email client.");
                publishInfo("Internet available, setting up email client."); // Publish internet status
                /* Set the network reconnection option */
                MailClient.networkReconnect(true);
                /** Enable the debug via Serial port
                * 0 for no debugging
                * 1 for basic level debugging
                */
                smtp.debug(1);

                /* Set the callback function to get the sending results */
                smtp.callback(smtpCallback);

                /* Declare the Session_Config for user defined session credentials */
                Session_Config config;

                /* Set the session config */
                config.server.host_name = SMTP_HOST;
                config.server.port = SMTP_PORT;
                config.login.email = AUTHOR_EMAIL;
                config.login.password = AUTHOR_PASSWORD;
                config.login.user_domain = "";

                /* Set the NTP config time */
                config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
                config.time.gmt_offset = 1;
                config.time.day_light_offset = 0;

                /* Connect to the server */
                if (!smtp.connect(&config)){
                    ESP_MAIL_PRINTF("Email connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
                } else {
                    if (smtp.isAuthenticated())
                        Serial.println("\nSuccessfully logged in to email server.");
                    else
                        Serial.println("\nConnected to email server with no Auth.");
                }
            } else {
                Serial.println("No internet connection, skipping Email setup.");
                publishInfo("No internet connection, skipping Email setup."); // Publish internet status
            }
            // ============================== EMAIL setup end

        } else {
            Serial.println("WiFi NOT connected, skipping MQTT and Email setup for WiFi mode.");
            publishInfo("WiFi NOT connected, skipping MQTT and Email setup for WiFi mode."); // Publish WiFi status
        }
        Serial.println("");
        Serial.print("WiFi Channel: ");
        Serial.println(WiFi.channel());
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
    if (currentSystemMode == MODE_GSM_ONLY) sendSMS("GSM_ONLY_ACTIVE");
    else if (currentSystemMode == MODE_WIFI_ONLY) setDashboardMode("WIFI_ONLY_ACTIVE");
    else setDashboardMode("BOTH_ACTIVE");

    digitalWrite(4, LOW);

}

void loop()
{

    // --- Handle MQTT (if WiFi enabled) ---
    // MQTT connection should be independent of internet connection if broker is local
    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
        mqttLoop(); // This calls client.loop() and non-blocking reconnect
    }


    // --- Handle GSM SMS reception ---
    if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        checkForIncomingSMS(); // Check for SMS commands
    }

    // --- Declare sensor variables at the start of loop for broader scope ---
    static float temperature = -1.0F;
    static float humidity = -1.0F;
    static float current = -1.0F;
    static float gasLPG = -1.0F;
    static float gasCO = -1.0F;
    static float gasSmoke = -1.0F;
    static int fire = -1;
    static bool motion = false;
    static float rms_Voltage = -1.0F;


    // --- Sensor Readings and Publishing/Alerting ---
    unsigned long now = millis();
    if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        lastSensorReadTime = now;

        temperature = getTemperatureDEG();
        humidity = getHumidity();
        current = currentValue(CURRENT_PIN);
        gasLPG = readGasLPG();
        gasCO = readGasCO();
        gasSmoke = readGasSmoke();
        fire = analogFlamePercent(FLAME_PIN);
        motion = isMotionDetected(); // This is non-blocking with its own internal timer

        rms_Voltage = g_lastReceivedVoltage; // Get the latest ESP-NOW voltage
        Serial.print("Voltage: ");
        Serial.println(rms_Voltage);

        temperature = isnan(temperature) ? -1.0F : temperature;
        humidity = isnan(humidity) ? -1.0F : humidity;
        current = currentValue(CURRENT_PIN);
       /*  gasLPG = (0 > gasLPG || gasLPG > 100 || isnan(gasLPG)) ? -1.0F : gasLPG;
        gasCO = (0 > gasCO || gasCO > 200 || isnan(gasCO)) ? -1.0F : gasCO;
        gasSmoke = (0 > gasSmoke || gasSmoke > 3700 || isnan(gasSmoke)) ? -1.0F : gasSmoke; */
        fire = fire == 47 ? -1 : fire; // Corrected to use -1 for invalid fire readings
        motion = isMotionDetected(); // This is non-blocking with its own internal timer
        // rms_Voltage = (((g_lastReceivedVoltage <= 20.0 || g_lastReceivedVoltage >= 300.0) && g_lastReceivedVoltage != 1118.08) || isnan(g_lastReceivedVoltage)) ? -1.0F : g_lastReceivedVoltage;
        rms_Voltage = (isnan(g_lastReceivedVoltage)) ? -1.0F : g_lastReceivedVoltage;

        // Publish sensor data via MQTT if WiFi is enabled and connected
        // This remains outside the internet check as MQTT is local
        if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage);
        }

        // Send sensor data via HTTP if WiFi is enabled and connected AND MQTT is not preferred or fails
        // Consider if your HTTP endpoint is local or external. If local, it also doesn't need internet.
        // If external, add an internet check. Assuming it might be for a local dashboard.
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
            digitalWrite(ALERT_PIN, HIGH);
            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🚶 Motion detected in the house!");
                    // myPrint("🚶 Motion detected in the house!"); // This also calls HTTP
                } else {
                    // Fallback if GSM not ready in BOTH mode, or for WIFI_ONLY mode
                    Serial.println("🚶 Motion detected (GSM not ready, or WiFi only).");
                    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                        if (publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage)) { // Re-publish all data with motion
                               
                        }
                        if (checkInternetConnection()) {
                            sendHtmlEmail(RECIPIENT_EMAIL, "Motion Alert!", "<h1>Urgent: Motion Detected!</h1><p>Motion has been detected in your smart home. Please check your dashboard.</p>");
                        }
                    }
                }
            }
            // --- Send Email Alert for Motion if WiFi & Internet are active ---
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                if (publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage)) { // Re-publish all data with motion
                    // Optionally publish a specific alert message
                    // client.publish("smart_home/alerts", "Motion Detected!");
                }
                if (checkInternetConnection()) {
                    sendHtmlEmail(RECIPIENT_EMAIL, "Motion Alert!", "<h1>Urgent: Motion Detected!</h1><p>Motion has been detected in your smart home. Please check your dashboard.</p>");
                }
                lastEmailAlertTime = now; // Reset email cooldown timer
            }
        } else {
            digitalWrite(ALERT_PIN, LOW);
            publishInfo("Alarm turned off!");
            sendSMS("Alarm turned off!");
        }


        // Fire Alert
        if (fire > 10 && now - lastFireAlert > fireAlertInterval) { // Assuming 'fire > 10' means fire detected
            lastFireAlert = now;
            ESPNowHandler_sendCommand("FIRE_SYSTEM_ON");
            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🔥Fire detected!");
                    digitalWrite(ALERT_PIN, HIGH);
                } else {
                    Serial.println("🔥 Fire detected (GSM not ready, or WiFi only).");
                    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                        if (publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage)) { // Re-publish all data with motion
                               
                        }
                        if (checkInternetConnection()) {
                            sendHtmlEmail(RECIPIENT_EMAIL, "Fire Alert!", "<h1>Critical: Fire Detected!</h1><p>Fire has been detected in your smart home. Please check your dashboard.</p>");
                        }
                    }
                    digitalWrite(ALERT_PIN, LOW);
                }
            }
            // --- Send Email Alert for Fire if WiFi & Internet are active ---
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                if (checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                    sendHtmlEmail(RECIPIENT_EMAIL, "Fire Alert!", "<h1>Critical: Fire Detected!</h1><p>A fire has been detected in your smart home. Take immediate action!</p>");
                    lastEmailAlertTime = now; // Reset email cooldown timer
                }
            }
        } else {
            digitalWrite(ALERT_PIN, LOW);
            ESPNowHandler_sendCommand("FIRE_SYSTEM_OFF");
            publishInfo("Alarm turned off!");
        }
        // High Voltage Alert
        if ((rms_Voltage > 230 || rms_Voltage < 100) && now - lastRms_VoltageAlert > rms_VoltageAlertInterval) {
            ESPNowHandler_sendCommand("OFF_APPLIANCE");
            lastRms_VoltageAlert = now;
            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    String voltage_msg = "High or Low Voltage at home - !";
                       voltage_msg += String(rms_Voltage);
                       voltage_msg += " volts";
                       voltage_msg += "\n Device Turned OFF";
                       voltage_msg += "\n Send ON APPLIANCE to turn it ON.";
                    sendSMS(voltage_msg);
                    // myPrint(voltage_msg);
                    digitalWrite(ALERT_PIN, HIGH);
                } else {
                    String email_subject = "Voltage Alert!";
                    String email_content = "<h1>Urgent: High/Low Voltage!</h1><p>Abnormal voltage detected: ";
                    email_content += String(rms_Voltage);
                    email_content += " volts. Appliance might be turned off.</p>";
                    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                        // client.publish("smart_home/alerts", "Fire Detected!");
                        if (checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                            sendHtmlEmail(RECIPIENT_EMAIL, "Voltage Alert!", email_content.c_str());
                            lastEmailAlertTime = now; // Reset email cooldown timer
                        }
                    }
                }
            }
            // --- Send Email Alert for Voltage if WiFi & Internet are active ---
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                String email_subject = "Voltage Alert!";
                String email_content = "<h1>Urgent: High/Low Voltage!</h1><p>Abnormal voltage detected: ";
                email_content += String(rms_Voltage);
                email_content += " volts. Appliance might be turned off.</p>";
                sendHtmlEmail(RECIPIENT_EMAIL, email_subject.c_str(), email_content.c_str());
                lastEmailAlertTime = now; // Reset email cooldown timer
            }
        } else {
            ESPNowHandler_sendCommand("ON_APPLIANCE");
         
            digitalWrite(ALERT_PIN, LOW);
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
                        String email_content = "<h1>Alert: High Temperature!</h1><p>The temperature in your home is ";
                        email_content += String(temperature);
                        email_content += " &deg;C. It's getting hot!</p>";
                        // sendHtmlEmail(RECIPIENT_EMAIL, "High Temperature Alert!", email_content.c_str());
                        if (checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                            sendHtmlEmail(RECIPIENT_EMAIL, "Heat Alert!", email_content.c_str());
                            lastEmailAlertTime = now; // Reset email cooldown timer
                        }
                    }
                    digitalWrite(ALERT_PIN, LOW);
                }
            }
            // --- Send Email Alert for High Temp if WiFi & Internet are active ---
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                String email_content = "<h1>Alert: High Temperature!</h1><p>The temperature in your home is ";
                email_content += String(temperature);
                email_content += " &deg;C. It's getting hot!</p>";
                sendHtmlEmail(RECIPIENT_EMAIL, "High Temperature Alert!", email_content.c_str());
                lastEmailAlertTime = now; // Reset email cooldown timer
            }
        }

        // Smoke Alert
        if ((gasSmoke >= 1000.0 || gasCO >= 150 || gasLPG >= 75) && now - lastSmokeAlert > smokeAlertInterval) { // Assuming 1000.0 is a threshold
            lastSmokeAlert = now;
            digitalWrite(ALERT_PIN, HIGH);
            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🔥Gas or Smoke detected");
                } else {
                    Serial.println("🔥 Gas or Smoke detected (GSM not ready, or WiFi only).");
                    if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                        // client.publish("smart_home/alerts", "Smoke Detected!");
                        if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                            sendHtmlEmail(RECIPIENT_EMAIL, "Smoke Alert!", "<h1>Critical: Abnormal gas or Smoke Detected!</h1><p>Smoke has been detected in your smart home. Investigate immediately!</p>");
                            lastEmailAlertTime = now; // Reset email cooldown timer
                        }
                    }
                }
            }
            // --- Send Email Alert for Smoke if WiFi & Internet are active ---
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
                if (checkInternetConnection() && now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) {
                    sendHtmlEmail(RECIPIENT_EMAIL, "Smoke Alert!", "<h1>Critical: Abnormal gas or Smoke Detected!</h1><p>Smoke has been detected in your smart home. Investigate immediately!</p>");
                    lastEmailAlertTime = now; // Reset email cooldown timer
                }
            }
        }
    } // End of SENSOR_READ_INTERVAL check

    // --- Periodically get GSM Time and Location for testing ---
    // Only attempt if GSM is enabled in the current mode and ready
    if ((currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) && isGSMReady()) {
        if (now - lastTimeLocationCheck >= TIME_LOCATION_CHECK_INTERVAL) {
            lastTimeLocationCheck = now;

            Serial.println("\n--- GSM Time & Location Check ---");

            String networkText = "GSM Network Time: ";
            String networkTime = getNetworkTime();
            networkText += networkTime;
            Serial.println(networkText);

            // String gsmLocationText = "GSM Location: ";
            // String gsmLocation = getGsmLocation();
            // gsmLocationText += gsmLocation;
            // Serial.println(gsmLocationText);
            Serial.println("---------------------------------");
        }
    }

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
    // digitalWrite(4, LOW);
    delay(1);
}

// Ensure `handleIncomingCommand` is defined in `aside.cpp` and declared in `aside.h`
// if `gsm.cpp` and `mqtt.cpp` are calling it.
// The provided `aside.cpp` contains the definition for `handleIncomingCommand`.
/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status){
    /* Print the current status */
    Serial.println(status.info());

    /* Print the sending result */
    if (status.success()){
        // ESP_MAIL_PRINTF used in the examples is for format printing via debug Serial port
        // that works for all supported Arduino platform SDKs e.g. AVR, SAMD, ESP32 and ESP8266.
        // In ESP8266 and ESP32, you can use Serial.printf directly.

        Serial.println("----------------");
        ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
        ESP_MAIL_PRINTF("Message sent failed: %d\n", status.failedCount());
        Serial.println("----------------\n");

        for (size_t i = 0; i < smtp.sendingResult.size(); i++)
        {
            /* Get the result item */
            SMTP_Result result = smtp.sendingResult.getItem(i);

            // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
            // your device time was synched with NTP server.
            // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
            // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)

            ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
            ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
            ESP_MAIL_PRINTF("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
            ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
            ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
        }
        Serial.println("----------------\n");

        // You need to clear sending result as the memory usage will grow up.
        smtp.sendingResult.clear();
    }
}
bool sendHtmlEmail(const char* recipient, const char* subject, const char* htmlContent) {
    // Declare the message class
    SMTP_Message message;

    // Set the message headers
    message.sender.name = F("ESP Board"); // Sender name
    message.sender.email = AUTHOR_EMAIL; // Sender email
    message.subject = subject;           // Email subject from parameter
    message.addRecipient(F("User"), recipient); // Add recipient from parameter

    // Set HTML content
    message.html.content = htmlContent;
    message.html.charSet = "UTF-8"; // Use UTF-8 for broader character support
    message.html.transfer_encoding = Content_Transfer_Encoding::enc_qp; // Quoted-Printable is good for HTML

    Serial.printf("Attempting to send HTML email to %s with subject: %s\n", recipient, subject);

    // Start sending Email and close the session
    // This function itself will check for an active connection implicitly
    // but we can add an explicit check here as well for clarity.
    if (WiFi.status() == WL_CONNECTED && checkInternetConnection()) {
        if (!MailClient.sendMail(&smtp, &message)) {
            ESP_MAIL_PRINTF("Error sending HTML email, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
            return false;
        }
        Serial.println("HTML Email sending initiated successfully.");
        return true;
    } else {
        Serial.println("Cannot send HTML email: No internet connection.");
        return false;
    }
}