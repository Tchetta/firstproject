#include <Arduino.h>
#include <WiFi.h>
#include "dht/dht.h"
#include "aside/aside.h"
#include "gsm/gsm.h"
#include "gas/gas.h"
#include "current/current.h"
#include "http/http_app.h"
#include "flame/flame.h"
// #include "pir_motion/pir.h"
#include "mqtt/mqtt.h" // <--- Include the new MQTT header
#include <Preferences.h> // For NVS (Non-Volatile Storage)
#include <nvs_flash.h> // <--- THIS IS THE MISSING INCLUDE!
#include "esp_now/esp_now_handler.h"
#include <ESP_Mail_Client.h>
#include <HTTPClient.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

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
// #define CURRENT_PIN 32
#define FLAME_PIN 35
#define PIR_SENSOR_PIN 4
#define ALERT_PIN 2
#define PIRtimeSeconds 3
// const int led = 21;

// === Sensor Mute and Manual Overrides ===
/* struct SensorMuteStatus {
    bool motion = false;
    bool fire = false;
    bool gas = false;
    bool voltage = false;
    bool temperature = false;
}; */
SensorMuteStatus mutedSensors;

bool motionManualOverride = false;
bool motionManuallyEnabled = false;

bool alarmManualOverride = false;
bool alarmManuallyEnabled = true; // default true


// ================== MOTION ===================
// Timer: Auxiliary variables
unsigned long nowMotion = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;
boolean motion = false;

volatile bool motionInterruptTriggered = false;
// bool motionEventDetected = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  motionInterruptTriggered = true;
  startTimer = true;
  lastTrigger = millis();
  digitalWrite(13, HIGH);
}

/* volatile unsigned long lastInterruptTime = 0;
const unsigned long DEBOUNCE_TIME = 300; // milliseconds

void IRAM_ATTR detectsMovement() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > DEBOUNCE_TIME) {
    motionInterruptTriggered = true;
    
    digitalWrite(13, HIGH);
    lastInterruptTime = currentTime;
  }
} */


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

// --- NEW GLOBAL FOR MANUAL ALARM OVERRIDE ---
bool manualAlarmOverride = false; // Controls alarm state by SMS/MQTT commands

// In gsm.cpp, ensure checkForIncomingSMS calls this global handleIncomingCommand
// In mqtt.cpp, ensure handleMqttCommand calls this global handleIncomingCommand

// ================== ESP_NOW =======================
// MAC address of the Sensor device (replace with your sensor's actual MAC)
// This is now defined in main and passed to the ESPNowHandler_init
uint8_t SENSOR_PEER_MAC_ADDRESS[] = {0x44, 0x17, 0x93, 0xf9, 0x71, 0x04};

// Global variable to store the last received sensor voltage
volatile float g_lastReceivedVoltage = 0.0;

const int IO = 23;
const int SCLK = 14;
const int CE = 25;
ThreeWire myWire(IO, SCLK, CE);
RtcDS1302<ThreeWire> Rtc(myWire);

bool rtcOnTime = false;

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

// ------------------- GSM RTC Time parse -------------------

RtcDateTime parseGSMDateTime(String gsmTimeStr) {
  // Expected format: "YYYY-MM-DD HH:MM:SS"
  int year = gsmTimeStr.substring(0, 4).toInt();
  int month = gsmTimeStr.substring(5, 7).toInt();
  int day = gsmTimeStr.substring(8, 10).toInt();
  int hour = gsmTimeStr.substring(11, 13).toInt();
  int minute = gsmTimeStr.substring(14, 16).toInt();
  int second = gsmTimeStr.substring(17, 19).toInt();

  return RtcDateTime(year, month, day, hour, minute, second);
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt) {
  char datestring[20];
  snprintf_P(datestring, countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(), dt.Day(), dt.Year(),
             dt.Hour(), dt.Minute(), dt.Second());
  Serial.print(datestring);
}

// ------------------- Setup and Loop -------------------
void setup()
{
    pinMode(4, OUTPUT);
    pinMode(13, OUTPUT);
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

    // setupPIR(PIR_SENSOR_PIN);
    dhtSetup(dhtPin);
    setupFlame();
    // setCurrentPin(CURRENT_PIN);
    initGasSensor();
    pinMode(ALERT_PIN, OUTPUT);

    Rtc.Begin();
    if (!Rtc.IsDateTimeValid()) {
        Rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
    }

    if (Rtc.GetIsWriteProtected()) {
        Rtc.SetIsWriteProtected(false);
    }
    if (!Rtc.GetIsRunning()) {
        Rtc.SetIsRunning(true);
    }
    if (Rtc.GetDateTime() < RtcDateTime(__DATE__, __TIME__)) {
        Rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
    }

    RtcDateTime nowSetup = Rtc.GetDateTime(); 

    printDateTime(nowSetup);
    Serial.println();

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
            IPAddress mqttBrokerIp(ip[0], ip[1], ip[2], 240); // Create an IPAddress object directly
            Serial.print("Derived MQTT Broker IP: ");
            Serial.println(mqttBrokerIp); // Print the IPAddress object
            mqttSetup(mqttBrokerIp, mqtt_port); // Setup MQTT client with dynamic IP

            // =============================== EMAIL =====================================
            // ONLY attempt email setup if there is an internet connection
            if (checkInternetConnection()) {
                configTime(3600, 0, "pool.ntp.org", "time.nist.gov");
                struct tm timeinfo;
                if (getLocalTime(&timeinfo)) {
                RtcDateTime ntpTime = RtcDateTime(1900 + timeinfo.tm_year, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                Rtc.SetDateTime(ntpTime);
                rtcOnTime = true;
                }

                Serial.println("Internet available, setting up email client.");
                publishInfo("Internet available, setting up email client."); // Publish internet status
                // * Set the network reconnection option 
                MailClient.networkReconnect(true);
                /** Enable the debug via Serial port
                * / 0 for no debugging
                * 1 for basic level debugging
                */
                smtp.debug(1);

                // Set the callback function to get the sending results 
                smtp.callback(smtpCallback);

                // * Declare the Session_Config for user defined session credentials 
                Session_Config config;

                // * Set the session config 
                config.server.host_name = SMTP_HOST;
                config.server.port = SMTP_PORT;
                config.login.email = AUTHOR_EMAIL;
                config.login.password = AUTHOR_PASSWORD;
                config.login.user_domain = "";

                // Set the NTP config time 
                config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
                config.time.gmt_offset = 1;
                config.time.day_light_offset = 0;

                // * Connect to the server 
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
            if (!rtcOnTime) {
                Serial.println("Setting RTC time to GSM time...");
                String gsmTime = getNetworkTime();
                if (gsmTime != "N/A") {
                    RtcDateTime gsmDateTime = parseGSMDateTime(gsmTime);
                    Rtc.SetDateTime(gsmDateTime);
                    Serial.println("RTC time successfully set to GSM time!");
                } else {

                }
            } else {
                Serial.println("Cannot set RTC time to GSM time!");
            }
        } else {
            Serial.println("GSM NOT connected!");
            myPrint("⚠️ ESP32 NOT connected via gsm!");
        }
    }
    
   if (currentSystemMode == MODE_GSM_ONLY && isGSMReady()) sendSMS("GSM_ONLY_ACTIVE");
   else {
    if (WiFi.status() == WL_CONNECTED) {
        if (currentSystemMode == MODE_WIFI_ONLY) {
            setDashboardMode("WIFI_ONLY_ACTIVE");
            Serial.println("WIFI_ONLY_ACTIVE");
        } else {
            setDashboardMode("BOTH_ACTIVE");
            Serial.println("WIFI_ONLY_ACTIVE");
        }
    }
   }

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

    /* pinMode(PIR_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIR_SENSOR_PIN), detectsMovement, RISING); */

    // After initial setup, inform dashboard about the mode


}

bool isMotionAllowedNow() {
    RtcDateTime nowTime = Rtc.GetDateTime();
    int hour = nowTime.Hour();
    bool blockedByTime = (hour >= 9 && hour < 15);
    // bool blockedByTime = false;
    if (motionManualOverride)
        return motionManuallyEnabled;
    return !blockedByTime;

    // return true;
}

unsigned long lastInternetCheck = 0;
const unsigned long internetCheckInterval = 90000;
bool internetAvailable = false;

void loop()
{
    nowMotion = millis();

    unsigned long nowInternet = millis();
    if (nowInternet - lastInternetCheck > internetCheckInterval) {
        internetAvailable = checkInternetConnection();
        lastInternetCheck = nowInternet;
    }

    // --- Centralized Alarm Management ---
    bool activateAlarm = false; // Flag to determine if the ALERT_PIN should be HIGH
    // --- Declare sensor variables at the start of loop for broader scope ---
    static float temperature = -1.0F;
    static float humidity = -1.0F;
    static float current = -1.0F;
    static float gasLPG = -1.0F;
    static float gasCO = -1.0F;
    static float gasSmoke = -1.0F;
    static int fire = -1;
    // static bool motion = false;
    static float rms_Voltage = -1.0F;

    // --- Handle GSM SMS reception ---
    if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
        checkForIncomingSMS(); // Check for SMS commands
    }

    // ========================= motion
    /* if (motionInterruptTriggered && (motion == false)) {
        Serial.println("🚨 motionInterruptTriggered = true");
        motionInterruptTriggered = false;

        if (!mutedSensors.motion && isMotionAllowedNow()) {
            motion = true;
            lastMotionAlert = millis();

            Serial.println("🚶 Motion Detected!");

            activateAlarm = true;
            // ESPNowHandler_sendCommand("ALARM_ON");

            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🚶 Motion detected in the house!");
                }
            }

            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) &&
                WiFi.status() == WL_CONNECTED &&
                (millis() - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) &&
                internetAvailable) {

                sendHtmlEmail(RECIPIENT_EMAIL, "Motion Alert!",
                            "<h1>Motion Detected</h1><p>Activity detected at home.</p>");
                lastEmailAlertTime = millis();
            }

            // 🔥❌ Remove this from here:
            publishSensorData(true, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage);
            // because those values are stale.
        } else {
            Serial.println("⚠️ Motion detected but ignored (muted or not allowed now).");
        }
    }

    if (startTimer && motion && (nowMotion - lastTrigger > PIRtimeSeconds * 1000)) {
        Serial.println("✅ Motion period ended. Resetting...");
        motion = false;
        startTimer = false;
        motionInterruptTriggered = false;
        digitalWrite(13, LOW);

        if (!manualAlarmOverride) { // only turn off if not manually forced ON
            digitalWrite(ALERT_PIN, LOW);
            ESPNowHandler_sendCommand("ALARM_OFF");
            publishInfo("Motion cleared. Alarm OFF");
        }

        publishSensorData(false, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage);
    } */

    // --- Handle MQTT (if WiFi enabled) ---
    // MQTT connection should be independent of internet connection if broker is local
   if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
        mqttLoop(); // This calls client.loop() and non-blocking reconnect
    }

    // --- Sensor Readings and Publishing/Alerting ---
    unsigned long now = millis();
    if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
        lastSensorReadTime = now;

        temperature = getTemperatureDEG();
        humidity = getHumidity();
        // current = currentValue(CURRENT_PIN);
        gasLPG = readGasLPG();
        gasCO = readGasCO();
        gasSmoke = readGasSmoke();
        fire = analogFlamePercent(FLAME_PIN);

        rms_Voltage = g_lastReceivedVoltage; // Get the latest ESP-NOW voltage
        Serial.print("Voltage: ");
        Serial.println(rms_Voltage);

        temperature = isnan(temperature) ? -1.0F : temperature;
        humidity = isnan(humidity) ? -1.0F : humidity;
        // Removed redundant current = currentValue(CURRENT_PIN);
        gasLPG = (gasLPG < 0 || isnan(gasLPG)) ? -1.0F : gasLPG; // Assuming 0 is minimum valid for LPG
        gasCO = (gasCO < 0 || isnan(gasCO)) ? -1.0F : gasCO;     // Assuming 0 is minimum valid for CO
        gasSmoke = (gasSmoke < 0 || isnan(gasSmoke)) ? -1.0F : gasSmoke; // Assuming 0 is minimum valid for Smoke
        fire = fire == 47 ? -1 : fire; // Corrected to use -1 for invalid fire readings
        // rms_Voltage = (isnan(g_lastReceivedVoltage)) ? -1.0F : g_lastReceivedVoltage;
        // High/Low Voltage Alert
        // --- Voltage Validation and Categorization ---
        if (!isnan(g_lastReceivedVoltage)) {
            rms_Voltage = g_lastReceivedVoltage;

            // Clamp invalid readings
            if (rms_Voltage < 20) {
                rms_Voltage = 0.0;
            } else if (rms_Voltage > 1000 || rms_Voltage == -1.0) {
                rms_Voltage = -2.0; // Custom marker for invalid
            }
        } else {
            rms_Voltage = -1.0F; // Sensor error
        }

        // ========================= motion

        /* if (startTimer && (nowMotion - lastTrigger > PIRtimeSeconds * 1000)) {
            Serial.println("✅ Motion period ended. Resetting...");
            motion = false;
            startTimer = false;
            motionInterruptTriggered = false;
            digitalWrite(13, LOW);

            if (!manualAlarmOverride) { // only turn off if not manually forced ON
                digitalWrite(ALERT_PIN, LOW);
                ESPNowHandler_sendCommand("ALARM_OFF");
                publishInfo("Motion cleared. Alarm OFF");
            }

            publishSensorData(false, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage);
        } */
        
        // Publish sensor data via MQTT if WiFi is enabled and connected
        if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED) {
            publishSensorData(motion, fire, temperature, humidity, gasLPG, gasCO, gasSmoke, rms_Voltage);
        }
        
        // Fire Alert
        if (fire > 10 && (now - lastFireAlert > fireAlertInterval)) { // Assuming 'fire > 10' means fire detected
            lastFireAlert = now;
            activateAlarm = true; // Fire detected, activate alarm
            ESPNowHandler_sendCommand("ALARM_ON");
            ESPNowHandler_sendCommand("FIRE_SYSTEM_ON"); // Turn on fire system regardless of communication mode
            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🔥Fire detected!");
                } else {
                    Serial.println("🔥 Fire detected (GSM not ready, or WiFi only).");
                }
            }
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && (now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL)) {
                if (internetAvailable) {
                    sendHtmlEmail(RECIPIENT_EMAIL, "Fire Alert!", "<h1>Critical: Fire Detected!</h1><p>A fire has been detected in your smart home. Take immediate action!</p>");
                    lastEmailAlertTime = now;
                }
            }
        } else if (fire <= 10) { // If fire is no longer detected, turn off fire system
             ESPNowHandler_sendCommand("FIRE_SYSTEM_OFF");
        }

        // --- Voltage-Based Appliance Control ---
        if ((rms_Voltage == 0.0 || rms_Voltage == -2.0 || rms_Voltage < 100 || rms_Voltage > 230) &&
            rms_Voltage != -1.0F &&
            (now - lastRms_VoltageAlert > rms_VoltageAlertInterval)) {

            lastRms_VoltageAlert = now;
            ESPNowHandler_sendCommand("OFF_APPLIANCE"); // Turn off regardless of type

            // GSM
            if ((currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) && isGSMReady()) {
                String voltage_msg = "";

                if (rms_Voltage == 0.0) {
                    voltage_msg += "⚡ No electricity detected (0V). Appliance turned OFF.";
                } else if (rms_Voltage == -2.0 || rms_Voltage == -1.0) {
                    voltage_msg += "⚠️ Invalid voltage reading (above 1000V). Appliance turned OFF.";
                } else if (rms_Voltage < 100) {
                    voltage_msg += "⚠️ Voltage too low - ";
                    voltage_msg += String(rms_Voltage);
                    voltage_msg += "V. Appliance turned OFF.";
                } else if (rms_Voltage > 230) {
                    voltage_msg += "⚠️ High voltage - ";
                    voltage_msg += String(rms_Voltage);
                    voltage_msg += "V. Appliance turned OFF.";
                }

                sendSMS(voltage_msg);
            }

            // Email (if not recently sent)
            // EMAIL
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) &&
                WiFi.status() == WL_CONNECTED &&
                (now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL) &&
                internetAvailable) {

                String email_msg = "<h1>Voltage Alert!</h1><p>";

                if (rms_Voltage == 0.0) {
                    email_msg += "⚡ No electricity detected (0V). Appliance has been turned OFF.";
                } else if (rms_Voltage == -2.0) {
                    email_msg += "⚠️ Invalid voltage reading (above 1000V). Appliance has been turned OFF.";
                } else if (rms_Voltage < 100) {
                    email_msg += "⚠️ Voltage too low: ";
                    email_msg += String(rms_Voltage);
                    email_msg += "V. Appliance has been turned OFF.";
                } else if (rms_Voltage > 230) {
                    email_msg += "⚠️ High voltage: ";
                    email_msg += String(rms_Voltage);
                    email_msg += "V. Appliance has been turned OFF.";
                }

                email_msg += "</p>";

                sendHtmlEmail(RECIPIENT_EMAIL, "Voltage Alert!", email_msg.c_str());
                lastEmailAlertTime = now;
            }

        } else if (rms_Voltage >= 100 && rms_Voltage <= 230 && rms_Voltage != -1.0F) {
            // Voltage OK, turn ON appliance
            ESPNowHandler_sendCommand("ON_APPLIANCE");
        }

        // High Temperature Alert (45-65 degC)
        if (temperature >= 45.0 && temperature < 65.0 && (now - lastHeatAlert > heatAlertInterval)) {
            lastHeatAlert = now;
            activateAlarm = true; // High temp, activate alarm
            ESPNowHandler_sendCommand("ALARM_ON");

            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🔥Home is too hot");
                } else {
                    Serial.print("🔥 Home is too hot (GSM not ready, or WiFi only): ");
                    Serial.println(temperature);
                }
            }
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && (now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL)) {
                if (internetAvailable) {
                    String email_content = "<h1>Alert: High Temperature!</h1><p>The temperature in your home is ";
                    email_content += String(temperature);
                    email_content += " &deg;C. It's getting hot!</p>";
                    sendHtmlEmail(RECIPIENT_EMAIL, "High Temperature Alert!", email_content.c_str());
                    lastEmailAlertTime = now;
                }
            }
        }

        // Smoke/Gas Alert
        if ((gasSmoke >= 900.0 || gasCO >= 150 || gasLPG >= 55) && (now - lastSmokeAlert > smokeAlertInterval)) {
            lastSmokeAlert = now;
            activateAlarm = true; // Gas/Smoke detected, activate alarm
            ESPNowHandler_sendCommand("ALARM_ON");

            if (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) {
                if (isGSMReady()) {
                    sendSMS("🔥Gas or Smoke detected");
                } else {
                    Serial.println("🔥 Gas or Smoke detected (GSM not ready, or WiFi only).");
                }
            }
            if ((currentSystemMode == MODE_WIFI_ONLY || currentSystemMode == MODE_BOTH) && WiFi.status() == WL_CONNECTED && (now - lastEmailAlertTime >= EMAIL_ALERT_INTERVAL)) {
                if (internetAvailable) {
                    sendHtmlEmail(RECIPIENT_EMAIL, "Smoke Alert!", "<h1>Critical: Abnormal gas or Smoke Detected!</h1><p>Smoke has been detected in your smart home. Investigate immediately!</p>");
                    lastEmailAlertTime = now;
                }
            }
        }

        // --- Apply Alarm State ---
        if (manualAlarmOverride) {
            if (alarmManuallyEnabled) {
                digitalWrite(ALERT_PIN, HIGH);
                ESPNowHandler_sendCommand("ALARM_ON");
                publishInfo("Alarm manually turned ON");
            } else {
                digitalWrite(ALERT_PIN, LOW);
                ESPNowHandler_sendCommand("ALARM_OFF");
                publishInfo("Alarm manually turned OFF");
            }
        } else if (activateAlarm) {
            digitalWrite(ALERT_PIN, HIGH);
            ESPNowHandler_sendCommand("ALARM_ON");
            publishInfo("Alarm activated due to alert!");
        } else {
            digitalWrite(ALERT_PIN, LOW);
            ESPNowHandler_sendCommand("ALARM_OFF");
            publishInfo("Alarm turned off!");
        }


        // --- Critical High Temperature for Calling ---
        // This part is still blocking due to callNumber/handUPCall.
        // In a non-blocking `loop()`, you'd need to manage calling state more carefully.
        // For now, it stays as is, but be aware it will block other activities while a call is active.
        // QUESTION 2: Is this temperature threshold for calling separate from the 'High Temperature Alert' that sends SMS/Email?
        // If temperature >= 65.0 is critical, should it *also* trigger the `activateAlarm` flag? YES.
        if (temperature >= 65.0 && !isCalling && (currentSystemMode == MODE_GSM_ONLY || currentSystemMode == MODE_BOTH) && isGSMReady()) {
            activateAlarm = true; // Turn on alarm for critical temp calling
            ESPNowHandler_sendCommand("ALARM_ON");

            if (callNumber(NUMBER_TO_CALL)) { // This initiates a call, which is blocking until the modem responds
                isCalling = true;
                callStart = now;
                // QUESTION 3: Should the ALERT_PIN be HIGH when a call is initiated due to critical temperature? YES.
                // This is handled by activateAlarm = true;
            }
        }

    } // End of SENSOR_READ_INTERVAL check

    // QUESTION 4: Should the ALERT_PIN be turned OFF immediately after the call ends? No, only when cause of alert goes.
    if (isCalling && now - callStart >= callDuration) {
        handUPCall(); // This also involves AT commands and might block briefly
        isCalling = false;
    }
    
    delay(1); // Keep a small delay to yield to other tasks
    
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
        ESP_MAIL_PRINTF("Message sent failed: %d: %d\n", status.failedCount());
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
    Serial.println("Attempting to send HTML Email");
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
        if (!MailClient.sendMail(&smtp, &message, true)) {
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