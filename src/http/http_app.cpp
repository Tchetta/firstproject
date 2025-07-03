// esp32_http_app.cpp - ESP32 as client, communicates with PC server via HTTP

// #include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "http_app.h"
#include "../aside/aside.h"
#include "../gsm/gsm.h"

#define LED_PIN 4

const char* serverURL = "http://192.168.221.240:8888/esp32tests/api"; // Replace with your PC's local IP

bool ledState = false;
unsigned long lastCheck = 0;
unsigned long lastSendLog = 0;

// void setupESPApp(const char* ssid, const char* password) {
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }

//   pinMode(LED_PIN, OUTPUT);
//   // digitalWrite(LED_PIN, HIGH);

//   Serial.println("");
//   Serial.println("\nConnected to WiFi.");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
//   // // myPrint("");
//   // // myPrint("IP address: " + WiFi.localIP());
//   // // myPrint("\nConnected to WiFi.");
// }

void handleClientESPApp() {
  if (millis() - lastCheck > 3000) {
    lastCheck = millis();
    checkLEDStateFromServer();
  }
}

void checkLEDStateFromServer() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(serverURL) + "/led_state.php");
    int httpCode = http.GET();

    if (httpCode == 200) {
      String payload = http.getString();
      payload.trim();
      if (payload == "on") {
        digitalWrite(LED_PIN, HIGH);
        ledState = true;
      } else {
        digitalWrite(LED_PIN, LOW);
        ledState = false;
      }
    } else {
      if (sendSMS("Can't access the LED state! Verify wifi connection.")) {

      }
    }
    http.end();
  }
}

bool serverAccess() {
  bool access = false;
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(serverURL) + "/check.php");
    int httpCode = http.GET();

    if (httpCode == 200) {
      // myPrint("Connection with server established successfully!");
      access = true;
    } else {
      sendSMS("Could NOT establish a connection with the server!");
      access =  false;
    }
    http.end();
  } else {
    if (sendSMS("Can't use Wifi! Verify wifi connection.")) {
      Serial.println("Can't use Wifi! Verify wifi connection.");
      access = false;
    }
  }
  return access;
}

void setLedState(String state) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(serverURL) + "/set_led.php");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String putData = "state=" + state;
    int httpCode = http.PUT(putData);

    if (httpCode == 200) {
      Serial.println("LED set!");
      // sendSerialLogToServer("LED set!");
    }
    http.end();
  }
}

void sendSensorReadings(bool motion, int fire, float temperature, float humidity, float gasLPG, float gasCO, float gasSmoke) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(serverURL) + "/sensor_data.php");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Format the data as key=value pairs
    String data = "motion=" + String(motion ? "1" : "0");
    data += "&fire=" + String(fire);
    data += "&temperature=" + String(temperature, 2);
    data += "&humidity=" + String(humidity, 2);
    data += "&gasLPG=" + String(gasLPG, 2);
    data += "&gasCO=" + String(gasCO, 2);
    data += "&gasSmoke=" + String(gasSmoke, 2);

    int httpCode = http.POST(data); // You could also use PUT here
    if (httpCode == 200) {
      Serial.println("Sensor data sent!");
      // sendSerialLogToServer("Sensor data sent to server");
    } else {
      Serial.print("Failed to send sensor data. HTTP code: ");
      Serial.println(httpCode);
    }

    http.end();
  } else {
    Serial.println("WiFi not connected. Cannot send sensor data.");
  }
}

void sendSerialLogToServer(String message) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(serverURL) + "/serial_log.php");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String postData = "log=" + message;
    int httpCode = http.POST(postData);
    http.end();
  }
}

