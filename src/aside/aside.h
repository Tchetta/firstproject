#ifndef ASIDE_H
#define ASIDE_H

#include <Preferences.h> // For NVS (Non-Volatile Storage)

void blink_internal_led(int count = 3, int delay = 300);
void setBlinkPin();
void setInternalBlinkPin();
void handleIncomingCommandOLD(const String &sender, const String &message);
void handleIncomingCommand(const String &sender, const String &message);
void myPrint(String text);
void flameSetup(int flamePin);
bool flameValue(int flamePin);
// int setup_wifi(const char* ssid, const char* password);

// --- Global variables for Operational Mode Management ---
enum SystemMode { MODE_GSM_ONLY, MODE_WIFI_ONLY, MODE_BOTH };
extern Preferences preferences; // For NVS
extern SystemMode currentSystemMode; // Default mode
extern const char* MODE_KEY; // NVS key

void setSystemMode(SystemMode newMode);

struct SensorMuteStatus {
    bool motion = false;
    bool fire = false;
    bool gas = false;
    bool voltage = false;
    bool temperature = false;
};

#endif