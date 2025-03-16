#include <Arduino.h>

// Sleep mode variables
// Define both timeout values and select the appropriate one in setup()
const unsigned long SLEEP_TIMEOUT_MS_DEBUG = 15000;    // 15 seconds (15,000 ms)
const unsigned long SLEEP_TIMEOUT_MS_NORMAL = 300000; // 5 minutes (300,000 ms)
unsigned long SLEEP_TIMEOUT_MS; // Will be set in setup() based on debugMode
bool sleepModeActive = false;
unsigned long lastActivityTime = 0;
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;


// Power management scopes
void reduceCPUSpeed() {
    // Set CPU frequency to lowest setting (80MHz vs 240MHz default)
    setCpuFrequencyMhz(80);
    Serial.println("CPU frequency reduced to 80MHz for power saving");
  }
  
  void restoreNormalCPUSpeed() {
    // Set CPU frequency back to default (240MHz)
    setCpuFrequencyMhz(240);
    Serial.println("CPU frequency restored to 240MHz");
  }
