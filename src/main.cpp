#include <NimBLEDevice.h>
#include <FastLED.h>
#include <ESP32Servo.h>

// UUIDs for custom GATT service and characteristics
#define TEMPERATURE_SERVICE_UUID       "12345678-1234-1234-1234-1234567890ab"
#define TEMPERATURE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-1234567890ab"
#define CONTROL_CHARACTERISTIC_UUID    "1234abcd-5678-1234-5678-1234567890ab"

// Descriptor UUIDs
#define TEMPERATURE_DESCRIPTOR_UUID    "2901" // Characteristic User Description for Temperature
#define CONTROL_DESCRIPTOR_UUID        "2901" // Characteristic User Description for Control

const int RGB_PIN = 48;                // Data pin for the built-in RGB LED (use GPIO 45 if 48 doesn’t work)
#define NUM_LEDS 1                     // Number of RGB LEDs (1 if it’s built-in)
CRGB leds[NUM_LEDS];                   // Array to control RGB LED

bool isConnected = false;
NimBLECharacteristic* pTemperatureCharacteristic;
NimBLECharacteristic* pControlCharacteristic;

// Servo control
Servo servo1;
Servo servo2;
const int servo1Pin = 18;       // The GPIO pin where the servo1 is connected
const int servo2Pin = 21;       // The GPIO pin where the servo2 is connected
const int restingPosition = 135;    // The position (in degrees) where the servo rests between twitches
const int twitchRange = 45;         // The total range of motion for twitches (±22.5 degrees from resting position)
const int interval = 15;            // Main loop interval in milliseconds
const int twitchInterval = 2000;    // Time between twitches in milliseconds
int countTwitch = 0;  // Counter for tracking twitch intervals
bool isTwitching = false;  // Flag to track if the servo is currently in a twitch position

void triggerTwitch();

// Callback class for handling client connections and disconnections
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        isConnected = true;
        Serial.println("Client connected, LED solid blue.");
        leds[0] = CRGB::Blue;          // Set RGB LED to solid blue when connected
        FastLED.show();
    }

    void onDisconnect(NimBLEServer* pServer) override {
        isConnected = false;
        Serial.println("Client disconnected, LED will blink blue.");
        pServer->startAdvertising();   // Restart advertising
    }
};

// Callback for handling client writes to the control characteristic
class ControlCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("Received control command: ");
            Serial.println(value.c_str());
            // Handle different commands here
            if (value == "1") {
                triggerTwitch();
            } else if (value == "0") {
                servo1.write(restingPosition);
                servo2.write(180 - restingPosition);
                isTwitching = false;  // Ensure it's not twitching
            }
        }
    }
};

void setup() {
    Serial.begin(115200);

    // Initialize FastLED for RGB LED
    FastLED.addLeds<NEOPIXEL, RGB_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::Blue;              // Set initial color to blue for advertising
    FastLED.show();

    // Initialize servos
    servo1.attach(servo1Pin, 500, 2400);  // Attach the servo to the specified pin with min/max pulse widths
    servo2.attach(servo2Pin, 500, 2400);  // Attach the servo to the specified pin with min/max pulse widths
    servo1.write(restingPosition);
    servo2.write(180 - restingPosition);

    // Initialize BLE device
    NimBLEDevice::init("ESP32_GATT_LED_Server");

    // Create BLE server
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create Temperature Service
    NimBLEService* pTemperatureService = pServer->createService(TEMPERATURE_SERVICE_UUID);

    // Create Temperature Characteristic with READ and NOTIFY properties
    pTemperatureCharacteristic = pTemperatureService->createCharacteristic(
                                    TEMPERATURE_CHARACTERISTIC_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pTemperatureCharacteristic->setValue("23.5");  // Initial temperature value

    // Add a descriptor to the Temperature Characteristic
    NimBLEDescriptor* temperatureDescriptor = pTemperatureCharacteristic->createDescriptor(
                                                TEMPERATURE_DESCRIPTOR_UUID,
                                                NIMBLE_PROPERTY::READ);
    temperatureDescriptor->setValue("Temperature Sensor (Celsius)");

    // Create Control Characteristic with WRITE property
    pControlCharacteristic = pTemperatureService->createCharacteristic(
                                CONTROL_CHARACTERISTIC_UUID,
                                NIMBLE_PROPERTY::WRITE);
    pControlCharacteristic->setCallbacks(new ControlCallback());

    // Add a descriptor to the Control Characteristic
    NimBLEDescriptor* controlDescriptor = pControlCharacteristic->createDescriptor(
                                            CONTROL_DESCRIPTOR_UUID,
                                            NIMBLE_PROPERTY::READ);
    controlDescriptor->setValue("Control Command");

    // Start the service
    pTemperatureService->start();

    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(TEMPERATURE_SERVICE_UUID);
    pAdvertising->start();

    Serial.println("GATT server with LED indicator and descriptors started. RGB LED will blink blue until a connection is established.");
}

void loop() {
    if (isConnected) {
        // Solid blue LED when connected
        leds[0] = CRGB::Blue;
        FastLED.show();

        // Update the temperature value and notify connected clients periodically
        float temperature = 24.1;  // Example temperature value, replace with actual sensor reading
        pTemperatureCharacteristic->setValue(String(temperature).c_str());  // Update temperature dynamically
        pTemperatureCharacteristic->notify();          // Send notification to clients
        delay(5000);                                   // Update every 5 seconds
    } else {
        // Fade blue LED in and out when waiting for a connection
        for (int brightness = 0; brightness <= 255; brightness += 5) {
            leds[0] = CRGB::Blue;
            leds[0].fadeLightBy(255 - brightness);   // Increase brightness
            FastLED.show();
            delay(10);                               // Adjust delay for speed of fade
        }
        for (int brightness = 255; brightness >= 0; brightness -= 5) {
            leds[0] = CRGB::Blue;
            leds[0].fadeLightBy(255 - brightness);   // Decrease brightness
            FastLED.show();
            delay(10);                               // Adjust delay for speed of fade
        }
    }

    // Increment the counter
    countTwitch++;

    // Check if it's time for a twitch (only twitch if not controlled by BLE)
    if (!isConnected && countTwitch >= (twitchInterval / interval)) {
        countTwitch = 0;  // Reset the counter
        triggerTwitch();  // Trigger twitch as usual
    }

    // Small delay to control the loop interval
    delay(interval);
}
// Function to trigger servo twitch
void triggerTwitch() {
    if (!isTwitching) {
        // Calculate a random position for the twitch
        int randomOffset = random(-twitchRange/2, twitchRange/2 + 1);  // Generate a random offset
        int twitchPosition = restingPosition + randomOffset;  // Calculate the new position

        // Ensure the twitch position is within the valid servo range (0-180 degrees)
        twitchPosition = constrain(twitchPosition, 0, 180);

        // Move the servos to the twitch position
        servo1.write(twitchPosition);
        servo2.write(180 - twitchPosition);
        isTwitching = true;
    } else {
        // Return the servos to the resting position
        servo1.write(restingPosition);
        servo2.write(180 - restingPosition);
        isTwitching = false;
    }
}