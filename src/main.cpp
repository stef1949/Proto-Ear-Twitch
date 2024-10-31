#include <NimBLEDevice.h>
#include <FastLED.h>

// UUIDs for custom GATT service and characteristics
#define TEMPERATURE_SERVICE_UUID       "12345678-1234-1234-1234-1234567890ab"
#define TEMPERATURE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-1234567890ab"
#define CONTROL_CHARACTERISTIC_UUID    "1234abcd-5678-1234-5678-1234567890ab"

// Descriptor UUIDs
#define TEMPERATURE_DESCRIPTOR_UUID    "2901" // Characteristic User Description for Temperature
#define CONTROL_DESCRIPTOR_UUID        "2902" // Characteristic User Description for Control

const int RGB_PIN = 48;                // Data pin for the built-in RGB LED (use GPIO 45 if 48 doesn’t work)
#define NUM_LEDS 1                     // Number of RGB LEDs (1 if it’s built-in)
CRGB leds[NUM_LEDS];                   // Array to control RGB LED

bool isConnected = false;
NimBLECharacteristic* pTemperatureCharacteristic;
NimBLECharacteristic* pControlCharacteristic;

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
        }
    }
};

void setup() {
    Serial.begin(115200);

    // Initialize FastLED for RGB LED
    FastLED.addLeds<NEOPIXEL, RGB_PIN>(leds, NUM_LEDS);
    leds[0] = CRGB::Blue;              // Set initial color to blue for advertising
    FastLED.show();

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
}