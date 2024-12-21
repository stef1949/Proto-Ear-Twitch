#include <NimBLEDevice.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// UUIDs for custom GATT service and characteristics
#define TEMPERATURE_SERVICE_UUID       "12345678-1234-1234-1234-1234567890ab"
#define TEMPERATURE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-1234567890ab"
#define CONTROL_CHARACTERISTIC_UUID    "1234abcd-5678-1234-5678-1234567890ab"
#define TEMPERATURE_DESCRIPTOR_UUID    "2901"
#define CONTROL_DESCRIPTOR_UUID        "2901"

bool isConnected = false;
NimBLECharacteristic* pTemperatureCharacteristic;
NimBLECharacteristic* pControlCharacteristic;

// Servo control
Servo servo1;
Servo servo2;
const int servo1Pin = 18;
const int servo2Pin = 19;
const int restingPosition = 135;
const int interval = 15;
const int deadBandWidth = 5; // Dead band width in microseconds

// Define missing variables
int APin = 25;         // Example GPIO pin
int freq = 1000;       // Frequency in Hz
ESP32PWM pwm;          // Create an instance of ESP32PWM

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Mode selection
bool useAccelerometerMode = true; // true for accelerometer mode, false for random mode

// Smoothing variables
float smoothedPitch = 0;
float smoothedRoll = 0;
const float alpha = 0.5; // smoothing factor

// Noise filtering variables
float filteredX = 0;
float filteredY = 0;
float filteredZ = 0;
const float noiseAlpha = 0.1; // Filtering factor for noise

/*
 * Description:
 * This code uses an ADXL345 accelerometer and controls two servos using an ESP32. The servos are mapped to the pitch
 * and roll angles calculated from accelerometer readings. The pitch and roll values are smoothed using an exponential
 * smoothing technique to ensure stable servo movements. BLE allows switching between accelerometer-based mode and random mode.
 * Noise filtering is applied to the raw accelerometer data for stability.
 *
 * Circuit:
 * - Connect the ADXL345 to the I2C pins of the ESP32 (SDA, SCL).
 * - Connect servo signal pins to GPIO pins 18 and 19. Power servos with an appropriate external power source.
 * - Ensure common ground between ESP32, servos, and power source.
 */

// Callback class for BLE server
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        isConnected = true;
        Serial.println("Client connected.");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        isConnected = false;
        Serial.println("Client disconnected.");
        pServer->startAdvertising();
    }
};

class ControlCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("Received control command: ");
            Serial.println(value.c_str());

            // Switch modes based on BLE input
            if (value == "accelerometer") {
                useAccelerometerMode = true;
                Serial.println("Switched to accelerometer mode.");
            } else if (value == "random") {
                useAccelerometerMode = false;
                Serial.println("Switched to random mode.");
            }
        }
    }
};

void setup() {
    // Allow allocation of all timers and Initialize servos
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);

    Serial.begin(115200);

    pwm.attachPin(APin, freq, 10); // 1KHz 10 bits

    // Initialize servos
    servo1.attach(servo1Pin, 500, 2400);
    servo2.attach(servo2Pin, 500, 2400);
    servo1.write(restingPosition);
    servo2.write(180 - restingPosition);

    // Initialize ADXL345
    if (!accel.begin()) {
        Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
        while (1);
    }
    Serial.println("ADXL345 initialized.");

    // Initialize BLE
    NimBLEDevice::init("ESP32_GATT_Server");
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pTemperatureService = pServer->createService(TEMPERATURE_SERVICE_UUID);
    pTemperatureCharacteristic = pTemperatureService->createCharacteristic(
                                    TEMPERATURE_CHARACTERISTIC_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pTemperatureCharacteristic->setValue("23.5");

    NimBLEDescriptor* temperatureDescriptor = pTemperatureCharacteristic->createDescriptor(
                                                TEMPERATURE_DESCRIPTOR_UUID,
                                                NIMBLE_PROPERTY::READ);
    temperatureDescriptor->setValue("Temperature Sensor (Celsius)");

    pControlCharacteristic = pTemperatureService->createCharacteristic(
                                CONTROL_CHARACTERISTIC_UUID,
                                NIMBLE_PROPERTY::WRITE);
    pControlCharacteristic->setCallbacks(new ControlCallback());

    NimBLEDescriptor* controlDescriptor = pControlCharacteristic->createDescriptor(
                                            CONTROL_DESCRIPTOR_UUID,
                                            NIMBLE_PROPERTY::READ);
    controlDescriptor->setValue("Control Command");

    pTemperatureService->start();
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(TEMPERATURE_SERVICE_UUID);
    pAdvertising->start();

    Serial.println("GATT server started.");
}

void loop() {
    static int lastServo1Position = restingPosition;
    static int lastServo2Position = 180 - restingPosition;

    if (useAccelerometerMode) {
        sensors_event_t event;
        accel.getEvent(&event);

        // Apply noise filtering to accelerometer readings
        filteredX = noiseAlpha * event.acceleration.x + (1 - noiseAlpha) * filteredX;
        filteredY = noiseAlpha * event.acceleration.y + (1 - noiseAlpha) * filteredY;
        filteredZ = noiseAlpha * event.acceleration.z + (1 - noiseAlpha) * filteredZ;

        // Print filtered accelerometer readings to Serial
        Serial.print("Filtered X: ");
        Serial.print(filteredX);
        Serial.print(" m/s^2, Filtered Y: ");
        Serial.print(filteredY);
        Serial.print(" m/s^2, Filtered Z: ");
        Serial.print(filteredZ);
        Serial.println(" m/s^2");

        // Calculate tilt angles
        float pitch = atan2(event.acceleration.y, sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.z, 2))) * 180.0 / PI;
        float roll = atan2(-event.acceleration.x, event.acceleration.z) * 180.0 / PI;

        // Map angles to servo positions
        int servo1Position = map(constrain(pitch, -90, 90), -90, 90, 0, 180);
        int servo2Position = map(constrain(roll, -90, 90), -90, 90, 0, 180);
        
        // Update servos if change exceeds dead band width
        if (abs(servo1Position - lastServo1Position) > deadBandWidth) {
            servo1.write(servo1Position);
            lastServo1Position = servo1Position;
        }
        if (abs(servo2Position - lastServo2Position) > deadBandWidth) {
            servo2.write(servo2Position);
            lastServo2Position = servo2Position;
        }
    } else {
        // Random mode logic
        int randomPosition1 = random(0, 181);
        int randomPosition2 = random(0, 181);

        servo1.write(randomPosition1);
        servo2.write(randomPosition2);
        Serial.print("Random mode: Servo1: ");
        Serial.print(randomPosition1);
        Serial.print(" Servo2: ");
        Serial.println(randomPosition2);
    }

    // Delay to allow servos to settle
    delay(interval);
}