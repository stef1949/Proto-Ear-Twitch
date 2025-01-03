#include <NimBLEDevice.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// UUIDs for custom GATT service and characteristics
#define TEMPERATURE_SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define TEMPERATURE_CHARACTERISTIC_UUID "abcd1234-5678-1234-5678-1234567890ab"
#define CONTROL_CHARACTERISTIC_UUID     "1234abcd-5678-1234-5678-1234567890ab"
#define TEMPERATURE_DESCRIPTOR_UUID     "2901"
#define CONTROL_DESCRIPTOR_UUID         "2901"

bool isConnected = false;
NimBLECharacteristic* pTemperatureCharacteristic;
NimBLECharacteristic* pControlCharacteristic;

// Servo control
Servo servo1;
Servo servo2;
const int servo1Pin = 18;
const int servo2Pin = 19;
const int restingPosition = 135;
const int interval = 10;
const int deadBandWidth = 1; // Dead band width in microseconds

unsigned long previousMillis = 0; // Stores the last time the servos were updated
const unsigned long stepInterval = 50; // Update interval in milliseconds

// Define missing variables
int APin = 25;         // Example GPIO pin
int freq = 50;       // Frequency in Hz
ESP32PWM pwm;          // Create an instance of ESP32PWM

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Mode selection
bool useAccelerometerMode = true; // true for accelerometer mode, false for random mode
bool synchronizeServos = true; // Variable to synchronize servo movements

// Smoothing variables
float smoothedPitch = 0;
float smoothedRoll = 0;
const float alpha = 0.005; // smoothing factor

// Noise filtering variables
float filteredX = 0;
float filteredY = 0;
float filteredZ = 0;
const float noiseAlpha = 0.5; // Filtering factor for noise

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
    accel.setRange(ADXL345_RANGE_2_G); // Set range to ±2g
    accel.setDataRate(ADXL345_DATARATE_100_HZ); // Adjust based on application
    accel.set
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

void smoothMove(Servo& servo, int& currentPos, int targetPos, unsigned long& lastStepTime, int stepDelay) {
    if (millis() - lastStepTime >= stepDelay) {
        lastStepTime = millis();
        
        // Calculate the step size (increase for larger differences)
        int stepSize = max(1, abs(targetPos - currentPos) / 5); // Step size proportional to the difference
        
        if (currentPos < targetPos) {
            currentPos = min(currentPos + stepSize, targetPos);
        } else if (currentPos > targetPos) {
            currentPos = max(currentPos - stepSize, targetPos);
        }

        servo.write(currentPos);
    }
}

void loop() {
    static unsigned long lastUpdate = 0;
    static int lastServo1Position = restingPosition;
    static int lastServo2Position = 180 - restingPosition;
    static unsigned long lastServo1StepTime = 0;
    static unsigned long lastServo2StepTime = 0;
    unsigned long currentMillis = millis(); // Define and update currentMillis
    
    // Check if it's time to update
    if (currentMillis - lastUpdate >= interval) {
        lastUpdate = currentMillis;
        
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
        float pitch = atan2(filteredY, sqrt(pow(filteredX, 2) + pow(filteredZ, 2))) * 180.0 / PI;
        float roll = atan2(-filteredX, sqrt(pow(filteredY, 2) + pow(filteredZ, 2))) * 180.0 / PI;

        // Map angles to servo positions
        int servo1Position = map(constrain(pitch, -180, 180), -180, 180, 0, 180);
        int servo2Position = map(constrain(roll, -180, 180), -180, 180, 0, 180);
        
       // Smoothly move each servo
        smoothMove(servo1, lastServo1Position, servo1Position, lastServo1StepTime, interval);
        smoothMove(servo2, lastServo2Position, servo2Position, lastServo2StepTime, interval);
            
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
}
}