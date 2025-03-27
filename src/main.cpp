#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <fastmath.h>
#include <NimBLEDevice.h>

#include "main.h"

bool isConnected = false;

// int wagMode = 0;
//int currentMode = 0;
// Servo control
Servo servo1;
Servo servo2;
// Published values for SG90 servos; adjust if needed
int minUs = 1000;
int maxUs = 2000;
int servo1Pin = 13;
int servo2Pin = 5;
const int restingPosition = 135;
const int interval = 10;
const int deadBandWidth = 1;           // Dead band width in microseconds
unsigned long previousMillis = 0;      // Stores the last time the servos were updated
const unsigned long stepInterval = 50; // Update interval in milliseconds

// Define missing variables ?????????
int freq = 500; // Frequency in Hz
int posVal = 0;
ESP32PWM pwm; // Create an instance of ESP32PWM

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Mode selection
bool synchronizeServos = true; // Variable to synchronize servo movements

// Smoothing variables
float smoothedPitch = 0;
float smoothedRoll = 0;
const float alpha = 0.4; // smoothing factor. reduce for more smoothing

// Noise filtering variables
float filteredX = 0;
float filteredY = 0;
float filteredZ = 0;
const float noiseAlpha = 0.1; // Filtering factor for noise. reduce for more filtering

uint8_t wagMode = 2;
uint8_t currentMode = 0;

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

// Class to handle characteristic callbacks

// Global or file-scope variables assumed declared elsewhere:
// extern const int restingPosition;
// extern const unsigned long interval;
// extern int wagMode;  // e.g., ACCELEROMETER_MODE, RANDOM_MODE, WAG_MODE
// extern float noiseAlpha, filteredX, filteredY, filteredZ;
// extern Servo servo1, servo2;
// Using the already defined accel object - removing conflicting declaration

// Static state variables used across updates
static int lastServo1Position = restingPosition;
static int lastServo2Position = 180 - restingPosition;
static unsigned long lastServo1StepTime = 0;
static unsigned long lastServo2StepTime = 0;

void smoothMove(Servo &servo, int &currentPos, int targetPos, unsigned long &lastStepTime, int stepDelay)
{
    if (millis() - lastStepTime >= stepDelay)
    {
        lastStepTime = millis();

        // Calculate the step size (increase for larger differences)
        int stepSize = max(1, abs(targetPos - currentPos) / 2); // Step size proportional to the difference

        if (currentPos < targetPos)
        {
            currentPos = min(currentPos + stepSize, targetPos);
        }
        else if (currentPos > targetPos)
        {
            currentPos = max(currentPos - stepSize, targetPos);
        }

        servo.write(currentPos);
    }
}

void updateAccelerometerMode()
{
    static unsigned long lastUpdate = 0;
    static int lastServo1Position = restingPosition;
    static int lastServo2Position = 180 - restingPosition;
    static unsigned long lastServo1StepTime = 0;
    static unsigned long lastServo2StepTime = 0;
    unsigned long currentMillis = millis(); // Define and update currentMillis

    sensors_event_t accelEvent;
    accel.getEvent(&accelEvent);

    // Filter accelerometer readings
    filteredX = noiseAlpha * accelEvent.acceleration.x + (1 - noiseAlpha) * filteredX;
    filteredY = noiseAlpha * accelEvent.acceleration.y + (1 - noiseAlpha) * filteredY;
    filteredZ = noiseAlpha * accelEvent.acceleration.z + (1 - noiseAlpha) * filteredZ;

    Serial.print("Filtered X: ");
    Serial.print(filteredX);
    Serial.print(" m/s^2, Filtered Y: ");
    Serial.print(filteredY);
    Serial.print(" m/s^2, Filtered Z: ");
    Serial.print(filteredZ);
    Serial.println(" m/s^2");

    // Calculate tilt angles (in degrees)
    float pitch = atan(filteredY / sqrt(pow(filteredX, 2) + pow(filteredZ, 2))) * 180.0 / PI;
    float roll = atan(-filteredX / sqrt(pow(filteredY, 2) + pow(filteredZ, 2))) * 180.0 / PI;

    // Map angles to servo positions
    int servo1Position = map(constrain(pitch, -180, 180), -180, 180, 0, 180);
    int servo2Position = map(constrain(roll, -180, 180), -180, 180, 0, 180);

    // Smoothly move each servo
    smoothMove(servo1, lastServo1Position, servo1Position, lastServo1StepTime, interval);
    smoothMove(servo2, lastServo2Position, servo2Position, lastServo2StepTime, interval);
    delay(100);
}

void updateRandomMode()
{
    int randomPosition1 = random(0, 181);
    int randomPosition2 = random(0, 181);

    servo1.write(randomPosition1);
    servo2.write(randomPosition2);

    Serial.print("Random mode: Servo1: ");
    Serial.println(randomPosition1);
    Serial.print(" Servo2: ");
    Serial.println(randomPosition2);
}

void updateWagMode()
{
    static int wagServo1 = restingPosition;
    static bool increasing1 = true;
    const int wagStep = 5;
    const int wagRange = 90;
    int lowerBound1 = restingPosition - wagRange;
    int upperBound1 = restingPosition + wagRange;

    // Update servo1 position within the wag range
    if (increasing1)
    {
        wagServo1 += wagStep;
        if (wagServo1 >= upperBound1)
        {
            increasing1 = false;
        }
    }
    else
    {
        wagServo1 -= wagStep;
        if (wagServo1 <= lowerBound1)
        {
            increasing1 = true;
        }
    }

    servo1.write(wagServo1);
    Serial.print("Wag mode: Servo1: ");
    Serial.println(wagServo1);

    // Update last position for continuity if modes are switched
    lastServo1Position = wagServo1;
}

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        Serial.println("Read request received");
        uint8_t wagValue = static_cast<uint8_t>(wagMode);
        pCharacteristic->setValue(&wagValue, 1);
        Serial.printf("Read request - returned mode: %d\n", wagValue);
    }

    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
    {
        std::string value = pCharacteristic->getValue();

        if (value.size() < 1)
        {
            Serial.println("No data written");
            return;
        }

        // Interpret the first byte as the new mode value.
        uint8_t newMode= value[0];

        if (newMode > 2)
        { // assuming valid range 0-2
            Serial.printf("Invalid mode value: %d\n", newMode);
            return;
        }
        if (newMode != wagMode)
        {
            wagMode = newMode;
            Serial.printf("Mode changed to: %d\n", wagMode);
        }
        //wagMode = static_cast<ModeOption>(newMode);
        //Serial.printf("Mode changed to: %d\n", newMode);
    }

    void onStatus(NimBLECharacteristic *pCharacteristic, int code) override
    {
        Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
    }

    void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue) override
    {
        std::string str = "Client ID: ";
        str += connInfo.getConnHandle();
        str += " Address: ";
        str += connInfo.getAddress().toString();
        if (subValue == 0)
            str += " Unsubscribed to ";
        else if (subValue == 1)
            str += " Subscribed to notifications for ";
        else if (subValue == 2)
            str += " Subscribed to indications for ";
        else if (subValue == 3)
            str += " Subscribed to notifications and indications for ";
        str += std::string(pCharacteristic->getUUID());

        Serial.printf("%s\n", str.c_str());
    }
} chrCallbacks;

class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
        std::string dscVal = pDescriptor->getValue();
        Serial.printf("Descriptor written value: %s\n", dscVal.c_str());
    }

    void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
        Serial.printf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
    }
} dscCallbacks;

void setup()
{
    // Allow allocation of all timers and Initialize servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    // ESP32PWM::allocateTimer(2);
    // ESP32PWM::allocateTimer(3);
    Serial.begin(115200);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.setTimerWidth(deadBandWidth);
    servo2.setTimerWidth(deadBandWidth);
    servo1.attach(servo1Pin, freq, 2500); // 1KHz 2500 bits

    // Initialize servos
    servo1.write(restingPosition);       // Set to resting position
    servo2.write(180 - restingPosition); // Set to opposite of resting position

    // Initialize accelerometer
    if (!accel.begin())
    {
        Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
        while (1)
            ;
    }
    accel.setRange(ADXL345_RANGE_2_G);
    accel.setDataRate(ADXL345_DATARATE_800_HZ);
    Serial.println("ADXL345 sensor initialized");

    // Initialize BLE
    Serial.println("Initializing BLE...");
    NimBLEDevice::init("LumiFur_Controller");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Power level 9 (highest) for best range
    // NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    // Face control characteristic with encryption
    pFaceCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY
        // NIMBLE_PROPERTY::READ_ENC | // only allow reading if paired / encrypted
        // NIMBLE_PROPERTY::WRITE_ENC  // only allow writing if paired / encrypted
    );

    // Set initial Wag mode
uint8_t wagValue = static_cast<uint8_t>(wagMode);
pFaceCharacteristic->setValue(&wagValue, 1);
pFaceCharacteristic->setCallbacks(&chrCallbacks);

    // Temperature characteristic with encryption
    pTemperatureCharacteristic =
        pService->createCharacteristic(
            TEMPERATURE_CHARACTERISTIC_UUID,
            NIMBLE_PROPERTY::READ |
                NIMBLE_PROPERTY::NOTIFY
            // NIMBLE_PROPERTY::READ_ENC
        );

    // Config characteristic with encryption
    pConfigCharacteristic = pService->createCharacteristic(
        CONFIG_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::NOTIFY |
            NIMBLE_PROPERTY::READ_ENC |
            NIMBLE_PROPERTY::WRITE_ENC);

    // Set up descriptors
    NimBLE2904 *pFormat2904 = pFaceCharacteristic->create2904();
    pFormat2904->setFormat(NimBLE2904::FORMAT_UINT8); // Use UINT8 for numeric mode values
    pFormat2904->setUnit(0x2700);                     // Unit-less number
    pFormat2904->setCallbacks(&dscCallbacks);

    // Add user description descriptor
    NimBLEDescriptor *pDesc =
        pFaceCharacteristic->createDescriptor(
            "2901",
            NIMBLE_PROPERTY::READ,
            20);
    pDesc->setValue("Wag Control");

    // nimBLEService* pBaadService = pServer->createService("BAAD");
    pService->start();
    //pServer->getAdvertising()->start();
    /** Create an advertising instance and add the services to the advertised data */
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("LumiFur_Wag Controller");
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->enableScanResponse(true);
    pAdvertising->start();

    Serial.println("BLE setup complete - advertising started");

    Serial.println("GATT server started.");
    
}

void pollWagMode(int mode)
{
    bool modeChanged = false;
    static int wagServo1 = restingPosition;
    static bool increasing1 = true;
    const int wagStep = 20;
    const int wagRange = 90;
    int lowerBound1 = restingPosition - wagRange;
    int upperBound1 = restingPosition + wagRange;

    unsigned long currentMillis = millis();
    static unsigned long lastUpdate = 0;
    if (currentMillis - lastUpdate >= interval)
    {
        lastUpdate = currentMillis;

        switch (mode)
        {
        case 0: // Accelerometer Mode
        {
            // updateAccelerometerMode();
            sensors_event_t event;
            accel.getEvent(&event);

            // Apply noise filtering to accelerometer readings
            filteredX = noiseAlpha * event.acceleration.x + (1 - noiseAlpha) * filteredX; // Apply noise filtering. Reduce noiseAlpha for more filtering
            filteredY = noiseAlpha * event.acceleration.y + (1 - noiseAlpha) * filteredY;
            filteredZ = noiseAlpha * event.acceleration.z + (1 - noiseAlpha) * filteredZ;
            /*
                        // Print filtered accelerometer readings to Serial
                        Serial.print("> X:");
                        Serial.println(filteredX);
                        Serial.print("> Y:");
                        Serial.println(filteredY);
                        Serial.print("> Z:");
                        Serial.println(filteredZ);
                        //Serial.println(" m/s^2");
            */
            // Calculate tilt angles
            float pitch = atan(filteredY / sqrt(pow(filteredX, 2) + pow(filteredZ, 2))) * 180.0 / PI; // Convert to degrees
            float roll = atan(-filteredX / sqrt(pow(filteredY, 2) + pow(filteredZ, 2))) * 180.0 / PI; // Convert to degrees

            // Map angles to servo positions
            int servo1Position = map(constrain(pitch, -180, 180), -180, 180, 0, 180); // Constrain pitch to [-180, 180] and map to [0, 180]
            int servo2Position = map(constrain(roll, -180, 180), -180, 180, 0, 180);  // Constrain roll to [-180, 180] and map to [0, 180]

            // Smoothly move each servo
            smoothMove(servo1, lastServo1Position, servo1Position, lastServo1StepTime, interval);
            smoothMove(servo2, lastServo2Position, servo2Position, lastServo2StepTime, interval);
            break;
        }
        case 1: // Random Mode
            updateRandomMode();
            break;
        case 2: // Wag Mode
            updateWagMode();

            break;
            default:
        Serial.printf("Unknown mode encountered: %d\n", wagMode);
        break;
        }
    }
}

void loop()
{
    bool modeChanged = false;

    static unsigned long lastUpdate = 0;

    unsigned long currentMillis = millis();
    bool isConnected = NimBLEDevice::getServer()->getConnectedCount() > 0;
    /*
        // Always handle BLE and temperature updates first.
        handleBLEConnection();
        if (notifyPending)
        {
            pFaceCharacteristic->notify();
            notifyPending = false;
        }
            */
    // Check if it's time to update
    pollWagMode(wagMode);
}