#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <ESP32Servo.h>

// Pin Definitions
#define MOTOR1_PIN 12    // Front Left
#define MOTOR2_PIN 13    // Front Right
#define MOTOR3_PIN 14    // Rear Right
#define MOTOR4_PIN 15    // Rear Left
#define GPS_TX_PIN 17
#define GPS_RX_PIN 16
#define LORA_CS 5
#define LORA_RST 4
#define LORA_DIO0 2
#define I2C_SDA 21
#define I2C_SCL 22
#define BATTERY_ADC 34
#define STATUS_LED 25
#define BUZZER_PIN 26

// System Configuration
#define LORA_FREQUENCY 433E6
#define LORA_SPREADING_FACTOR 9
#define LORA_BANDWIDTH 125E3
#define LORA_CODING_RATE 5
#define LORA_TX_POWER 20

// Motor Configuration
#define MOTOR_MIN_THROTTLE 1000
#define MOTOR_MAX_THROTTLE 2000
#define MOTOR_ARM_DELAY 2000

// PID Constants
#define PID_ROLL_P 2.0
#define PID_ROLL_I 0.1
#define PID_ROLL_D 0.5
#define PID_PITCH_P 2.0
#define PID_PITCH_I 0.1
#define PID_PITCH_D 0.5
#define PID_YAW_P 3.0
#define PID_YAW_I 0.0
#define PID_YAW_D 0.0

// Global Objects
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Servo motors[4];

// System State
bool armed = false;
bool gpsFix = false;
float batteryVoltage = 0.0;
uint32_t messageId = 0;

// Flight Control Variables
float throttle = 0.0;
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;
float motorOutputs[4] = {0.0, 0.0, 0.0, 0.0};

// GPS Data
double latitude = 0.0;
double longitude = 0.0;
double altitude = 0.0;
double speed = 0.0;

// LoRa Alert Packet Structure
struct AlertPacket {
    uint8_t header;
    uint8_t message_id;
    uint8_t status;
    float latitude;
    float longitude;
    uint64_t timestamp;
    uint8_t confidence;
    int8_t temperature;
    uint16_t checksum;
} __attribute__((packed));

// Telemetry Packet Structure
struct TelemetryPacket {
    uint8_t header;
    uint8_t message_id;
    uint8_t status;
    float latitude;
    float longitude;
    uint16_t altitude;
    uint8_t speed;
    uint8_t heading;
    uint8_t satellites;
    int8_t temperature;
    uint8_t voltage;
    uint8_t current;
    uint16_t checksum;
} __attribute__((packed));

// Function Declarations
void setupPins();
bool initializeGPS();
bool initializeLoRa();
void initializeMotors();
void updateGPS();
void updateBatteryVoltage();
void armMotors();
void disarmMotors();
void updateMotors();
void sendAlert(float lat, float lon, uint8_t confidence, int8_t temperature);
void sendTelemetry();
uint16_t calculateChecksum(const void* data, size_t length);
uint8_t createStatusFlags(uint8_t confidence = 0);
uint64_t getUnixTimestamp();
void checkFireDetection();
void blinkLED(int times, int delayMs);

// PID Controller Class
class PIDController {
private:
    float kp, ki, kd;
    float prevError = 0.0;
    float integral = 0.0;
    float maxOutput;
    unsigned long lastTime = 0;

public:
    PIDController(float p, float i, float d, float maxOut) {
        kp = p;
        ki = i;
        kd = d;
        maxOutput = maxOut;
    }

    float calculate(float error, float dt) {
        if (dt <= 0) return 0.0;

        // Proportional term
        float pTerm = kp * error;

        // Integral term
        integral += error * dt;
        integral = constrain(integral, -maxOutput, maxOutput);
        float iTerm = ki * integral;

        // Derivative term
        float derivative = (error - prevError) / dt;
        float dTerm = kd * derivative;
        prevError = error;

        // Calculate output
        float output = pTerm + iTerm + dTerm;
        return constrain(output, -maxOutput, maxOutput);
    }

    void reset() {
        prevError = 0.0;
        integral = 0.0;
        lastTime = 0;
    }
};

// PID Controllers
PIDController rollPID(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D, 400.0);
PIDController pitchPID(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D, 400.0);
PIDController yawPID(PID_YAW_P, PID_YAW_I, PID_YAW_D, 400.0);

// Main Setup Function
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Flight Controller Starting...");

    // Initialize pins
    setupPins();

    // Initialize components
    if (!initializeGPS()) {
        Serial.println("GPS initialization failed!");
        blinkLED(3, 500);
    }

    if (!initializeLoRa()) {
        Serial.println("LoRa initialization failed!");
        blinkLED(4, 500);
    }

    initializeMotors();

    // Initialize I2C for inter-ESP communication
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    Serial.println("System initialization complete!");
    blinkLED(2, 200);
}

// Main Loop
void loop() {
    static unsigned long lastGPSUpdate = 0;
    static unsigned long lastTelemetry = 0;
    static unsigned long lastFireCheck = 0;
    static unsigned long lastBatteryUpdate = 0;

    unsigned long currentTime = millis();

    // Update GPS (as fast as possible)
    updateGPS();

    // Update battery voltage (every 5 seconds)
    if (currentTime - lastBatteryUpdate > 5000) {
        updateBatteryVoltage();
        lastBatteryUpdate = currentTime;
    }

    // Send telemetry (every 10 seconds)
    if (currentTime - lastTelemetry > 10000) {
        sendTelemetry();
        lastTelemetry = currentTime;
    }

    // Check for fire detection (every 5 seconds)
    if (currentTime - lastFireCheck > 5000) {
        checkFireDetection();
        lastFireCheck = currentTime;
    }

    // Simple flight control (armed and throttle > 0)
    if (armed && throttle > 0.0) {
        // Basic stabilization
        float rollCorrection = rollPID.calculate(0.0 - roll, 0.01);
        float pitchCorrection = pitchPID.calculate(0.0 - pitch, 0.01);
        float yawCorrection = yawPID.calculate(0.0 - yaw, 0.01);

        // Calculate motor outputs (X configuration)
        motorOutputs[0] = throttle - rollCorrection - pitchCorrection + yawCorrection; // Front Left (CW)
        motorOutputs[1] = throttle + rollCorrection - pitchCorrection - yawCorrection; // Front Right (CCW)
        motorOutputs[2] = throttle + rollCorrection + pitchCorrection + yawCorrection; // Rear Right (CW)
        motorOutputs[3] = throttle - rollCorrection + pitchCorrection - yawCorrection; // Rear Left (CCW)

        // Constrain outputs
        for (int i = 0; i < 4; i++) {
            motorOutputs[i] = constrain(motorOutputs[i], 0.0, 1.0);
        }

        updateMotors();
    }

    delay(10); // 100Hz main loop
}

// Pin Setup
void setupPins() {
    // Motor pins
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    pinMode(MOTOR3_PIN, OUTPUT);
    pinMode(MOTOR4_PIN, OUTPUT);

    // LED and buzzer
    pinMode(STATUS_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // Battery monitoring
    pinMode(BATTERY_ADC, INPUT);

    Serial.println("Pins configured");
}

// GPS Initialization
bool initializeGPS() {
    gpsSerial.begin(9600, SERIAL_8N1, GPS_TX_PIN, GPS_RX_PIN);
    if (!gpsSerial) {
        return false;
    }

    Serial.println("GPS initialized");
    return true;
}

// LoRa Initialization
bool initializeLoRa() {
    SPI.begin(18, 19, 23, LORA_CS);  // SCK, MISO, MOSI, CS
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("LoRa begin failed!");
        return false;
    }

    // Configure LoRa parameters
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);

    Serial.println("LoRa initialized");
    return true;
}

// Motor Initialization
void initializeMotors() {
    motors[0].attach(MOTOR1_PIN, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    motors[1].attach(MOTOR2_PIN, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    motors[2].attach(MOTOR3_PIN, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    motors[3].attach(MOTOR4_PIN, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);

    // Set all motors to minimum
    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(MOTOR_MIN_THROTTLE);
    }

    Serial.println("Motors initialized");
}

// GPS Update
void updateGPS() {
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitude = gps.altitude.meters();
        speed = gps.speed.kmph();
        gpsFix = true;

        Serial.printf("GPS: %.6f, %.6f, %.1fm, %.1fkm/h\n",
                     latitude, longitude, altitude, speed);
    }
}

// Battery Voltage Update
void updateBatteryVoltage() {
    int adcValue = analogRead(BATTERY_ADC);
    // Assuming voltage divider: 10k/10k, 3.3V ADC reference
    batteryVoltage = (adcValue / 4095.0) * 3.3 * 2.0;

    Serial.printf("Battery: %.2fV\n", batteryVoltage);

    // Low battery warning
    if (batteryVoltage < 10.5) {
        blinkLED(5, 100);
    }
}

// Motor Control
void armMotors() {
    if (!armed && throttle < 0.1) {
        Serial.println("Arming motors...");

        // Send max throttle signal
        for (int i = 0; i < 4; i++) {
            motors[i].writeMicroseconds(MOTOR_MAX_THROTTLE);
        }

        delay(MOTOR_ARM_DELAY);

        // Send min throttle signal
        for (int i = 0; i < 4; i++) {
            motors[i].writeMicroseconds(MOTOR_MIN_THROTTLE);
        }

        armed = true;
        Serial.println("Motors armed!");
    }
}

void disarmMotors() {
    Serial.println("Disarming motors...");
    armed = false;
    throttle = 0.0;

    for (int i = 0; i < 4; i++) {
        motors[i].writeMicroseconds(MOTOR_MIN_THROTTLE);
        motorOutputs[i] = 0.0;
    }

    Serial.println("Motors disarmed!");
}

void updateMotors() {
    for (int i = 0; i < 4; i++) {
        int pulseWidth = map(motorOutputs[i] * 1000, 0, 1000, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
        motors[i].writeMicroseconds(pulseWidth);
    }
}

// LoRa Communication
void sendAlert(float lat, float lon, uint8_t confidence, int8_t temperature) {
    AlertPacket packet;

    // Header: Version 0, High priority (if confidence > 80), ALERT type
    uint8_t priority = (confidence > 80) ? 1 : 0;
    packet.header = (0 << 7) | (priority << 6) | 0x00; // ALERT type = 0x00

    packet.message_id = messageId++;
    packet.status = createStatusFlags(confidence);
    packet.latitude = lat;
    packet.longitude = lon;
    packet.timestamp = getUnixTimestamp();
    packet.confidence = confidence;
    packet.temperature = temperature;
    packet.checksum = calculateChecksum(&packet, sizeof(packet) - 2);

    // Send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(packet));

    int result = LoRa.endPacket();
    if (result == 1) {
        Serial.println("Alert sent successfully!");
    } else {
        Serial.println("Failed to send alert!");
    }
}

void sendTelemetry() {
    TelemetryPacket packet;

    // Header: Version 0, Normal priority, TELEMETRY type
    packet.header = (0 << 7) | (0 << 6) | 0x01; // TELEMETRY type = 0x01

    packet.message_id = messageId++;
    packet.status = createStatusFlags();
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = (uint16_t)altitude;
    packet.speed = (uint8_t)speed;
    packet.heading = (uint8_t)gps.course.deg();
    packet.satellites = (uint8_t)gps.satellites.value();
    packet.temperature = 25; // Placeholder - add temperature sensor
    packet.voltage = (uint8_t)(batteryVoltage * 10); // Store as voltage * 10
    packet.current = 0; // Placeholder - add current sensor
    packet.checksum = calculateChecksum(&packet, sizeof(packet) - 2);

    // Send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(packet));

    int result = LoRa.endPacket();
    if (result == 1) {
        Serial.println("Telemetry sent successfully!");
    } else {
        Serial.println("Failed to send telemetry!");
    }
}

// Utility Functions
uint16_t calculateChecksum(const void* data, size_t length) {
    uint16_t crc = 0xFFFF;
    const uint8_t* bytes = (const uint8_t*)data;

    for (size_t i = 0; i < length; i++) {
        crc ^= bytes[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
    }

    return crc;
}

uint8_t createStatusFlags(uint8_t confidence) {
    uint8_t flags = 0;

    // Fire detected (if confidence > 0)
    if (confidence > 0) {
        flags |= (1 << 7);
    }

    // Large fire (if confidence > 80)
    if (confidence > 80) {
        flags |= (1 << 6);
    }

    // Critical alert (if confidence > 90)
    if (confidence > 90) {
        flags |= (1 << 5);
    }

    // GPS valid
    if (gpsFix) {
        flags |= (1 << 4);
    }

    // Battery low (if voltage < 10.5V for 3S LiPo)
    if (batteryVoltage < 10.5) {
        flags |= (1 << 3);
    }

    // System OK
    flags |= (1 << 2);

    return flags;
}

uint64_t getUnixTimestamp() {
    // This is a simplified version - you might want to add NTP or GPS time sync
    return millis() / 1000;
}

void checkFireDetection() {
    // Request fire detection data from ESP32-CAM
    Wire.beginTransmission(0x42); // ESP32-CAM I2C address
    Wire.write(0x01); // Request fire detection data
    Wire.endTransmission();

    delay(10); // Wait for processing

    // Read response
    Wire.requestFrom(0x42, 3); // Expect 3 bytes: fire_detected, confidence, temperature

    if (Wire.available() >= 3) {
        uint8_t fire_detected = Wire.read();
        uint8_t confidence = Wire.read();
        int8_t temperature = Wire.read();

        if (fire_detected && confidence > 50) {
            Serial.printf("Fire detected! Confidence: %d%%\n", confidence);
            sendAlert(latitude, longitude, confidence, temperature);
            blinkLED(10, 100); // Rapid blink for fire alert
        }
    }
}

void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(STATUS_LED, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED, LOW);
        delay(delayMs);
    }
}

// Serial Command Interface (for testing and calibration)
void processSerialCommand(String command) {
    command.trim();

    if (command == "ARM") {
        armMotors();
    } else if (command == "DISARM") {
        disarmMotors();
    } else if (command.startsWith("THROTTLE ")) {
        float newThrottle = command.substring(9).toFloat();
        throttle = constrain(newThrottle, 0.0, 1.0);
        Serial.printf("Throttle set to: %.2f\n", throttle);
    } else if (command.startsWith("ROLL ")) {
        roll = command.substring(5).toFloat();
        Serial.printf("Roll set to: %.2f\n", roll);
    } else if (command.startsWith("PITCH ")) {
        pitch = command.substring(6).toFloat();
        Serial.printf("Pitch set to: %.2f\n", pitch);
    } else if (command.startsWith("YAW ")) {
        yaw = command.substring(4).toFloat();
        Serial.printf("Yaw set to: %.2f\n", yaw);
    } else if (command == "STATUS") {
        Serial.printf("Armed: %s, Throttle: %.2f, Battery: %.2fV\n",
                     armed ? "YES" : "NO", throttle, batteryVoltage);
    } else if (command == "GPS") {
        Serial.printf("GPS: %.6f, %.6f, Fix: %s\n",
                     latitude, longitude, gpsFix ? "YES" : "NO");
    } else {
        Serial.println("Unknown command. Available: ARM, DISARM, THROTTLE, STATUS, GPS");
    }
}

// Serial Event Handler
void serialEvent() {
    static String command = "";

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (command.length() > 0) {
                processSerialCommand(command);
                command = "";
            }
        } else {
            command += c;
        }
    }
}

