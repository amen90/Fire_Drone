# 💻 ESP32 Firmware Architecture & Implementation

## Project Structure Overview

```
drone_firmware/
├── flight_controller/          # ESP32-FC firmware
│   ├── src/
│   │   ├── main.cpp
│   │   ├── config.h
│   │   ├── flight_controller/
│   │   │   ├── flight_control.cpp
│   │   │   ├── flight_control.h
│   │   │   ├── motor_control.cpp
│   │   │   ├── motor_control.h
│   │   │   ├── pid_controller.cpp
│   │   │   ├── pid_controller.h
│   │   │   └── sensors/
│   │   │       ├── imu_manager.cpp
│   │   │       ├── imu_manager.h
│   │   │       ├── gps_manager.cpp
│   │   │       └── gps_manager.h
│   │   ├── communication/
│   │   │   ├── lora_manager.cpp
│   │   │   ├── lora_manager.h
│   │   │   ├── inter_esp_comm.cpp
│   │   │   └── inter_esp_comm.h
│   │   ├── system/
│   │   │   ├── power_management.cpp
│   │   │   ├── power_management.h
│   │   │   ├── watchdog.cpp
│   │   │   └── watchdog.h
│   │   └── utils/
│   │       ├── logger.cpp
│   │       ├── logger.h
│   │       ├── math_utils.cpp
│   │       └── math_utils.h
│   ├── lib/
│   │   ├── TinyGPS/
│   │   ├── LoRa/
│   │   ├── PID/
│   │   └── FreeRTOS/
│   ├── test/
│   │   ├── unit_tests/
│   │   └── integration_tests/
│   └── platformio.ini
├── camera_ai/                 # ESP32-CAM firmware
│   ├── src/
│   │   ├── main.cpp
│   │   ├── config.h
│   │   ├── ai/
│   │   │   ├── fire_detection.cpp
│   │   │   ├── fire_detection.h
│   │   │   ├── tensor_flow.cpp
│   │   │   ├── tensor_flow.h
│   │   │   └── model_data.h
│   │   ├── camera/
│   │   │   ├── camera_manager.cpp
│   │   │   ├── camera_manager.h
│   │   │   ├── image_processing.cpp
│   │   │   └── image_processing.h
│   │   ├── communication/
│   │   │   ├── i2c_slave.cpp
│   │   │   └── i2c_slave.h
│   │   └── system/
│   │       ├── memory_manager.cpp
│   │       ├── memory_manager.h
│   │       ├── performance_monitor.cpp
│   │       └── performance_monitor.h
│   ├── models/
│   │   └── fire_detection.tflite
│   ├── lib/
│   │   ├── TensorFlowLite/
│   │   └── ESP32Camera/
│   ├── test/
│   └── platformio.ini
└── shared/                    # Shared libraries and utilities
    ├── protocols/
    │   ├── alert_protocol.h
    │   ├── communication_protocol.h
    │   └── sensor_data.h
    ├── config/
    │   ├── pin_config.h
    │   └── system_config.h
    └── utils/
        ├── crc16.h
        ├── ring_buffer.h
        └── timing.h
```

## ESP32 Flight Controller Firmware

### Main Entry Point

```cpp
// flight_controller/src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreeRTOS.h>
#include "config.h"
#include "flight_control.h"
#include "lora_manager.h"
#include "gps_manager.h"
#include "inter_esp_comm.h"
#include "power_management.h"
#include "logger.h"

// Global objects
FlightControl flightControl;
LoRaManager loraManager;
GPSManager gpsManager;
InterESPComm interESPComm;
PowerManagement powerMgmt;
Logger logger;

// Task handles
TaskHandle_t flightControlTaskHandle;
TaskHandle_t communicationTaskHandle;
TaskHandle_t monitoringTaskHandle;

// Function declarations
void flightControlTask(void *pvParameters);
void communicationTask(void *pvParameters);
void monitoringTask(void *pvParameters);
void setupPins();
void initializePeripherals();

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    logger.init();

    LOG_INFO("ESP32 Flight Controller Starting...");

    // Setup hardware pins
    setupPins();

    // Initialize peripherals
    initializePeripherals();

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        flightControlTask,     // Task function
        "FlightControl",       // Task name
        4096,                  // Stack size
        NULL,                  // Parameters
        3,                     // Priority (high)
        &flightControlTaskHandle,
        1                      // Core 1
    );

    xTaskCreatePinnedToCore(
        communicationTask,
        "Communication",
        3072,
        NULL,
        2,
        &communicationTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        monitoringTask,
        "Monitoring",
        2048,
        NULL,
        1,
        &monitoringTaskHandle,
        0
    );

    LOG_INFO("All tasks created successfully");
}

void loop() {
    // Main loop can be used for low-priority tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void setupPins() {
    // Motor PWM pins
    pinMode(MOTOR1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN, OUTPUT);
    pinMode(MOTOR3_PIN, OUTPUT);
    pinMode(MOTOR4_PIN, OUTPUT);

    // LED and buzzer
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Battery monitoring
    pinMode(BATTERY_ADC_PIN, INPUT);

    LOG_INFO("Pins configured");
}

void initializePeripherals() {
    // Initialize I2C for inter-ESP communication
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // Initialize SPI for LoRa
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    // Initialize components
    if (!flightControl.init()) {
        LOG_ERROR("Flight control initialization failed");
        while(1);
    }

    if (!loraManager.init()) {
        LOG_ERROR("LoRa initialization failed");
        while(1);
    }

    if (!gpsManager.init()) {
        LOG_ERROR("GPS initialization failed");
        while(1);
    }

    if (!interESPComm.init()) {
        LOG_ERROR("Inter-ESP communication initialization failed");
        while(1);
    }

    LOG_INFO("All peripherals initialized");
}
```

### Flight Control Task

```cpp
// flight_controller/src/flight_control.cpp
#include "flight_control.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "imu_manager.h"
#include "gps_manager.h"
#include "logger.h"

FlightControl::FlightControl() :
    armed(false),
    flightMode(MANUAL),
    throttle(0.0f),
    roll(0.0f),
    pitch(0.0f),
    yaw(0.0f)
{
    // Initialize PID controllers
    rollPID = new PIDController(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D, PID_ROLL_MAX);
    pitchPID = new PIDController(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D, PID_PITCH_MAX);
    yawPID = new PIDController(PID_YAW_P, PID_YAW_I, PID_YAW_D, PID_YAW_MAX);
    altitudePID = new PIDController(PID_ALT_P, PID_ALT_I, PID_ALT_D, PID_ALT_MAX);
}

bool FlightControl::init() {
    LOG_INFO("Initializing flight control system");

    // Initialize motor control
    if (!motorControl.init()) {
        LOG_ERROR("Motor control initialization failed");
        return false;
    }

    // Initialize IMU
    if (!imuManager.init()) {
        LOG_ERROR("IMU initialization failed");
        return false;
    }

    // Calibrate sensors
    if (!calibrateSensors()) {
        LOG_ERROR("Sensor calibration failed");
        return false;
    }

    // Set initial motor speeds to minimum
    motorControl.setAllMotors(MOTOR_MIN_THROTTLE);

    LOG_INFO("Flight control system initialized");
    return true;
}

void FlightControl::update(float dt) {
    // Read sensor data
    IMUData imuData = imuManager.getData();
    GPSData gpsData = gpsManager.getData();

    // Update PID controllers based on flight mode
    switch (flightMode) {
        case MANUAL:
            updateManualMode(imuData, dt);
            break;
        case STABILIZE:
            updateStabilizeMode(imuData, dt);
            break;
        case ALTITUDE_HOLD:
            updateAltitudeHoldMode(imuData, gpsData, dt);
            break;
        case POSITION_HOLD:
            updatePositionHoldMode(imuData, gpsData, dt);
            break;
        case AUTO:
            updateAutoMode(imuData, gpsData, dt);
            break;
    }

    // Calculate motor outputs
    calculateMotorOutputs();

    // Update motors
    motorControl.updateMotors(motorOutputs);
}

void FlightControl::updateStabilizeMode(const IMUData& imuData, float dt) {
    // Level flight stabilization
    float rollCorrection = rollPID->calculate(0.0f - imuData.roll, dt);
    float pitchCorrection = pitchPID->calculate(0.0f - imuData.pitch, dt);
    float yawCorrection = yawPID->calculate(yawTarget - imuData.yaw, dt);

    // Apply corrections to motor outputs
    motorOutputs[0] = throttle - rollCorrection - pitchCorrection + yawCorrection; // Front Left
    motorOutputs[1] = throttle + rollCorrection - pitchCorrection - yawCorrection; // Front Right
    motorOutputs[2] = throttle + rollCorrection + pitchCorrection + yawCorrection; // Rear Right
    motorOutputs[3] = throttle - rollCorrection + pitchCorrection - yawCorrection; // Rear Left
}

void FlightControl::calculateMotorOutputs() {
    // Constrain motor outputs to valid range
    for (int i = 0; i < 4; i++) {
        motorOutputs[i] = constrain(motorOutputs[i], MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    }

    // Apply motor mixing for quadcopter X configuration
    float frontLeft = throttle - roll - pitch + yaw;
    float frontRight = throttle + roll - pitch - yaw;
    float rearRight = throttle + roll + pitch + yaw;
    float rearLeft = throttle - roll + pitch - yaw;

    motorOutputs[0] = frontLeft;
    motorOutputs[1] = frontRight;
    motorOutputs[2] = rearRight;
    motorOutputs[3] = rearLeft;
}

void FlightControl::arm() {
    if (!armed && throttle < 0.1f) {
        LOG_INFO("Arming motors");
        armed = true;
        motorControl.armMotors();
    }
}

void FlightControl::disarm() {
    LOG_INFO("Disarming motors");
    armed = false;
    throttle = 0.0f;
    motorControl.disarmMotors();
}

bool FlightControl::calibrateSensors() {
    LOG_INFO("Calibrating sensors");

    // IMU calibration
    LOG_INFO("Calibrating IMU...");
    imuManager.calibrate();

    // ESC calibration
    LOG_INFO("Calibrating ESCs...");
    motorControl.calibrateESCs();

    LOG_INFO("Sensor calibration complete");
    return true;
}
```

### Motor Control Implementation

```cpp
// flight_controller/src/motor_control.cpp
#include "motor_control.h"
#include <ESP32Servo.h>
#include "logger.h"

MotorControl::MotorControl() {
    // Initialize servo objects for ESCs
    motors[0] = new Servo();
    motors[1] = new Servo();
    motors[2] = new Servo();
    motors[3] = new Servo();
}

bool MotorControl::init() {
    LOG_INFO("Initializing motor control");

    // Attach motors to PWM pins
    motors[0]->attach(MOTOR1_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
    motors[1]->attach(MOTOR2_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
    motors[2]->attach(MOTOR3_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
    motors[3]->attach(MOTOR4_PIN, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);

    // Set all motors to minimum throttle
    setAllMotors(MOTOR_MIN_THROTTLE);

    LOG_INFO("Motor control initialized");
    return true;
}

void MotorControl::setMotor(int motorIndex, float throttle) {
    if (motorIndex < 0 || motorIndex >= 4) {
        LOG_ERROR("Invalid motor index");
        return;
    }

    // Convert throttle (0.0-1.0) to PWM pulse width
    int pulseWidth = map(throttle * 1000, 0, 1000, MOTOR_MIN_PULSE, MOTOR_MAX_PULSE);
    motors[motorIndex]->writeMicroseconds(pulseWidth);
}

void MotorControl::setAllMotors(float throttle) {
    for (int i = 0; i < 4; i++) {
        setMotor(i, throttle);
    }
}

void MotorControl::updateMotors(const float motorOutputs[4]) {
    for (int i = 0; i < 4; i++) {
        setMotor(i, motorOutputs[i]);
    }
}

void MotorControl::armMotors() {
    LOG_INFO("Arming motors");

    // Send maximum throttle signal
    setAllMotors(MOTOR_MAX_THROTTLE);
    delay(2000);

    // Send minimum throttle signal
    setAllMotors(MOTOR_MIN_THROTTLE);
    delay(1000);

    LOG_INFO("Motors armed");
}

void MotorControl::disarmMotors() {
    LOG_INFO("Disarming motors");
    setAllMotors(MOTOR_MIN_THROTTLE);
}

void MotorControl::calibrateESCs() {
    LOG_INFO("Starting ESC calibration");

    // Send maximum throttle
    setAllMotors(MOTOR_MAX_THROTTLE);
    delay(2000);

    // Send minimum throttle
    setAllMotors(MOTOR_MIN_THROTTLE);
    delay(4000);

    LOG_INFO("ESC calibration complete");
}
```

### LoRa Communication Manager

```cpp
// flight_controller/src/lora_manager.cpp
#include "lora_manager.h"
#include <LoRa.h>
#include "alert_protocol.h"
#include "logger.h"

LoRaManager::LoRaManager() :
    initialized(false),
    lastPacketTime(0),
    packetCounter(0)
{
}

bool LoRaManager::init() {
    LOG_INFO("Initializing LoRa communication");

    // Initialize LoRa module
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    LoRa.setSPIFrequency(1000000);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        LOG_ERROR("LoRa initialization failed");
        return false;
    }

    // Configure LoRa parameters
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);

    initialized = true;
    LOG_INFO("LoRa initialized successfully");
    return true;
}

bool LoRaManager::sendAlert(float latitude, float longitude, uint8_t confidence) {
    if (!initialized) {
        LOG_ERROR("LoRa not initialized");
        return false;
    }

    // Create alert packet
    AlertPacket packet;
    packet.header = ALERT_HEADER;
    packet.message_id = packetCounter++;
    packet.status = ALERT_STATUS_FIRE_DETECTED;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.timestamp = getUnixTimestamp();
    packet.confidence = confidence;
    packet.checksum = calculateChecksum(&packet, sizeof(packet) - 2);

    // Send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(packet));
    int result = LoRa.endPacket();

    if (result == 1) {
        LOG_INFO("Alert packet sent successfully");
        lastPacketTime = millis();
        return true;
    } else {
        LOG_ERROR("Failed to send alert packet");
        return false;
    }
}

bool LoRaManager::sendTelemetry(const TelemetryData& telemetry) {
    if (!initialized) {
        return false;
    }

    // Create telemetry packet
    TelemetryPacket packet;
    packet.header = TELEMETRY_HEADER;
    packet.message_id = packetCounter++;
    packet.battery_voltage = telemetry.battery_voltage;
    packet.altitude = telemetry.altitude;
    packet.speed = telemetry.speed;
    packet.satellites = telemetry.satellites;
    packet.status = telemetry.status;
    packet.checksum = calculateChecksum(&packet, sizeof(packet) - 2);

    // Send packet
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&packet, sizeof(packet));
    int result = LoRa.endPacket();

    if (result == 1) {
        LOG_INFO("Telemetry packet sent");
        return true;
    } else {
        LOG_ERROR("Failed to send telemetry packet");
        return false;
    }
}

uint16_t LoRaManager::calculateChecksum(const void* data, size_t length) {
    uint16_t checksum = 0;
    const uint8_t* bytes = (const uint8_t*)data;

    for (size_t i = 0; i < length; i++) {
        checksum = (checksum << 8) ^ crc16_table[(checksum >> 8) ^ bytes[i]];
    }

    return checksum;
}

uint32_t LoRaManager::getUnixTimestamp() {
    // This would typically get time from GPS or RTC
    // For now, return milliseconds since boot
    return millis() / 1000;
}
```

### GPS Manager

```cpp
// flight_controller/src/gps_manager.cpp
#include "gps_manager.h"
#include <TinyGPS++.h>
#include "logger.h"

GPSManager::GPSManager() :
    initialized(false),
    gps(new TinyGPSPlus()),
    lastUpdateTime(0)
{
}

bool GPSManager::init() {
    LOG_INFO("Initializing GPS");

    // Initialize serial port for GPS
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (!SerialGPS) {
        LOG_ERROR("GPS serial initialization failed");
        return false;
    }

    initialized = true;
    LOG_INFO("GPS initialized");
    return true;
}

void GPSManager::update() {
    if (!initialized) return;

    // Read data from GPS
    while (SerialGPS.available() > 0) {
        char c = SerialGPS.read();
        gps->encode(c);
    }

    // Update GPS data if new data is available
    if (gps->location.isUpdated()) {
        currentData.latitude = gps->location.lat();
        currentData.longitude = gps->location.lng();
        currentData.altitude = gps->altitude.meters();
        currentData.speed = gps->speed.kmph();
        currentData.course = gps->course.deg();
        currentData.satellites = gps->satellites.value();
        currentData.hdop = gps->hdop.hdop();
        currentData.lastUpdate = millis();

        LOG_DEBUG("GPS updated: %.6f, %.6f, %.1f",
                 currentData.latitude, currentData.longitude, currentData.altitude);
    }

    lastUpdateTime = millis();
}

GPSData GPSManager::getData() {
    return currentData;
}

bool GPSManager::hasFix() {
    return gps->location.isValid() && gps->satellites.value() >= GPS_MIN_SATELLITES;
}

float GPSManager::getDistanceTo(float targetLat, float targetLon) {
    if (!hasFix()) return 0.0f;

    return gps->distanceBetween(
        currentData.latitude, currentData.longitude,
        targetLat, targetLon
    );
}

float GPSManager::getCourseTo(float targetLat, float targetLon) {
    if (!hasFix()) return 0.0f;

    return gps->courseTo(
        currentData.latitude, currentData.longitude,
        targetLat, targetLon
    );
}
```

### Inter-ESP32 Communication

```cpp
// flight_controller/src/inter_esp_comm.cpp
#include "inter_esp_comm.h"
#include "logger.h"

InterESPComm::InterESPComm() :
    initialized(false)
{
}

bool InterESPComm::init() {
    LOG_INFO("Initializing inter-ESP communication");

    // I2C is already initialized in main.cpp
    initialized = true;

    LOG_INFO("Inter-ESP communication initialized");
    return true;
}

bool InterESPComm::sendCommand(uint8_t command, const uint8_t* data, size_t length) {
    if (!initialized) return false;

    Wire.beginTransmission(ESP32_CAM_I2C_ADDRESS);
    Wire.write(command);

    if (data && length > 0) {
        Wire.write(data, length);
    }

    uint8_t result = Wire.endTransmission();

    if (result == 0) {
        LOG_DEBUG("Command sent successfully");
        return true;
    } else {
        LOG_ERROR("Failed to send command, error: %d", result);
        return false;
    }
}

bool InterESPComm::requestData(uint8_t requestType, uint8_t* buffer, size_t length) {
    if (!initialized) return false;

    Wire.beginTransmission(ESP32_CAM_I2C_ADDRESS);
    Wire.write(CMD_REQUEST_DATA);
    Wire.write(requestType);
    Wire.endTransmission();

    delay(10); // Wait for processing

    size_t bytesRead = Wire.requestFrom(ESP32_CAM_I2C_ADDRESS, length);
    if (bytesRead == length) {
        for (size_t i = 0; i < length; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    } else {
        LOG_ERROR("Failed to read data, expected %d bytes, got %d", length, bytesRead);
        return false;
    }
}

bool InterESPComm::checkFireDetection() {
    uint8_t result[2]; // [fire_detected, confidence]

    if (requestData(REQUEST_FIRE_STATUS, result, 2)) {
        bool fireDetected = result[0] > 0;
        uint8_t confidence = result[1];

        if (fireDetected) {
            LOG_INFO("Fire detected with confidence: %d%%", confidence);
        }

        return fireDetected;
    }

    return false;
}

FireDetectionData InterESPComm::getFireDetectionData() {
    FireDetectionData data;

    if (requestData(REQUEST_FIRE_DATA, (uint8_t*)&data, sizeof(data))) {
        return data;
    }

    // Return default values on error
    data.fire_detected = false;
    data.confidence = 0;
    data.latitude = 0.0f;
    data.longitude = 0.0f;

    return data;
}
```

## ESP32-CAM AI Firmware

### Main Entry Point

```cpp
// camera_ai/src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "esp_camera.h"
#include "config.h"
#include "fire_detection.h"
#include "i2c_slave.h"
#include "camera_manager.h"
#include "logger.h"

// Global objects
FireDetection fireDetection;
I2CSlave i2cSlave;
CameraManager cameraManager;
Logger logger;

// Task handles
TaskHandle_t aiTaskHandle;
TaskHandle_t cameraTaskHandle;
TaskHandle_t communicationTaskHandle;

// Function declarations
void aiProcessingTask(void *pvParameters);
void cameraCaptureTask(void *pvParameters);
void communicationTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    logger.init();

    LOG_INFO("ESP32-CAM AI System Starting...");

    // Initialize camera
    if (!cameraManager.init()) {
        LOG_ERROR("Camera initialization failed");
        while(1);
    }

    // Initialize AI model
    if (!fireDetection.init()) {
        LOG_ERROR("AI model initialization failed");
        while(1);
    }

    // Initialize I2C slave
    if (!i2cSlave.init(ESP32_CAM_I2C_ADDRESS)) {
        LOG_ERROR("I2C slave initialization failed");
        while(1);
    }

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        aiProcessingTask,
        "AI Processing",
        8192,
        NULL,
        3,
        &aiTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        cameraCaptureTask,
        "Camera Capture",
        4096,
        NULL,
        2,
        &cameraTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
        communicationTask,
        "Communication",
        3072,
        NULL,
        1,
        &communicationTaskHandle,
        1
    );

    LOG_INFO("ESP32-CAM tasks created");
}

void loop() {
    // Main loop for status monitoring
    static unsigned long lastStatusTime = 0;

    if (millis() - lastStatusTime > 5000) {
        LOG_INFO("System status - Free heap: %d bytes", ESP.getFreeHeap());
        lastStatusTime = millis();
    }

    delay(1000);
}
```

### Fire Detection Implementation

```cpp
// camera_ai/src/fire_detection.cpp
#include "fire_detection.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model_data.h"
#include "logger.h"

FireDetection::FireDetection() :
    initialized(false),
    model(nullptr),
    interpreter(nullptr),
    input(nullptr),
    output(nullptr)
{
}

bool FireDetection::init() {
    LOG_INFO("Initializing TensorFlow Lite model");

    // Initialize error reporter
    static tflite::MicroErrorReporter micro_error_reporter;
    error_reporter = &micro_error_reporter;

    // Load model
    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        LOG_ERROR("Model version mismatch");
        return false;
    }

    // Initialize resolver
    static tflite::AllOpsResolver resolver;

    // Create interpreter
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter = &static_interpreter;

    // Allocate tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        LOG_ERROR("Tensor allocation failed");
        return false;
    }

    // Get input/output tensors
    input = interpreter->input(0);
    output = interpreter->output(0);

    LOG_INFO("TensorFlow Lite model initialized");
    LOG_INFO("Input shape: %d x %d x %d", input->dims->data[1], input->dims->data[2], input->dims->data[3]);
    LOG_INFO("Output shape: %d", output->dims->data[1]);

    initialized = true;
    return true;
}

FireDetectionResult FireDetection::detectFire(camera_fb_t* frame) {
    FireDetectionResult result;
    result.fire_detected = false;
    result.confidence = 0;

    if (!initialized || !frame) {
        return result;
    }

    // Preprocess image
    if (!preprocessImage(frame)) {
        LOG_ERROR("Image preprocessing failed");
        return result;
    }

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk) {
        LOG_ERROR("Inference failed");
        return result;
    }

    // Get results
    float* output_data = output->data.f;
    float fire_score = output_data[0];    // Fire class probability
    float no_fire_score = output_data[1]; // No-fire class probability

    // Calculate confidence
    result.confidence = (uint8_t)(fire_score * 100.0f);

    // Make decision
    if (fire_score > FIRE_DETECTION_THRESHOLD) {
        result.fire_detected = true;
        LOG_INFO("Fire detected! Confidence: %d%%", result.confidence);
    } else {
        LOG_DEBUG("No fire detected. Fire score: %.3f", fire_score);
    }

    return result;
}

bool FireDetection::preprocessImage(camera_fb_t* frame) {
    if (!frame || frame->format != PIXFORMAT_RGB565) {
        LOG_ERROR("Invalid frame format");
        return false;
    }

    // Convert RGB565 to RGB888 and resize to 96x96
    uint8_t* rgb888 = (uint8_t*)input->data.uint8;

    for (int y = 0; y < MODEL_INPUT_HEIGHT; y++) {
        for (int x = 0; x < MODEL_INPUT_WIDTH; x++) {
            // Simple bilinear interpolation for resizing
            int src_x = (x * frame->width) / MODEL_INPUT_WIDTH;
            int src_y = (y * frame->height) / MODEL_INPUT_HEIGHT;

            // Get RGB565 pixel
            uint16_t rgb565 = ((uint16_t*)frame->buf)[src_y * frame->width + src_x];

            // Convert to RGB888
            uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565 & 0x1F) * 255 / 31;

            // Store in input tensor (RGB order)
            int pixel_index = (y * MODEL_INPUT_WIDTH + x) * 3;
            rgb888[pixel_index + 0] = r;
            rgb888[pixel_index + 1] = g;
            rgb888[pixel_index + 2] = b;
        }
    }

    return true;
}

void FireDetection::getModelInfo() {
    if (!initialized) return;

    LOG_INFO("Model Information:");
    LOG_INFO("- Arena size: %d bytes", kTensorArenaSize);
    LOG_INFO("- Input tensor size: %d bytes", input->bytes);
    LOG_INFO("- Output tensor size: %d bytes", output->bytes);
    LOG_INFO("- Free heap after init: %d bytes", ESP.getFreeHeap());
}
```

### Camera Manager

```cpp
// camera_ai/src/camera_manager.cpp
#include "camera_manager.h"
#include "esp_camera.h"
#include "logger.h"

CameraManager::CameraManager() :
    initialized(false)
{
}

bool CameraManager::init() {
    LOG_INFO("Initializing camera");

    // Configure camera
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA; // 320x240 for processing
    config.jpeg_quality = 12;
    config.fb_count = 2;

    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        LOG_ERROR("Camera initialization failed: %s", esp_err_to_name(err));
        return false;
    }

    // Configure camera settings for outdoor use
    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6
    s->set_whitebal(s, 1);       // 0 = disable, 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable, 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable, 1 = enable
    s->set_aec2(s, 0);           // 0 = disable, 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable, 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable, 1 = enable
    s->set_wpc(s, 1);            // 0 = disable, 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable, 1 = enable
    s->set_lenc(s, 1);           // 0 = disable, 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
    s->set_vflip(s, 0);          // 0 = disable, 1 = enable
    s->set_dcw(s, 1);            // 0 = disable, 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable, 1 = enable

    LOG_INFO("Camera initialized successfully");
    initialized = true;
    return true;
}

camera_fb_t* CameraManager::captureFrame() {
    if (!initialized) {
        LOG_ERROR("Camera not initialized");
        return nullptr;
    }

    // Capture frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        LOG_ERROR("Frame capture failed");
        return nullptr;
    }

    LOG_DEBUG("Frame captured: %dx%d, format: %d", fb->width, fb->height, fb->format);
    return fb;
}

void CameraManager::releaseFrame(camera_fb_t* fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

bool CameraManager::setFrameSize(framesize_t frameSize) {
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, frameSize);
        LOG_INFO("Frame size set to: %d", frameSize);
        return true;
    }
    return false;
}
```

### Configuration Files

```cpp
// shared/config/pin_config.h
#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ESP32 Flight Controller Pins
#define MOTOR1_PIN          12  // Front Left
#define MOTOR2_PIN          13  // Front Right
#define MOTOR3_PIN          14  // Rear Right
#define MOTOR4_PIN          15  // Rear Left

#define GPS_TX_PIN          17
#define GPS_RX_PIN          16
#define GPS_BAUD_RATE       9600

#define LORA_CS_PIN         5
#define LORA_RST_PIN        4
#define LORA_DIO0_PIN       2
#define LORA_SCK_PIN        18
#define LORA_MISO_PIN       19
#define LORA_MOSI_PIN       23

#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define ESP32_CAM_I2C_ADDRESS 0x42

#define BATTERY_ADC_PIN     34
#define STATUS_LED_PIN      25
#define BUZZER_PIN          26

// ESP32-CAM Pins
#define PWDN_GPIO_NUM       32
#define RESET_GPIO_NUM      33
#define XCLK_GPIO_NUM       4
#define SIOD_GPIO_NUM       18
#define SIOC_GPIO_NUM       23

#define Y9_GPIO_NUM         36
#define Y8_GPIO_NUM         37
#define Y7_GPIO_NUM         38
#define Y6_GPIO_NUM         39
#define Y5_GPIO_NUM         35
#define Y4_GPIO_NUM         34
#define Y3_GPIO_NUM         5
#define Y2_GPIO_NUM         14
#define VSYNC_GPIO_NUM      25
#define HREF_GPIO_NUM       23
#define PCLK_GPIO_NUM       22

#endif
```

```cpp
// shared/config/system_config.h
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Flight Control Parameters
#define PID_ROLL_P          2.0f
#define PID_ROLL_I          0.1f
#define PID_ROLL_D          0.5f
#define PID_ROLL_MAX        400.0f

#define PID_PITCH_P         2.0f
#define PID_PITCH_I         0.1f
#define PID_PITCH_D         0.5f
#define PID_PITCH_MAX       400.0f

#define PID_YAW_P           3.0f
#define PID_YAW_I           0.0f
#define PID_YAW_D          0.0f
#define PID_YAW_MAX        400.0f

#define PID_ALT_P           1.5f
#define PID_ALT_I           0.8f
#define PID_ALT_D           0.3f
#define PID_ALT_MAX        500.0f

// Motor Parameters
#define MOTOR_MIN_THROTTLE  0.0f
#define MOTOR_MAX_THROTTLE  1.0f
#define MOTOR_MIN_PULSE     1000
#define MOTOR_MAX_PULSE     2000

// AI Model Parameters
#define MODEL_INPUT_WIDTH   96
#define MODEL_INPUT_HEIGHT  96
#define FIRE_DETECTION_THRESHOLD 0.7f
#define TENSOR_ARENA_SIZE   (64 * 1024)

// LoRa Parameters
#define LORA_FREQUENCY      433E6
#define LORA_BANDWIDTH      125E3
#define LORA_SPREADING_FACTOR 9
#define LORA_CODING_RATE    5
#define LORA_TX_POWER       20
#define LORA_PREAMBLE_LENGTH 8

// GPS Parameters
#define GPS_MIN_SATELLITES  4

// Protocol Headers
#define ALERT_HEADER        0xAA
#define TELEMETRY_HEADER    0xBB

// I2C Commands
#define CMD_REQUEST_DATA    0x01
#define REQUEST_FIRE_STATUS 0x10
#define REQUEST_FIRE_DATA   0x11

#endif
```

## PlatformIO Configuration

### Flight Controller platformio.ini

```ini
[env:esp32-flight-controller]
platform = espressif32
board = esp32dev
framework = arduino

; Build options
build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue

; Libraries
lib_deps =
    madhephaestus/ESP32Servo@^1.1.1
    mikalhart/TinyGPSPlus@^1.0.2
    sandeepmistry/LoRa@^0.8.0

; Serial Monitor options
monitor_speed = 115200
monitor_filters = esp32_exception_decoder

; Upload options
upload_speed = 921600
```

### ESP32-CAM platformio.ini

```ini
[env:esp32-camera]
platform = espressif32
board = esp32dev
framework = arduino

; Build options
build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue

; Libraries
lib_deps =
    esphome/ESP32-camera@^2.0.0
    tensorflow/tensorflow-lite@^2.1.0-ALPHA

; Serial Monitor options
monitor_speed = 115200
monitor_filters = esp32_exception_decoder

; Upload options
upload_speed = 921600
```

This comprehensive firmware architecture provides a solid foundation for the wildfire surveillance drone system, with proper separation of concerns, real-time task scheduling, and robust error handling.
