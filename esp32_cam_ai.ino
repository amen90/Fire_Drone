#include "esp_camera.h"
#include <Wire.h>
#include <WiFi.h>
#include <TensorFlowLite_ESP32.h>
#include "model_data.h"  // Include your trained model data

// Camera Pin Configuration for AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    33
#define XCLK_GPIO_NUM      4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23
#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM       14
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// System Configuration
#define I2C_SLAVE_ADDRESS 0x42
#define MODEL_INPUT_WIDTH 96
#define MODEL_INPUT_HEIGHT 96
#define FIRE_DETECTION_THRESHOLD 0.7
#define INFERENCE_INTERVAL 2000  // 2 seconds between inferences
#define MAX_CONFIDENCE_HISTORY 10

// Global Variables
bool cameraInitialized = false;
bool modelInitialized = false;
camera_fb_t* fb = NULL;
unsigned long lastInferenceTime = 0;

// Fire Detection Results
bool fireDetected = false;
uint8_t fireConfidence = 0;
int8_t ambientTemperature = 25;  // Placeholder for temperature sensor

// Confidence History (for stability)
uint8_t confidenceHistory[MAX_CONFIDENCE_HISTORY];
int historyIndex = 0;

// TensorFlow Lite Variables
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor = nullptr;
TfLiteTensor* output_tensor = nullptr;

// Tensor Arena (adjust size based on your model)
constexpr int kTensorArenaSize = 64 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Model and Resolver
const tflite::Model* model = nullptr;
tflite::MicroMutableOpResolver<5> resolver; // Adjust based on your model operations

// Function Declarations
bool initializeCamera();
bool initializeModel();
bool loadModel();
void captureAndProcessImage();
void runInference();
void preprocessImage();
float getFireConfidence();
void updateConfidenceHistory(uint8_t confidence);
uint8_t getStableConfidence();
void sendResultsToFlightController();
void onI2CReceive(int len);
void onI2CRequest();

// I2C Command Handlers
#define CMD_REQUEST_FIRE_STATUS 0x01
#define CMD_REQUEST_FIRE_DATA   0x02

// Setup Function
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-CAM AI System Starting...");

    // Initialize I2C as slave
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);
    Serial.printf("I2C Slave initialized at address 0x%02X\n", I2C_SLAVE_ADDRESS);

    // Initialize camera
    if (initializeCamera()) {
        Serial.println("Camera initialized successfully");
    } else {
        Serial.println("Camera initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    // Initialize AI model
    if (initializeModel()) {
        Serial.println("AI model initialized successfully");
    } else {
        Serial.println("AI model initialization failed!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("ESP32-CAM AI System ready!");
}

// Main Loop
void loop() {
    // Run inference at regular intervals
    if (millis() - lastInferenceTime > INFERENCE_INTERVAL) {
        captureAndProcessImage();
        lastInferenceTime = millis();
    }

    // Small delay to prevent watchdog timeout
    delay(10);
}

// Camera Initialization
bool initializeCamera() {
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

    // Frame size for AI processing (96x96 is optimal for small models)
    config.frame_size = FRAMESIZE_QVGA;  // 320x240, will be cropped/resized to 96x96

    config.jpeg_quality = 12;
    config.fb_count = 1;  // Single frame buffer for AI processing

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    // Configure camera settings for outdoor use
    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);      // -2 to 2
        s->set_contrast(s, 0);        // -2 to 2
        s->set_saturation(s, 0);      // -2 to 2
        s->set_whitebal(s, 1);        // 0=disable, 1=enable
        s->set_awb_gain(s, 1);        // 0=disable, 1=enable
        s->set_wb_mode(s, 0);         // 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
        s->set_exposure_ctrl(s, 1);   // 0=disable, 1=enable
        s->set_aec2(s, 0);            // 0=disable, 1=enable
        s->set_ae_level(s, 0);        // -2 to 2
        s->set_aec_value(s, 300);     // 0 to 1200
        s->set_gain_ctrl(s, 1);       // 0=disable, 1=enable
        s->set_agc_gain(s, 0);        // 0 to 30
        s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
        s->set_bpc(s, 0);             // 0=disable, 1=enable
        s->set_wpc(s, 1);             // 0=disable, 1=enable
        s->set_raw_gma(s, 1);         // 0=disable, 1=enable
        s->set_lenc(s, 1);            // 0=disable, 1=enable
        s->set_hmirror(s, 0);         // 0=disable, 1=enable
        s->set_vflip(s, 0);           // 0=disable, 1=enable
        s->set_dcw(s, 1);             // 0=disable, 1=enable
        s->set_colorbar(s, 0);        // 0=disable, 1=enable
    }

    cameraInitialized = true;
    return true;
}

// Model Initialization
bool initializeModel() {
    // Map the model into a usable data structure
    model = tflite::GetModel(model_data);

    if (model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.printf("Model provided is schema version %d not equal to supported version %d.\n",
                     model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    // Add operations to resolver (adjust based on your model)
    resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                       tflite::ops::micro::Register_CONV_2D());
    resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                       tflite::ops::micro::Register_MAX_POOL_2D());
    resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                       tflite::ops::micro::Register_RESHAPE());
    resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                       tflite::ops::micro::Register_FULLY_CONNECTED());
    resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                       tflite::ops::micro::Register_SOFTMAX());

    // Build an interpreter
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        Serial.println("AllocateTensors() failed");
        return false;
    }

    // Get pointers to input and output tensors
    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    Serial.printf("Model input shape: %d x %d x %d\n",
                 input_tensor->dims->data[1],
                 input_tensor->dims->data[2],
                 input_tensor->dims->data[3]);
    Serial.printf("Model output shape: %d\n", output_tensor->dims->data[1]);

    modelInitialized = true;
    return true;
}

// Main Processing Function
void captureAndProcessImage() {
    if (!cameraInitialized || !modelInitialized) {
        return;
    }

    // Capture frame
    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    Serial.printf("Captured frame: %dx%d, format: %d\n",
                 fb->width, fb->height, fb->format);

    // Process the frame
    runInference();

    // Return frame buffer
    esp_camera_fb_return(fb);
    fb = NULL;
}

// Run AI Inference
void runInference() {
    if (!fb) return;

    // Preprocess image (resize, convert format)
    preprocessImage();

    // Run inference
    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("Inference failed");
        return;
    }

    // Get results
    float fire_score = output_tensor->data.f[0];    // Fire class probability
    float no_fire_score = output_tensor->data.f[1]; // No-fire class probability

    Serial.printf("Inference results - Fire: %.3f, No-fire: %.3f\n",
                 fire_score, no_fire_score);

    // Determine fire detection
    fireDetected = (fire_score > FIRE_DETECTION_THRESHOLD);
    fireConfidence = (uint8_t)(fire_score * 100.0f);

    // Update confidence history for stability
    updateConfidenceHistory(fireConfidence);
    uint8_t stableConfidence = getStableConfidence();

    // Update final results
    fireDetected = (stableConfidence > (FIRE_DETECTION_THRESHOLD * 100));
    fireConfidence = stableConfidence;

    Serial.printf("Stable confidence: %d%%, Fire detected: %s\n",
                 fireConfidence, fireDetected ? "YES" : "NO");

    // Send results to flight controller if fire detected
    if (fireDetected) {
        sendResultsToFlightController();
    }
}

// Image Preprocessing
void preprocessImage() {
    if (!fb || !input_tensor) return;

    // Convert RGB565 to RGB888 and resize to 96x96
    uint8_t* rgb888 = input_tensor->data.uint8;

    for (int y = 0; y < MODEL_INPUT_HEIGHT; y++) {
        for (int x = 0; x < MODEL_INPUT_WIDTH; x++) {
            // Bilinear interpolation for resizing
            int src_x = (x * fb->width) / MODEL_INPUT_WIDTH;
            int src_y = (y * fb->height) / MODEL_INPUT_HEIGHT;

            // Get RGB565 pixel
            uint16_t rgb565 = ((uint16_t*)fb->buf)[src_y * fb->width + src_x];

            // Convert to RGB888
            uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
            uint8_t g = ((rgb565 >> 5) & 0x3F) * 255 / 63;
            uint8_t b = (rgb565 & 0x1F) * 255 / 31;

            // Store in input tensor (RGB order for most models)
            int pixel_index = (y * MODEL_INPUT_WIDTH + x) * 3;
            rgb888[pixel_index + 0] = r;
            rgb888[pixel_index + 1] = g;
            rgb888[pixel_index + 2] = b;
        }
    }

    Serial.println("Image preprocessing completed");
}

// Confidence History Management
void updateConfidenceHistory(uint8_t confidence) {
    confidenceHistory[historyIndex] = confidence;
    historyIndex = (historyIndex + 1) % MAX_CONFIDENCE_HISTORY;
}

uint8_t getStableConfidence() {
    // Return average of last few readings for stability
    int validReadings = 0;
    int sum = 0;

    for (int i = 0; i < MAX_CONFIDENCE_HISTORY; i++) {
        if (confidenceHistory[i] > 0) {  // Only count non-zero readings
            sum += confidenceHistory[i];
            validReadings++;
        }
    }

    if (validReadings == 0) return 0;

    return (uint8_t)(sum / validReadings);
}

// Send Results to Flight Controller
void sendResultsToFlightController() {
    // This function would be called when fire is detected
    // The actual sending happens through I2C when requested
    Serial.printf("Fire alert ready - Confidence: %d%%\n", fireConfidence);
}

// I2C Communication Handlers
void onI2CReceive(int len) {
    Serial.printf("I2C Receive: %d bytes\n", len);

    if (len >= 1) {
        uint8_t command = Wire.read();
        Serial.printf("I2C Command: 0x%02X\n", command);

        // Process additional data if any
        for (int i = 1; i < len; i++) {
            uint8_t data = Wire.read();
            Serial.printf("Data[%d]: 0x%02X\n", i, data);
        }
    }
}

void onI2CRequest() {
    Serial.println("I2C Request received");

    // Send fire detection data (3 bytes: fire_detected, confidence, temperature)
    uint8_t response[3];
    response[0] = fireDetected ? 1 : 0;
    response[1] = fireConfidence;
    response[2] = ambientTemperature;

    Wire.write(response, 3);

    Serial.printf("Sent I2C response: Fire=%d, Conf=%d%%, Temp=%d°C\n",
                 response[0], response[1], response[2]);
}

// Utility Functions
void printHeapInfo() {
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Min free heap: %d bytes\n", ESP.getMinFreeHeap());
    Serial.printf("Max alloc heap: %d bytes\n", ESP.getMaxAllocHeap());
}

void restartSystem() {
    Serial.println("Restarting system...");
    delay(1000);
    ESP.restart();
}

// Serial Debug Interface
void processSerialCommand(String command) {
    command.trim();

    if (command == "STATUS") {
        Serial.printf("Camera: %s, Model: %s\n",
                     cameraInitialized ? "OK" : "FAIL",
                     modelInitialized ? "OK" : "FAIL");
        printHeapInfo();
    } else if (command == "CAPTURE") {
        captureAndProcessImage();
    } else if (command == "HEAP") {
        printHeapInfo();
    } else if (command == "RESTART") {
        restartSystem();
    } else {
        Serial.println("Available commands: STATUS, CAPTURE, HEAP, RESTART");
    }
}

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

// Model Data Placeholder (Replace with your actual model)
// This should be replaced with your trained model's C array
// Generated by converting .tflite to .h file
#ifndef MODEL_DATA_H
#define MODEL_DATA_H

#include <cstdint>

const unsigned int model_data_len = 1000;  // Replace with actual model size
const unsigned char model_data[] = {
    // Replace with your actual model data
    // This is just a placeholder - you need to generate this from your .tflite file
    0x18, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4C, 0x33, 0x00, 0x00, 0x00, 0x00,
    // ... your model data here ...
};

#endif // MODEL_DATA_H

