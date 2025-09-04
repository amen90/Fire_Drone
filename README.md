# 🔥 Autonomous Wildfire Surveillance Drone

<div align="center">

**Real-Time Fire Detection & Alert System using AI & LoRa Communication**

[![ESP32](https://img.shields.io/badge/ESP32-Real--Time%20AI-green?style=flat-square)](https://www.espressif.com/en/products/som/esp32)
[![TensorFlow](https://img.shields.io/badge/TensorFlow-Lite-orange?style=flat-square)](https://www.tensorflow.org/lite)
[![LoRa](https://img.shields.io/badge/LoRa-Long--Range-blue?style=flat-square)](https://lora-alliance.org/)

*An autonomous quadcopter system for real-time wildfire detection, GPS tracking, and long-range alert transmission.*

[🚀 Quick Start](#-quick-start) • [📦 Features](#-key-features) • [🔧 Hardware](#-hardware-setup) • [📚 Documentation](#-documentation)

---

## 🌟 Overview

This project implements a complete autonomous drone system for wildfire surveillance combining:

- **AI-Powered Detection**: Real-time fire classification using ESP32-CAM
- **Autonomous Flight**: PID-controlled quadcopter with GPS navigation
- **Long-Range Communication**: LoRa-based alert system (up to 10km range)
- **Ground Monitoring**: Interactive map visualization and data logging

### Key Features
- **Real-time Fire Detection**: AI model running on ESP32-CAM for live fire classification
- **Long-range Communication**: LoRa-based alert system with GPS coordinates
- **Autonomous Flight**: PID-controlled drone with waypoint navigation
- **Ground Station**: Python-based dashboard for monitoring and visualization
- **Low Power Design**: Optimized for 15-20 minute flight times
- **Robust Operation**: Designed for mountainous terrain and harsh conditions

## 🏗️ System Architecture

### High-Level Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Drone System  │    │   LoRa Network   │    │ Ground Station  │
│                 │    │                  │    │                 │
│ ┌─────────────┐ │    │ ┌──────────────┐ │    │ ┌─────────────┐ │
│ │  ESP32-CAM  │◄┼────┼►│  LoRa Module │◄┼────┼►│   LoRa RX   │ │
│ │  (AI + Cam) │ │    │ │   (SX1278)   │ │    │ │             │ │
│ └─────────────┘ │    │ └──────────────┘ │    │ └─────────────┘ │
│        ▲        │    │         ▲        │    │        ▲        │
│        │        │    │         │        │    │        │        │
│ ┌─────────────┐ │    │         │        │    │ ┌─────────────┐ │
│ │   ESP32 FC  │◄┼────┼─────────┘        │    │ │  Python App │ │
│ │  (Flight C) │ │    │                  │    │ │  (Dashboard)│ │
│ └─────────────┘ │    │                  │    │ └─────────────┘ │
│        ▲        │    │                  │    │        │        │
│        │        │    │                  │    │        ▼        │
│ ┌─────────────┐ │    │                  │    │ ┌─────────────┐ │
│ │   GPS NEO-6M│◄┼────┼──────────────────┼────┼►│   Map View  │ │
│ └─────────────┘ │    │                  │    │ │  (Folium)   │ │
│                 │    │                  │    │ └─────────────┘ │
│ ┌─────────────┐ │    │                  │    │                 │
│ │ Motors &    │ │    │                  │    │ ┌─────────────┐ │
│ │ ESCs        │ │    │                  │    │ │   CSV Logs  │ │
│ └─────────────┘ │    │                  │    │ └─────────────┘ │
│                 │    │                  │    │                 │
│ ┌─────────────┐ │    │                  │    │                 │
│ │ LiPo Battery│ │    │                  │    │                 │
│ │  (3S 2200mAh)│ │    │                  │    │                 │
│ └─────────────┘ │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Component Breakdown

#### 1. **Drone System Components**
- **ESP32-CAM**: Primary microcontroller with camera for AI inference and image capture
- **ESP32 Flight Controller**: Handles flight control, motor management, and sensor fusion
- **GPS Module**: Provides location data for navigation and alert coordinates
- **LoRa Module**: Long-range communication for alerts and telemetry
- **Power System**: LiPo battery with distribution board
- **Motors & ESCs**: Quadcopter propulsion system

#### 2. **Communication Flow**
- **Fire Detection** → ESP32-CAM processes images → AI classification
- **Alert Generation** → Fire detected → GPS coordinates captured → LoRa packet sent
- **Ground Reception** → LoRa gateway receives → Python app processes → Map visualization

#### 3. **Data Flow Architecture**

```
Camera Feed → Image Preprocessing → AI Model Inference → Fire Detection
      ↓              ↓                    ↓              ↓
   ESP32-CAM    Resize/Crop        TensorFlow Lite    Classification
      ↓              ↓                    ↓              ↓
   I2C/Serial    Memory Buffer      Model Output       Decision Logic
      ↓              ↓                    ↓              ↓
   ESP32-FC     Flight Controller   Alert Trigger      LoRa Transmission
```

### Software Architecture Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐  │
│  │  Fire Detection │  │  Flight Control │  │ Alert System│  │
│  │     Engine      │  │     Manager     │  │   Manager   │  │
│  └─────────────────┘  └─────────────────┘  └─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                    Middleware Layer                         │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐  │
│  │   LoRa Protocol │  │   GPS Interface │  │  Camera API │  │
│  │    Handler      │  │    Manager      │  │   Manager   │  │
│  └─────────────────┘  └─────────────────┘  └─────────────┘  │
├─────────────────────────────────────────────────────────────┤
│                     Hardware Layer                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐  │
│  │   ESP32-CAM     │  │   ESP32 FC      │  │  SX1278 LoRa│  │
│  │   Peripherals   │  │   Peripherals   │  │   Module    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## ⚙️ Hardware Specifications

### Core Components

#### **Microcontrollers**
- **ESP32-WROOM-32** (Flight Controller)
  - Dual-core Xtensa LX6 microprocessor
  - 520 KB SRAM, 448 KB ROM
  - WiFi 802.11 b/g/n, Bluetooth 4.2
  - 38 programmable GPIOs
  - Operating voltage: 2.2-3.6V
  - **Justification**: Powerful enough for flight control algorithms, sensor fusion, and communication

- **ESP32-CAM** (AI + Camera)
  - Same ESP32 core as above
  - OV2640 2MP camera module
  - MicroSD card slot for storage
  - **Justification**: Integrated camera for computer vision, sufficient processing for TinyML inference

#### **Communication Modules**
- **SX1278 LoRa Module** (433MHz)
  - Frequency: 410-525MHz
  - Max transmission power: +20dBm
  - Sensitivity: -148dBm
  - Range: Up to 10-20km (line of sight)
  - Data rate: 0.018-37.5 kbps
  - **Justification**: Long-range, low-power communication perfect for remote areas

- **NEO-6M GPS Module**
  - 50-channel GPS receiver
  - Accuracy: 2.5m CEP
  - Update rate: 5Hz
  - Operating voltage: 2.7-3.6V
  - **Justification**: Reliable positioning for navigation and alert coordinates

#### **Power System**
- **LiPo Battery: 3S 2200mAh 11.1V**
  - Capacity: 2200mAh
  - Voltage: 11.1V (3 cells)
  - Discharge rate: 25C
  - Weight: ~180g
  - **Justification**: Balances flight time (15-20 min) with weight constraints

- **Power Distribution Board**
  - 5V/3A and 3.3V/1A regulators
  - Battery voltage monitoring
  - Power switch and protection
  - **Justification**: Clean power for sensitive electronics

#### **Propulsion System**
- **Brushless Motors: 2205 2300KV**
  - Size: 2205 (22mm diameter, 5mm shaft)
  - KV rating: 2300 RPM/V
  - Weight: 28g each
  - **Justification**: High power-to-weight ratio for agile flight

- **Electronic Speed Controllers: 20A BLHeli**
  - Current rating: 20A continuous
  - Voltage: 2-4S LiPo
  - Protocol: PWM/DShot
  - **Justification**: Reliable motor control with good efficiency

- **Propellers: 5045 (5x4.5 inch)**
  - Size: 5x4.5 inch
  - Pitch: 4.5 inch
  - Material: Polycarbonate
  - **Justification**: Good thrust-to-weight ratio for the motor size

#### **Frame**
- **250mm Quadcopter Frame**
  - Wheelbase: 250mm
  - Material: Carbon fiber
  - Weight: 120g
  - **Justification**: Lightweight, durable, good for payload

### Hardware BOM (Bill of Materials)

| Component | Model | Quantity | Cost Estimate | Supplier |
|-----------|-------|----------|---------------|----------|
| ESP32-WROOM-32 | ESP32-DevKitC | 1 | $8 | Amazon/AliExpress |
| ESP32-CAM | AI-Thinker | 1 | $7 | Amazon/AliExpress |
| LoRa Module | SX1278 433MHz | 1 | $5 | Amazon/AliExpress |
| GPS Module | NEO-6M | 1 | $10 | Amazon/AliExpress |
| Brushless Motors | 2205 2300KV | 4 | $40 | Amazon/AliExpress |
| ESCs | 20A BLHeli | 4 | $40 | Amazon/AliExpress |
| Propellers | 5045 PC | 4 | $5 | Amazon/AliExpress |
| LiPo Battery | 3S 2200mAh | 1 | $15 | Amazon/AliExpress |
| Frame | 250mm Carbon Fiber | 1 | $20 | Amazon/AliExpress |
| Power Board | Custom/Generic | 1 | $10 | Amazon/AliExpress |
| **Total** | | | **$160** | |

## 🔌 Wiring Diagrams

### ESP32 Flight Controller Pinout

```
ESP32-WROOM-32 Pin Assignments:

Motor Control:
- GPIO 12: Motor 1 (Front Left) - ESC PWM
- GPIO 13: Motor 2 (Front Right) - ESC PWM
- GPIO 14: Motor 3 (Rear Right) - ESC PWM
- GPIO 15: Motor 4 (Rear Left) - ESC PWM

GPS Interface:
- GPIO 16: GPS RX
- GPIO 17: GPS TX

LoRa Interface:
- GPIO 18: LoRa SCK (SPI)
- GPIO 19: LoRa MISO (SPI)
- GPIO 23: LoRa MOSI (SPI)
- GPIO 5: LoRa CS (Chip Select)
- GPIO 4: LoRa RST (Reset)
- GPIO 2: LoRa DIO0 (Interrupt)

Power & Sensors:
- GPIO 34: Battery Voltage ADC
- GPIO 35: Current Sensor ADC
- 3.3V: Sensor power
- GND: Common ground

I2C Bus (for future sensors):
- GPIO 21: I2C SDA
- GPIO 22: I2C SCL
```

### ESP32-CAM Pinout

```
ESP32-CAM Pin Assignments:

Camera Interface:
- GPIO 0: Camera PWDN
- GPIO 32: Camera RESET
- GPIO 4: Camera XCLK
- GPIO 18: Camera PCLK
- GPIO 33: Camera VSYNC
- GPIO 19: Camera HREF
- GPIO 36: Camera D0
- GPIO 39: Camera D1
- GPIO 34: Camera D2
- GPIO 35: Camera D3
- GPIO 25: Camera D4
- GPIO 26: Camera D5
- GPIO 27: Camera D6
- GPIO 14: Camera D7

Communication:
- GPIO 12: I2C SDA (to ESP32-FC)
- GPIO 13: I2C SCL (to ESP32-FC)
- GPIO 15: Ready Signal (to ESP32-FC)

Power:
- 3.3V: From power board
- GND: Common ground
```

### Power Distribution Schematic

```
LiPo Battery (11.1V)
      │
      ├──┬─── Power Switch
      │  │
      ├──┼─── Voltage Regulator (5V/3A) ───┬─── ESCs (11.1V)
      │  │                                  │
      ├──┼─── Voltage Regulator (3.3V/1A) ─┼─── ESP32-FC (3.3V)
      │  │                                  │
      │                                     ├─── ESP32-CAM (3.3V)
      │                                     │
      │                                     ├─── GPS Module (3.3V)
      │                                     │
      │                                     └─── LoRa Module (3.3V)
      │
      └──┬─── Battery Monitor ADC (ESP32 GPIO 34)
```

### Communication Wiring

```
ESP32-FC          ESP32-CAM
GPIO 21 ──────── GPIO 12 (SDA)
GPIO 22 ──────── GPIO 13 (SCL)
GPIO 15 ◄─────── GPIO 15 (Ready)

ESP32-FC          LoRa SX1278
GPIO 18 ──────── SCK
GPIO 19 ──────── MISO
GPIO 23 ──────── MOSI
GPIO 5  ──────── CS
GPIO 4  ──────── RST
GPIO 2  ◄─────── DIO0

ESP32-FC          GPS NEO-6M
GPIO 16 ◄─────── TX
GPIO 17 ──────── RX
```

## 💻 Firmware Architecture

### ESP32 Flight Controller Firmware

#### Directory Structure
```
firmware/
├── flight_controller/
│   ├── src/
│   │   ├── main.cpp                 # Main entry point
│   │   ├── flight_control.h         # Flight control declarations
│   │   ├── flight_control.cpp       # PID controllers & motor control
│   │   ├── gps_manager.h            # GPS interface declarations
│   │   ├── gps_manager.cpp          # GPS data parsing & management
│   │   ├── lora_manager.h           # LoRa communication declarations
│   │   ├── lora_manager.cpp         # LoRa packet handling
│   │   ├── sensor_fusion.h          # Sensor fusion declarations
│   │   ├── sensor_fusion.cpp        # IMU/MPU6050 integration
│   │   └── config.h                 # Pin definitions & constants
│   ├── lib/
│   │   ├── PID/                     # PID controller library
│   │   ├── TinyGPS/                 # GPS parsing library
│   │   └── LoRa/                    # LoRa communication library
│   └── platformio.ini              # PlatformIO configuration
```

#### Main Control Loop

```cpp
// Main flight control loop (runs at 100Hz)
void flight_control_task(void *pvParameters) {
    while (true) {
        // Read sensor data
        read_imu_data();
        read_gps_data();

        // Update PID controllers
        update_pid_controllers();

        // Calculate motor outputs
        calculate_motor_speeds();

        // Send commands to ESCs
        update_motor_outputs();

        // Check for fire alerts from ESP32-CAM
        check_fire_alerts();

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz loop
    }
}
```

### ESP32-CAM Firmware

#### Directory Structure
```
firmware/
├── camera_ai/
│   ├── src/
│   │   ├── main.cpp                 # Main entry point
│   │   ├── camera_capture.h         # Camera control declarations
│   │   ├── camera_capture.cpp       # Image capture & preprocessing
│   │   ├── ai_inference.h           # AI model declarations
│   │   ├── ai_inference.cpp         # TensorFlow Lite inference
│   │   ├── communication.h          # Inter-ESP communication
│   │   └── communication.cpp        # I2C communication with ESP32-FC
│   ├── models/
│   │   └── fire_detection.tflite    # Trained TensorFlow Lite model
│   └── platformio.ini              # PlatformIO configuration
```

#### AI Inference Pipeline

```cpp
// Fire detection inference loop (runs at 2Hz)
void ai_inference_task(void *pvParameters) {
    while (true) {
        // Capture image from camera
        camera_fb_t *fb = esp_camera_fb_get();

        // Preprocess image (resize to 96x96, RGB565 to RGB888)
        preprocess_image(fb);

        // Run TensorFlow Lite inference
        TfLiteStatus status = interpreter->Invoke();

        // Get classification results
        float fire_confidence = get_fire_confidence();

        // Make decision
        if (fire_confidence > FIRE_THRESHOLD) {
            send_fire_alert(fire_confidence);
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz inference
    }
}
```

## 🧠 AI Pipeline for Fire Detection

### Dataset Collection Strategy

#### Data Sources
1. **Public Datasets**
   - FireNet Dataset (Kaggle)
   - Forest Fire Dataset
   - Custom captured images

2. **Data Collection Protocol**
   - Capture from various altitudes (10m, 20m, 50m)
   - Different lighting conditions (dawn, midday, dusk)
   - Various fire sizes (small flames to large fires)
   - Mountainous terrain backgrounds
   - Similar non-fire scenes (sunsets, campfires, machinery)

#### Dataset Structure
```
dataset/
├── fire/
│   ├── small_fire_001.jpg
│   ├── medium_fire_002.jpg
│   └── large_fire_003.jpg
├── no_fire/
│   ├── forest_001.jpg
│   ├── mountain_002.jpg
│   └── sunset_003.jpg
└── annotations/
    ├── fire_labels.json
    └── preprocessing_config.json
```

### Model Architecture

#### TinyML Fire Detection Model

```python
# Model definition for fire detection
def create_fire_detection_model():
    model = tf.keras.Sequential([
        tf.keras.layers.Input(shape=(96, 96, 3)),
        tf.keras.layers.Conv2D(16, (3, 3), activation='relu'),
        tf.keras.layers.MaxPooling2D((2, 2)),
        tf.keras.layers.Conv2D(32, (3, 3), activation='relu'),
        tf.keras.layers.MaxPooling2D((2, 2)),
        tf.keras.layers.Conv2D(64, (3, 3), activation='relu'),
        tf.keras.layers.MaxPooling2D((2, 2)),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dropout(0.5),
        tf.keras.layers.Dense(2, activation='softmax')  # Fire/No-Fire
    ])
    return model

# Compile model
model = create_fire_detection_model()
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])
```

#### Training Pipeline

```python
# Training script
def train_fire_detection_model():
    # Data preprocessing
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        validation_split=0.2
    )

    # Load datasets
    train_generator = train_datagen.flow_from_directory(
        'dataset/',
        target_size=(96, 96),
        batch_size=32,
        class_mode='categorical',
        subset='training'
    )

    validation_generator = train_datagen.flow_from_directory(
        'dataset/',
        target_size=(96, 96),
        batch_size=32,
        class_mode='categorical',
        subset='validation'
    )

    # Train model
    history = model.fit(
        train_generator,
        epochs=50,
        validation_data=validation_generator,
        callbacks=[early_stopping, model_checkpoint]
    )

    return model, history
```

### TensorFlow Lite Conversion

```python
# Convert to TensorFlow Lite
def convert_to_tflite(model_path, output_path):
    # Load trained model
    model = tf.keras.models.load_model(model_path)

    # Convert to TFLite
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8

    # Quantize model
    def representative_dataset_gen():
        for image in representative_images:
            image = tf.expand_dims(image, 0)
            yield [image]

    converter.representative_dataset = representative_dataset_gen
    tflite_model = converter.convert()

    # Save quantized model
    with open(output_path, 'wb') as f:
        f.write(tflite_model)

    return tflite_model
```

### Model Performance Targets
- **Accuracy**: >95% on test set
- **Model Size**: <100KB (for ESP32 flash constraints)
- **Inference Time**: <500ms per frame
- **Memory Usage**: <50KB RAM during inference

## 📡 LoRa Alert System Protocol

### Message Format Specification

#### Alert Packet Structure
```
Packet Format (32 bytes total):
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Header  │ Msg ID  │ Status  │ Lat MSB │ Lat LSB │ Lon MSB │ Lon LSB │ Time    │
│ (1 byte)│ (1 byte)│ (1 byte)│ (4 bytes│ (4 bytes│ (4 bytes│ (4 bytes│ (8 bytes│
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
│ Time    │ Conf    │ Checksum│
│ (cont)  │ (1 byte)│ (2 bytes)│
└─────────┴─────────┴─────────┘
```

#### Field Definitions

| Field | Size | Description | Range |
|-------|------|-------------|-------|
| Header | 1 byte | Packet type (0xAA = Alert) | 0xAA |
| Msg ID | 1 byte | Sequential message counter | 0-255 |
| Status | 1 byte | Alert status flags | Bitfield |
| Latitude | 4 bytes | GPS latitude (float32) | -90 to +90 |
| Longitude | 4 bytes | GPS longitude (float32) | -180 to +180 |
| Timestamp | 8 bytes | Unix timestamp (uint64) | Seconds since epoch |
| Confidence | 1 byte | AI confidence score | 0-100 |
| Checksum | 2 bytes | CRC-16 checksum | 0-65535 |

#### Status Flags (Bitfield)
```
Bit 7: Reserved
Bit 6: Reserved
Bit 5: GPS Valid
Bit 4: Battery Low
Bit 3: Fire Detected
Bit 2: Large Fire (>50% confidence)
Bit 1: Critical Alert
Bit 0: System OK
```

### Example Packet Creation

```cpp
// Create fire alert packet
struct AlertPacket {
    uint8_t header = 0xAA;
    uint8_t message_id;
    uint8_t status;
    float latitude;
    float longitude;
    uint64_t timestamp;
    uint8_t confidence;
    uint16_t checksum;
};

AlertPacket create_fire_alert(float lat, float lon, uint8_t conf) {
    static uint8_t msg_counter = 0;

    AlertPacket packet;
    packet.message_id = msg_counter++;
    packet.status = (1 << 3) | (1 << 5); // Fire detected + GPS valid
    packet.latitude = lat;
    packet.longitude = lon;
    packet.timestamp = get_unix_timestamp();
    packet.confidence = conf;
    packet.checksum = calculate_crc16(&packet, sizeof(packet) - 2);

    return packet;
}
```

### LoRa Configuration

```cpp
// LoRa module configuration
#define LORA_FREQUENCY 433E6        // 433MHz band
#define LORA_BANDWIDTH 125E3        // 125kHz bandwidth
#define LORA_SPREADING_FACTOR 9     // SF9 for range
#define LORA_CODING_RATE 5          // 4/5 coding rate
#define LORA_TX_POWER 20            // 20dBm transmit power
#define LORA_PREAMBLE_LENGTH 8      // 8 symbol preamble
```

## 🖥️ Ground Station Application

### Python Application Architecture

```
ground_station/
├── main.py                    # Main application entry
├── lora_receiver.py           # LoRa communication handler
├── alert_processor.py         # Alert processing logic
├── map_visualizer.py          # Folium map integration
├── data_logger.py             # CSV logging system
├── config.py                  # Configuration settings
└── requirements.txt           # Python dependencies
```

### Main Application Code

```python
# main.py
import tkinter as tk
from tkinter import ttk
import folium
import threading
import time
from lora_receiver import LoRaReceiver
from alert_processor import AlertProcessor
from map_visualizer import MapVisualizer
from data_logger import DataLogger

class GroundStationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Wildfire Surveillance Ground Station")
        self.root.geometry("1200x800")

        # Initialize components
        self.lora_receiver = LoRaReceiver()
        self.alert_processor = AlertProcessor()
        self.map_visualizer = MapVisualizer()
        self.data_logger = DataLogger()

        # Create GUI
        self.create_widgets()

        # Start background threads
        self.start_background_threads()

    def create_widgets(self):
        # Create main frames
        self.map_frame = ttk.Frame(self.root)
        self.map_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.control_frame = ttk.Frame(self.root, width=300)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Map display
        self.map_label = ttk.Label(self.map_frame, text="Fire Alert Map")
        self.map_label.pack(pady=10)

        # Control panel
        self.status_label = ttk.Label(self.control_frame,
                                    text="Status: Waiting for alerts...")
        self.status_label.pack(pady=10)

        self.alert_listbox = tk.Listbox(self.control_frame, height=10)
        self.alert_listbox.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)

        # Buttons
        self.clear_button = ttk.Button(self.control_frame,
                                     text="Clear Alerts",
                                     command=self.clear_alerts)
        self.clear_button.pack(pady=5)

        self.export_button = ttk.Button(self.control_frame,
                                      text="Export Data",
                                      command=self.export_data)
        self.export_button.pack(pady=5)

    def start_background_threads(self):
        # LoRa listening thread
        self.lora_thread = threading.Thread(target=self.lora_listener)
        self.lora_thread.daemon = True
        self.lora_thread.start()

        # Map update thread
        self.map_thread = threading.Thread(target=self.map_updater)
        self.map_thread.daemon = True
        self.map_thread.start()

    def lora_listener(self):
        while True:
            try:
                # Receive LoRa packet
                packet = self.lora_receiver.receive_packet()

                if packet:
                    # Process alert
                    alert = self.alert_processor.process_packet(packet)

                    # Log to CSV
                    self.data_logger.log_alert(alert)

                    # Update GUI
                    self.root.after(0, self.update_alert_display, alert)

                    # Update map
                    self.root.after(0, self.map_visualizer.add_alert_marker, alert)

            except Exception as e:
                print(f"LoRa receive error: {e}")

            time.sleep(0.1)

    def update_alert_display(self, alert):
        alert_text = f"ALERT: Fire at {alert['lat']:.4f}, {alert['lon']:.4f} " \
                    f"(Conf: {alert['confidence']}%)"
        self.alert_listbox.insert(0, alert_text)
        self.status_label.config(text="Status: Fire Alert Received!")

    def clear_alerts(self):
        self.alert_listbox.delete(0, tk.END)
        self.map_visualizer.clear_markers()

    def export_data(self):
        filename = f"fire_alerts_{int(time.time())}.csv"
        self.data_logger.export_to_csv(filename)
        messagebox.showinfo("Export Complete", f"Data exported to {filename}")

if __name__ == "__main__":
    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()
```

### LoRa Receiver Module

```python
# lora_receiver.py
import serial
import struct
import time

class LoRaReceiver:
    def __init__(self, port='COM3', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connect()

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to LoRa receiver on {self.port}")
        except Exception as e:
            print(f"Failed to connect to LoRa receiver: {e}")

    def receive_packet(self):
        if not self.serial:
            return None

        try:
            # Read packet header
            header = self.serial.read(1)
            if not header or header[0] != 0xAA:
                return None

            # Read remaining packet data
            data = self.serial.read(31)  # 32 bytes total - 1 for header
            if len(data) != 31:
                return None

            # Parse packet
            packet = self.parse_packet(header + data)
            return packet

        except Exception as e:
            print(f"Error receiving packet: {e}")
            return None

    def parse_packet(self, raw_data):
        try:
            # Unpack binary data
            header, msg_id, status, lat, lon, timestamp, confidence, checksum = \
                struct.unpack('>BBBL LdBBH', raw_data)

            # Verify checksum
            calculated_checksum = self.calculate_crc16(raw_data[:-2])
            if calculated_checksum != checksum:
                print("Checksum mismatch")
                return None

            return {
                'message_id': msg_id,
                'status': status,
                'latitude': lat,
                'longitude': lon,
                'timestamp': timestamp,
                'confidence': confidence,
                'fire_detected': bool(status & (1 << 3)),
                'gps_valid': bool(status & (1 << 5))
            }

        except struct.error as e:
            print(f"Packet parsing error: {e}")
            return None

    def calculate_crc16(self, data):
        # CRC-16-CCITT implementation
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc
```

### Map Visualization Module

```python
# map_visualizer.py
import folium
import webbrowser
import os
from folium.plugins import MarkerCluster

class MapVisualizer:
    def __init__(self, center_lat=37.7749, center_lon=-122.4194, zoom=10):
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.zoom = zoom
        self.map = None
        self.markers = []
        self.create_map()

    def create_map(self):
        self.map = folium.Map(
            location=[self.center_lat, self.center_lon],
            zoom_start=self.zoom,
            tiles='OpenStreetMap'
        )
        self.marker_cluster = MarkerCluster().add_to(self.map)

    def add_alert_marker(self, alert):
        if not alert or not alert.get('fire_detected'):
            return

        # Create popup content
        popup_content = f"""
        <b>Fire Alert #{alert['message_id']}</b><br>
        Confidence: {alert['confidence']}%<br>
        Time: {time.strftime('%H:%M:%S', time.localtime(alert['timestamp']))}<br>
        GPS: {'Valid' if alert['gps_valid'] else 'Invalid'}
        """

        # Choose marker color based on confidence
        if alert['confidence'] > 80:
            color = 'red'
            icon = 'fire'
        elif alert['confidence'] > 60:
            color = 'orange'
            icon = 'exclamation-triangle'
        else:
            color = 'yellow'
            icon = 'exclamation-circle'

        # Add marker
        marker = folium.Marker(
            location=[alert['latitude'], alert['longitude']],
            popup=popup_content,
            icon=folium.Icon(color=color, icon=icon)
        )

        marker.add_to(self.marker_cluster)
        self.markers.append(marker)

        # Save updated map
        self.save_map()

    def clear_markers(self):
        self.marker_cluster = MarkerCluster().add_to(self.map)
        self.markers = []
        self.save_map()

    def save_map(self, filename='fire_alerts_map.html'):
        self.map.save(filename)

        # Auto-refresh browser if map is open
        if hasattr(self, 'browser_opened') and self.browser_opened:
            # Force browser refresh (platform dependent)
            pass

    def open_in_browser(self):
        self.save_map()
        webbrowser.open('file://' + os.path.abspath('fire_alerts_map.html'))
        self.browser_opened = True
```

### Data Logger Module

```python
# data_logger.py
import csv
import os
import time
from datetime import datetime

class DataLogger:
    def __init__(self, log_directory='logs'):
        self.log_directory = log_directory
        self.current_log_file = None
        self.ensure_log_directory()

        # Create new log file for this session
        self.create_new_log_file()

    def ensure_log_directory(self):
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)

    def create_new_log_file(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'fire_alerts_{timestamp}.csv'
        self.current_log_file = os.path.join(self.log_directory, filename)

        # Write CSV header
        with open(self.current_log_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp', 'message_id', 'latitude', 'longitude',
                'confidence', 'fire_detected', 'gps_valid', 'status_flags'
            ])

    def log_alert(self, alert):
        if not alert:
            return

        with open(self.current_log_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                alert['timestamp'],
                alert['message_id'],
                alert['latitude'],
                alert['longitude'],
                alert['confidence'],
                alert['fire_detected'],
                alert['gps_valid'],
                alert['status']
            ])

    def get_recent_alerts(self, hours=24):
        alerts = []
        cutoff_time = time.time() - (hours * 3600)

        # Read current log file
        if os.path.exists(self.current_log_file):
            with open(self.current_log_file, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    if float(row['timestamp']) > cutoff_time:
                        alerts.append(row)

        return alerts

    def export_to_csv(self, filename):
        # Combine all log files from today
        today = datetime.now().strftime('%Y%m%d')
        all_alerts = []

        for file in os.listdir(self.log_directory):
            if file.startswith(f'fire_alerts_{today}') and file.endswith('.csv'):
                filepath = os.path.join(self.log_directory, file)
                with open(filepath, 'r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    all_alerts.extend(list(reader))

        # Write combined data
        with open(filename, 'w', newline='') as csvfile:
            if all_alerts:
                writer = csv.DictWriter(csvfile, fieldnames=all_alerts[0].keys())
                writer.writeheader()
                writer.writerows(all_alerts)
```

### Requirements File

```txt
# requirements.txt
tkinter==8.6
folium==0.12.1
pyserial==3.5
pandas==1.5.3
numpy==1.24.1
matplotlib==3.6.3
```

## ⚡ Power Analysis & Flight Time Estimation

### Component Power Consumption

| Component | Current Draw | Voltage | Power (W) | Duty Cycle | Avg Power (W) |
|-----------|--------------|---------|-----------|------------|---------------|
| ESP32-FC | 150mA | 3.3V | 0.495 | 100% | 0.495 |
| ESP32-CAM | 200mA | 3.3V | 0.660 | 100% | 0.660 |
| GPS NEO-6M | 45mA | 3.3V | 0.149 | 100% | 0.149 |
| LoRa SX1278 | 120mA TX | 3.3V | 0.396 | 10% | 0.040 |
| Motors (4x) | 2A each | 11.1V | 88.8 | 70% | 62.16 |
| **Total** | | | **90.5W** | | **63.5W** |

### Battery Specifications
- **Battery**: 3S LiPo 2200mAh 11.1V
- **Energy Capacity**: 2200mAh × 11.1V = 24.42Wh
- **Usable Capacity**: 80% (safety margin) = 19.54Wh
- **Discharge Rate**: 25C maximum

### Flight Time Calculation

#### Method 1: Energy-Based Calculation
```
Available Energy = 19.54Wh
Average Power Draw = 63.5W
Theoretical Flight Time = 19.54Wh ÷ 63.5W = 0.308 hours = 18.5 minutes
```

#### Method 2: Current-Based Calculation
```
Battery Capacity = 2200mAh at 11.1V
Average Current Draw = 63.5W ÷ 11.1V = 5.72A
Theoretical Flight Time = 2200mAh ÷ 5.72A = 384 minutes = 6.4 hours
```
*Wait, this calculation is wrong. Let me correct it:*

```
Average Current Draw = 63.5W ÷ 11.1V = 5.72A
Available Capacity at 11.1V = 2200mAh = 2.2Ah
Flight Time = 2.2Ah ÷ 5.72A = 0.384 hours = 23 minutes
```

#### Method 3: Peukert's Law (More Accurate)
```
Peukert Exponent for LiPo = ~1.3
Effective Capacity = 2200mAh × (5.72A)^(1-1.3) = 2200mAh × (5.72)^(-0.3)
Effective Capacity = 2200mAh × 0.65 = 1430mAh effective
Flight Time = 1430mAh ÷ 5.72A = 0.25 hours = 15 minutes
```

### Realistic Flight Time Estimation
- **Hovering**: 15-18 minutes
- **Forward Flight**: 12-15 minutes
- **With Payload**: 10-12 minutes

### Power Optimization Strategies
1. **Dynamic Voltage Scaling**: Reduce ESP32 clock speed during cruise
2. **GPS Duty Cycling**: Power GPS only when needed
3. **LoRa Optimization**: Reduce transmission frequency
4. **Motor Efficiency**: Use more efficient propellers
5. **Weight Reduction**: Minimize payload weight

### Battery Monitoring Implementation

```cpp
// Battery voltage monitoring
#define BATTERY_PIN 34
#define BATTERY_DIVIDER_RATIO 2.0  // Voltage divider ratio
#define BATTERY_LOW_THRESHOLD 10.5 // 3.5V per cell

float read_battery_voltage() {
    int adc_value = analogRead(BATTERY_PIN);
    float voltage = (adc_value / 4095.0) * 3.3 * BATTERY_DIVIDER_RATIO;
    return voltage;
}

bool is_battery_low() {
    float voltage = read_battery_voltage();
    return voltage < BATTERY_LOW_THRESHOLD;
}
```

## 🧪 Testing Plan

### Unit Testing

#### AI Model Testing
```python
# test_ai_model.py
import pytest
import numpy as np
from fire_detection_model import FireDetectionModel

class TestFireDetectionModel:
    def setup_method(self):
        self.model = FireDetectionModel('models/fire_detection.tflite')

    def test_model_load(self):
        assert self.model.is_loaded()

    def test_fire_detection_high_confidence(self):
        # Load test image with fire
        test_image = load_test_image('test_data/fire_test.jpg')
        result = self.model.predict(test_image)

        assert result['fire_confidence'] > 0.8
        assert result['prediction'] == 'fire'

    def test_no_fire_detection(self):
        # Load test image without fire
        test_image = load_test_image('test_data/no_fire_test.jpg')
        result = self.model.predict(test_image)

        assert result['fire_confidence'] < 0.3
        assert result['prediction'] == 'no_fire'

    def test_inference_time(self):
        test_image = load_test_image('test_data/fire_test.jpg')

        import time
        start_time = time.time()
        result = self.model.predict(test_image)
        inference_time = time.time() - start_time

        assert inference_time < 0.5  # Less than 500ms

    def test_model_memory_usage(self):
        # Test memory consumption during inference
        initial_memory = get_memory_usage()
        result = self.model.predict(load_test_image('test_data/fire_test.jpg'))
        final_memory = get_memory_usage()

        memory_delta = final_memory - initial_memory
        assert memory_delta < 50 * 1024  # Less than 50KB
```

#### LoRa Communication Testing
```python
# test_lora_communication.py
import pytest
from lora_manager import LoRaManager
from alert_protocol import AlertPacket

class TestLoRaCommunication:
    def setup_method(self):
        self.lora = LoRaManager()

    def test_packet_creation(self):
        packet = AlertPacket.create_fire_alert(
            latitude=37.7749,
            longitude=-122.4194,
            confidence=85
        )

        assert packet.header == 0xAA
        assert packet.confidence == 85
        assert abs(packet.latitude - 37.7749) < 0.0001

    def test_packet_serialization(self):
        packet = AlertPacket.create_fire_alert(37.7749, -122.4194, 85)
        serialized = packet.serialize()

        assert len(serialized) == 32
        assert serialized[0] == 0xAA  # Header

    def test_packet_checksum(self):
        packet = AlertPacket.create_fire_alert(37.7749, -122.4194, 85)

        # Test valid checksum
        assert packet.verify_checksum()

        # Test corrupted packet
        packet.confidence = 99  # Change data
        assert not packet.verify_checksum()

    def test_lora_transmission(self):
        packet = AlertPacket.create_fire_alert(37.7749, -122.4194, 85)

        # Transmit packet
        success = self.lora.transmit_packet(packet)
        assert success

        # Wait for transmission
        time.sleep(0.1)

        # Check transmission status
        assert self.lora.get_transmission_status() == 'success'
```

### Integration Testing

#### Flight Controller Integration Test
```python
# test_flight_integration.py
import pytest
from flight_controller import FlightController
from motor_controller import MotorController

class TestFlightIntegration:
    def setup_method(self):
        self.fc = FlightController()
        self.mc = MotorController()

    def test_motor_startup_sequence(self):
        # Test motors start in correct order
        self.fc.arm_motors()

        # Check motor initialization
        for i in range(4):
            assert self.mc.get_motor_speed(i) == 0

        # Test throttle response
        self.fc.set_throttle(0.5)
        time.sleep(0.1)

        for i in range(4):
            speed = self.mc.get_motor_speed(i)
            assert 0.45 < speed < 0.55

    def test_pid_controller_response(self):
        # Test pitch control
        initial_pitch = self.fc.get_pitch()

        # Apply pitch command
        self.fc.set_pitch_target(initial_pitch + 10)  # 10 degree target

        time.sleep(0.5)  # Allow PID to respond

        current_pitch = self.fc.get_pitch()
        error = abs(current_pitch - (initial_pitch + 10))

        assert error < 2.0  # Within 2 degrees

    def test_sensor_fusion(self):
        # Test IMU and GPS integration
        position = self.fc.get_position()
        attitude = self.fc.get_attitude()

        assert 'latitude' in position
        assert 'longitude' in position
        assert 'altitude' in position

        assert 'roll' in attitude
        assert 'pitch' in attitude
        assert 'yaw' in attitude

    def test_emergency_stop(self):
        # Test emergency stop functionality
        self.fc.set_throttle(0.8)  # High throttle
        time.sleep(0.1)

        # Verify motors are running
        for i in range(4):
            assert self.mc.get_motor_speed(i) > 0.7

        # Trigger emergency stop
        self.fc.emergency_stop()

        # Verify all motors stopped
        time.sleep(0.1)
        for i in range(4):
            assert self.mc.get_motor_speed(i) == 0
```

### Field Testing

#### LoRa Range Testing
```python
# lora_range_test.py
import time
import matplotlib.pyplot as plt
from lora_manager import LoRaManager
from gps_manager import GPSManager

class LoRaRangeTester:
    def __init__(self):
        self.lora = LoRaManager()
        self.gps = GPSManager()
        self.test_results = []

    def run_range_test(self, max_distance=5000, step_distance=100):
        base_position = self.gps.get_position()

        for distance in range(0, max_distance + step_distance, step_distance):
            print(f"Testing at {distance}m...")

            # Move to test position (manual movement required)
            input(f"Move to {distance}m position and press Enter")

            current_position = self.gps.get_position()

            # Send test packet
            test_packet = self.create_test_packet(distance)
            success = self.lora.transmit_packet(test_packet)

            # Record result
            result = {
                'distance': distance,
                'latitude': current_position['latitude'],
                'longitude': current_position['longitude'],
                'transmission_success': success,
                'rssi': self.lora.get_last_rssi() if success else None,
                'snr': self.lora.get_last_snr() if success else None
            }

            self.test_results.append(result)

        self.save_results()
        self.generate_report()

    def create_test_packet(self, distance):
        position = self.gps.get_position()
        return AlertPacket.create_test_packet(
            position['latitude'],
            position['longitude'],
            distance
        )

    def save_results(self):
        with open('lora_range_test_results.csv', 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.test_results[0].keys())
            writer.writeheader()
            writer.writerows(self.test_results)

    def generate_report(self):
        # Calculate statistics
        successful_transmissions = [r for r in self.test_results if r['transmission_success']]
        success_rate = len(successful_transmissions) / len(self.test_results) * 100

        plt.figure(figsize=(12, 8))

        # Plot success rate vs distance
        distances = [r['distance'] for r in self.test_results]
        success_flags = [1 if r['transmission_success'] else 0 for r in self.test_results]

        plt.subplot(2, 2, 1)
        plt.plot(distances, success_flags, 'bo-')
        plt.title('Transmission Success vs Distance')
        plt.xlabel('Distance (m)')
        plt.ylabel('Success (1=Yes, 0=No)')

        # Plot RSSI vs distance
        plt.subplot(2, 2, 2)
        rssi_values = [r['rssi'] for r in successful_transmissions if r['rssi'] is not None]
        rssi_distances = [r['distance'] for r in successful_transmissions if r['rssi'] is not None]
        plt.plot(rssi_distances, rssi_values, 'ro-')
        plt.title('RSSI vs Distance')
        plt.xlabel('Distance (m)')
        plt.ylabel('RSSI (dBm)')

        plt.tight_layout()
        plt.savefig('lora_range_test_report.png')
        plt.show()

        print(f"Range Test Results:")
        print(f"Success Rate: {success_rate:.1f}%")
        print(f"Maximum Reliable Range: {max([r['distance'] for r in successful_transmissions])}m")
```

#### Flight Testing Protocol
1. **Pre-Flight Checks**
   - Battery voltage > 11.5V (fully charged)
   - All sensors calibrated
   - GPS lock acquired
   - LoRa link test successful
   - Motors respond correctly to throttle

2. **Flight Test Sequence**
   - Hover test (1 minute)
   - Altitude hold test (2 minutes)
   - Position hold with GPS (3 minutes)
   - Waypoint navigation test (5 minutes)
   - Fire detection simulation (2 minutes)

3. **Safety Procedures**
   - Test flights in open areas away from people
   - Have emergency stop ready
   - Monitor battery voltage continuously
   - Stay within visual line of sight
   - Have backup landing site identified

### Performance Metrics

#### AI Model Metrics
- Accuracy: >95% on test dataset
- Precision: >90% for fire detection
- Recall: >85% for fire detection
- F1-Score: >87%
- Inference time: <500ms
- Model size: <100KB

#### Flight Performance Metrics
- Hover stability: ±0.5m altitude, ±2° attitude
- Position accuracy: ±3m with GPS
- Maximum speed: >15 m/s
- Flight time: >15 minutes
- Power efficiency: >80% motor efficiency

#### Communication Metrics
- LoRa range: >3km line of sight
- Packet success rate: >95% within range
- Latency: <2 seconds end-to-end
- Battery impact: <5% of total consumption

## 🚀 Future Improvements

### Phase 1: Enhanced AI & Vision (1-3 months)
1. **Thermal Camera Integration**
   - Add MLX90640 thermal camera
   - Multi-spectral fire detection
   - Temperature-based classification
   - Better performance in low light/smoke

2. **Advanced AI Models**
   - Implement object detection (YOLOv5-Tiny)
   - Multi-class classification (fire size, type)
   - Real-time model updates via LoRa
   - Edge learning capabilities

3. **Computer Vision Enhancements**
   - Image stabilization for better detection
   - Multi-frame analysis for confidence
   - Smoke detection algorithms
   - Day/night optimized models

### Phase 2: Swarm Coordination (3-6 months)
1. **Multi-Drone Communication**
   - Mesh networking with LoRa
   - Drone-to-drone coordination
   - Distributed fire mapping
   - Collaborative surveillance patterns

2. **Autonomous Mission Planning**
   - Dynamic waypoint generation
   - Fire perimeter mapping
   - Optimal search patterns
   - Return-to-home algorithms

3. **Ground Station Enhancements**
   - Real-time video streaming
   - Multi-drone monitoring
   - Mission planning interface
   - Historical data analysis

### Phase 3: Advanced Features (6-12 months)
1. **Solar Charging Integration**
   - Solar panels on drone frame
   - MPPT charging controller
   - Extended mission duration
   - Weather-adaptive charging

2. **Environmental Sensors**
   - Air quality monitoring (PM2.5, CO, CO2)
   - Weather station (temperature, humidity, wind)
   - Gas detection sensors
   - Radiation monitoring

3. **Cloud Integration**
   - AWS/Azure IoT integration
   - Real-time data streaming
   - Predictive analytics
   - Global monitoring dashboard

### Technical Improvements
1. **Hardware Upgrades**
   - Raspberry Pi 4 with Coral TPU for better AI
   - Higher capacity LiPo batteries (4S configuration)
   - GPS-RTK for centimeter accuracy
   - 4G/5G cellular backup communication

2. **Software Optimizations**
   - ROS2 integration for better modularity
   - Advanced path planning algorithms
   - Machine learning-based flight optimization
   - Over-the-air firmware updates

3. **Safety & Reliability**
   - Dual-redundant flight controllers
   - Advanced failsafe systems
   - Automated emergency procedures
   - Regulatory compliance (FAA Part 107)

### Research Directions
1. **AI Research**
   - Federated learning across drone swarm
   - Meta-learning for wildfire patterns
   - Computer vision in adverse conditions
   - Predictive fire spread modeling

2. **Communication Research**
   - LoRa mesh networking protocols
   - Satellite communication integration
   - Cognitive radio for dynamic spectrum use
   - Ultra-long-range communication

3. **Energy Research**
   - Advanced battery technologies
   - Energy harvesting from wind/temperature
   - Dynamic power management
   - Fuel cell integration

## 📋 Project Timeline & Milestones

### Month 1: Foundation
- [ ] Hardware procurement and testing
- [ ] Basic drone assembly and maiden flight
- [ ] ESP32 firmware development (flight control)
- [ ] LoRa communication setup

### Month 2: AI Integration
- [ ] Dataset collection and preparation
- [ ] AI model training and optimization
- [ ] ESP32-CAM integration
- [ ] TinyML deployment

### Month 3: System Integration
- [ ] Complete drone system integration
- [ ] Alert system implementation
- [ ] Ground station development
- [ ] Basic testing and validation

### Month 4: Field Testing & Optimization
- [ ] Comprehensive testing (unit, integration, field)
- [ ] Performance optimization
- [ ] Documentation completion
- [ ] Demonstration system

### Budget Estimate
- **Hardware**: $160 (components) + $50 (tools) = $210
- **Software**: $0 (open source)
- **Testing**: $100 (batteries, props replacement)
- **Miscellaneous**: $50 (shipping, enclosures)
- **Total**: ~$410 for complete system

## 🔧 Quick Start Guide

### 1. Hardware Assembly
1. Mount ESP32 on flight controller PCB
2. Connect motors to ESCs, ESCs to flight controller
3. Wire GPS module to UART pins
4. Connect LoRa module to SPI pins
5. Attach ESP32-CAM to flight controller via I2C
6. Connect battery and power distribution

### 2. Software Setup
1. Install Arduino IDE with ESP32 support
2. Install required libraries (TinyGPS, LoRa, PID)
3. Flash flight controller firmware to ESP32
4. Flash AI firmware to ESP32-CAM
5. Install Python dependencies for ground station

### 3. Configuration
1. Calibrate IMU and magnetometer
2. Configure LoRa frequency and parameters
3. Set GPS update rate
4. Train and deploy AI model
5. Configure ground station serial port

### 4. Testing
1. Test individual components
2. Perform hover tests
3. Test LoRa communication
4. Validate AI detection accuracy
5. Conduct field tests

This comprehensive guide provides everything needed to build, deploy, and operate the autonomous wildfire surveillance drone system. The modular design allows for incremental development and testing at each stage.
