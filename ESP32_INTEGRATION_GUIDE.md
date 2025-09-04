# 📋 ESP32 Firmware Integration Guide

### Firmware Files
1. **`drone_flight_controller.ino`** - Flight controller for ESP32-DevKitC
2. **`esp32_cam_ai.ino`** - AI camera firmware for ESP32-CAM

### Tools & Scripts
3. **`model_converter.py`** - Converts your .tflite model to ESP32 C array
4. **`AI_MODEL_REQUIREMENTS.md`** - Complete guide for preparing your AI model
5. **`QUICK_START.md`** - Step-by-step setup instructions

## What need to be provided

### 1. AI Model Data
create one file: **`model_data.h`**

This file contains the trained AI model as a C array that the ESP32 can use.

#### How to Create model_data.h:
```bash
# Step 1: Train your model and save as .tflite
# (Use ai_pipeline.py or your own training script)

# Step 2: Convert to ESP32 format
python model_converter.py your_model.tflite model_data.h

# Step 3: Copy model_data.h to your Arduino project folder
```

### 2. Dataset Structure
The training data should be organized as:
```
dataset/
├── fire/          # Your fire images (1000+)
│   ├── fire001.jpg
│   ├── fire002.jpg
│   └── ...
└── no_fire/       # Your non-fire images (1000+)
    ├── forest001.jpg
    ├── sunset001.jpg
    └── ...
```

## Arduino IDE Setup

### Required Libraries
Install these in Arduino IDE (Sketch → Include Library → Manage Libraries):

1. **ESP32Servo** - For motor control
2. **TinyGPSPlus** - For GPS parsing
3. **LoRa** by Sandeep Mistry - For LoRa communication
4. **TensorFlow Lite Micro** - Download from GitHub and install manually

### Board Setup
1. **Flight Controller**: Select "ESP32 Dev Module"
2. **AI Camera**: Select "AI-Thinker ESP32-CAM"

## Model Specifications (Critical!)

The AI model must have these exact specifications:

### Input:
- **Shape**: `(96, 96, 3)` - 96x96 RGB image
- **Data Type**: `uint8` (0-255 pixel values)
- **Format**: RGB channels (not BGR)

### Output:
- **Shape**: `(2,)` - Two probabilities
- **Format**: `[fire_probability, no_fire_probability]`
- **Activation**: Softmax

### Performance:
- **Model Size**: < 100KB (ESP32 flash limit)
- **Inference Time**: < 500ms per frame
- **Accuracy**: > 95% on your test data

## Quick Test Commands

### Flight Controller Serial Commands:
```
ARM         # Arm motors
DISARM      # Disarm motors
THROTTLE 0.5  # Set throttle (0.0-1.0)
STATUS      # Show system status
GPS         # Show GPS data
```

### AI Camera Serial Commands:
```
STATUS      # Show system status
CAPTURE     # Force image capture
HEAP        # Show memory usage
RESTART     # Restart ESP32
```

## Integration Steps

### Step 1: Upload Firmware
1. **Flight Controller**: Upload `drone_flight_controller.ino`
2. **AI Camera**: Upload `esp32_cam_ai.ino` (GPIO 0 to GND during upload!)

### Step 2: Test Individual Components
1. Test GPS position accuracy
2. Test LoRa communication
3. Test AI fire detection
4. Test motor throttle response

### Step 3: Integration Testing
1. Test camera-to-AI pipeline
2. Test AI-to-flight-controller communication
3. Test GPS and LoRa coordination
4. Test full fire detection workflow

## Hardware Wiring (Reference)

### ESP32 Flight Controller Pinout:
```
Motor Control:
GPIO 12: Motor 1 (Front Left)
GPIO 13: Motor 2 (Front Right)
GPIO 14: Motor 3 (Rear Right)
GPIO 15: Motor 4 (Rear Left)

Communication:
GPIO 16/17: GPS (UART)
GPIO 18-23: LoRa (SPI)
GPIO 21/22: I2C (to ESP32-CAM)

Power:
GPIO 34: Battery voltage (ADC)
GPIO 25: Status LED
```

### ESP32-CAM Pinout:
```
Camera pins are pre-configured
I2C: GPIO 12/13 (to Flight Controller)
```

## Troubleshooting

### Common Issues:

1. **"Model too large"**
   - Solution: Use quantization, reduce model complexity

2. **"Out of memory"**
   - Solution: Increase tensor arena size, use smaller model

3. **"GPS no fix"**
   - Solution: Ensure antenna has sky view, wait 5-10 minutes

4. **"LoRa no communication"**
   - Solution: Check antennas, verify frequency, test range

5. **"AI not detecting fire"**
   - Solution: Verify model input format, test on desktop first

## Performance Expectations

### Flight Controller:
- **CPU Usage**: < 50% during normal flight
- **Memory Usage**: < 100KB RAM
- **Response Time**: < 10ms for motor commands

### AI Camera:
- **Inference Time**: < 500ms per frame
- **Memory Usage**: < 200KB RAM
- **Detection Rate**: 2 frames per second

### Communication:
- **LoRa Range**: 5-10km line-of-sight
- **Latency**: < 100ms for alerts
- **Reliability**: > 95% success rate

## Next Steps

1. **Prepare your dataset** (see AI_MODEL_REQUIREMENTS.md)
2. **Train your AI model** (use ai_pipeline.py)
3. **Convert model** (use model_converter.py)
4. **Upload firmware** (follow QUICK_START.md)
5. **Test components** (use serial commands)
6. **Field testing** (gradually increase complexity)

## Support

If there is issues:
1. Check serial output for error messages
2. Verify model meets specifications
3. Test with simple examples first
4. Ensure proper power and wiring
5. Check memory and flash usage

Remember: Start with individual component testing, then integrate step by step!

