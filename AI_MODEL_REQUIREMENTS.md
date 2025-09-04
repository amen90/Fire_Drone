# AI Model Requirements & Data Preparation Guide

## Overview

This guide explains what data need to be prepared and how to format it for the ESP32 wildfire detection system.

##  AI Model Specifications

### Model Architecture Requirements
The TensorFlow model must meet these specifications for ESP32 compatibility:

#### Input Requirements
- **Shape**: `(96, 96, 3)` - 96x96 RGB images
- **Data Type**: `uint8` (0-255 pixel values)
- **Format**: RGB channel order
- **Preprocessing**: Model should handle raw RGB values (no built-in preprocessing)

#### Output Requirements
- **Shape**: `(2,)` - Binary classification
- **Classes**: `[fire_probability, no_fire_probability]`
- **Activation**: Softmax
- **Data Type**: `float32`

#### Performance Requirements
- **Model Size**: < 100KB (ESP32 flash limit)
- **Inference Time**: < 500ms per frame
- **Memory Usage**: < 50KB RAM during inference
- **Accuracy**: > 95% on test data

## 📊 Dataset Requirements

### Image Specifications

#### Resolution & Format
```
- Minimum: 96x96 pixels
- Recommended: 224x224 or larger (will be resized)
- Format: JPEG, PNG, or BMP
- Color: RGB (3 channels)
```

#### Class Distribution
```
- Fire images: 1000+ samples
- No-fire images: 1000+ samples
- Balanced dataset: 40-60% fire, 40-60% no-fire
- Validation split: 20-30% of total data
```

#### Image Types Required

##### Fire Class Images:
- **Wildfires**: Forest fires, grass fires, brush fires
- **Campfires**: Controlled fires, bonfires
- **Industrial fires**: Construction fires, equipment fires
- **Urban fires**: Building fires, vehicle fires
- **Various distances**: Close-up (5-10m), medium (20-50m), far (100m+)
- **Different times**: Day, dusk, night (if using thermal)
- **Weather conditions**: Clear, smoky, rainy

##### No-Fire Class Images:
- **Similar environments**: Forests, grasslands, urban areas
- **Similar lighting**: Same times as fire images
- **Similar colors**: Red/orange objects that might be confused
- **Similar shapes**: Smoke-like patterns, bright lights
- **Similar textures**: Flickering patterns, glowing objects

### Dataset Structure

#### Directory Structure
```
dataset/
├── fire/
│   ├── forest_fire_001.jpg
│   ├── forest_fire_002.jpg
│   ├── campfire_001.jpg
│   ├── industrial_fire_001.jpg
│   └── ...
├── no_fire/
│   ├── forest_001.jpg
│   ├── sunset_001.jpg
│   ├── headlights_001.jpg
│   ├── steam_001.jpg
│   └── ...
└── validation/  (optional separate validation set)
    ├── fire/
    └── no_fire/
```

#### Image Naming Convention
```
{class}_{source}_{number:03d}.{extension}

Examples:
- fire_forest_001.jpg
- fire_campfire_023.jpg
- no_fire_forest_045.jpg
- no_fire_sunset_012.jpg
```

## 🛠️ Data Preparation Steps

### Step 1: Data Collection

#### Sources for Fire Images
1. **Public Datasets**:
   - Kaggle: "Forest Fire Dataset", "Wildfire Images"
   - Google Images: Search for "wildfire", "forest fire", "brush fire"
   - Academic datasets: Research papers on fire detection

2. **Self-Captured Images**:
   - Campfires (with permission)
   - Construction fires
   - Controlled burns
   - Training exercises

3. **Video Sources**:
   - YouTube wildfire videos
   - News footage
   - Documentary films

#### Sources for No-Fire Images
1. **Similar Environments**:
   - Forest trails, parks, nature reserves
   - Grasslands, meadows
   - Urban areas with similar backgrounds

2. **Confusing Objects**:
   - Sunsets, sunrises
   - Car headlights, taillights
   - Street lights, neon signs
   - Steam, fog, mist
   - Smoke from non-fire sources

### Step 2: Data Preprocessing

#### Image Resizing Script
```python
import cv2
import os
from pathlib import Path

def preprocess_images(input_dir, output_dir, target_size=(224, 224)):
    """Resize and preprocess images for training"""

    input_path = Path(input_dir)
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp'}

    for image_file in input_path.glob('*'):
        if image_file.suffix.lower() in image_extensions:
            try:
                # Read image
                img = cv2.imread(str(image_file))
                if img is None:
                    continue

                # Resize to target size
                img_resized = cv2.resize(img, target_size, interpolation=cv2.INTER_LINEAR)

                # Save processed image
                output_file = output_path / f"{image_file.stem}_processed{image_file.suffix}"
                cv2.imwrite(str(output_file), img_resized)

                print(f"Processed: {image_file.name} -> {output_file.name}")

            except Exception as e:
                print(f"Error processing {image_file.name}: {e}")

# Usage
preprocess_images("dataset/fire", "dataset_processed/fire", (224, 224))
preprocess_images("dataset/no_fire", "dataset_processed/no_fire", (224, 224))
```

#### Data Augmentation
```python
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator

def create_augmented_data_generator():
    """Create data generator with augmentation"""

    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        zoom_range=0.2,
        brightness_range=[0.8, 1.2],
        fill_mode='nearest'
    )

    return train_datagen

# Usage
datagen = create_augmented_data_generator()
train_generator = datagen.flow_from_directory(
    'dataset_processed/',
    target_size=(224, 224),
    batch_size=32,
    class_mode='categorical'
)
```

### Step 3: Model Training

#### TensorFlow/Keras Model Template
```python
import tensorflow as tf
from tensorflow.keras import layers, models

def create_fire_detection_model(input_shape=(224, 224, 3)):
    """Create CNN model for fire detection"""

    model = models.Sequential([
        # Input layer
        layers.Input(shape=input_shape),

        # Convolutional layers
        layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        layers.Conv2D(64, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        layers.Conv2D(128, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        # Dense layers
        layers.Flatten(),
        layers.Dense(256, activation='relu'),
        layers.BatchNormalization(),
        layers.Dropout(0.5),
        layers.Dense(2, activation='softmax')  # Fire, No-fire
    ])

    return model

# Create and compile model
model = create_fire_detection_model()
model.compile(
    optimizer='adam',
    loss='categorical_crossentropy',
    metrics=['accuracy']
)

# Display model summary
model.summary()
```

#### Training Script
```python
def train_fire_detection_model():
    """Train the fire detection model"""

    # Create data generators
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        validation_split=0.2
    )

    # Training generator
    train_generator = train_datagen.flow_from_directory(
        'dataset_processed/',
        target_size=(224, 224),
        batch_size=32,
        class_mode='categorical',
        subset='training',
        shuffle=True
    )

    # Validation generator
    validation_generator = train_datagen.flow_from_directory(
        'dataset_processed/',
        target_size=(224, 224),
        batch_size=32,
        class_mode='categorical',
        subset='validation',
        shuffle=False
    )

    # Callbacks
    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor='val_loss',
            patience=10,
            restore_best_weights=True
        ),
        tf.keras.callbacks.ModelCheckpoint(
            'best_fire_model.h5',
            monitor='val_accuracy',
            save_best_only=True
        ),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.5,
            patience=5,
            min_lr=1e-6
        )
    ]

    # Train model
    history = model.fit(
        train_generator,
        epochs=50,
        validation_data=validation_generator,
        callbacks=callbacks
    )

    return model, history

# Train the model
model, history = train_fire_detection_model()

# Save the trained model
model.save('fire_detection_model.h5')
```

### Step 4: TensorFlow Lite Conversion

#### Basic TFLite Conversion
```python
import tensorflow as tf

def convert_to_tflite(model_path, output_path):
    """Convert Keras model to TensorFlow Lite"""

    # Load trained model
    model = tf.keras.models.load_model(model_path)

    # Create converter
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    # Optimize for size and speed
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    # Convert model
    tflite_model = converter.convert()

    # Save TFLite model
    with open(output_path, 'wb') as f:
        f.write(tflite_model)

    print(f"TFLite model saved: {output_path}")
    print(f"Model size: {len(tflite_model)} bytes")

    return tflite_model

# Convert model
tflite_model = convert_to_tflite('fire_detection_model.h5', 'fire_detection_model.tflite')
```

#### Quantized TFLite Conversion (Recommended for ESP32)
```python
def convert_to_quantized_tflite(model_path, output_path, validation_data=None):
    """Convert to quantized TensorFlow Lite model"""

    # Load model
    model = tf.keras.models.load_model(model_path)

    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    # Enable quantization
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    # Set quantization parameters
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8

    # Representative dataset for quantization
    def representative_dataset_gen():
        # Use a subset of your training data
        for image_batch, _ in validation_data.take(100):
            # Convert to int8 range
            image_batch = image_batch * 255.0  # Assuming input is 0-1
            yield [image_batch]

    if validation_data:
        converter.representative_dataset = representative_dataset_gen

    # Convert
    tflite_quantized = converter.convert()

    # Save
    with open(output_path, 'wb') as f:
        f.write(tflite_quantized)

    print(f"Quantized TFLite model saved: {output_path}")
    print(f"Model size: {len(tflite_quantized)} bytes")

    return tflite_quantized
```

### Step 5: ESP32 Integration

#### Convert TFLite to C Header
```python
# Use the provided converter script
python model_converter.py fire_detection_model.tflite model_data.h
```

#### Update ESP32-CAM Firmware
1. **Replace the model data** in `esp32_cam_ai.ino`:
   ```cpp
   // Remove the placeholder model_data.h include
   // Add your generated model_data.h
   #include "model_data.h"  // Your actual model
   ```

2. **Adjust tensor arena size** based on your model:
   ```cpp
   // Check model size and adjust accordingly
   constexpr int kTensorArenaSize = 64 * 1024;  // Increase if needed
   ```

3. **Update model operations resolver**:
   ```cpp
   // Add any additional operations your model uses
   resolver.AddBuiltin(tflite::BuiltinOperator_AVERAGE_POOL_2D,
                      tflite::ops::micro::Register_AVERAGE_POOL_2D());
   ```

## 📋 Checklist for Model Preparation

### Data Collection ✅
- [ ] Collected 1000+ fire images from various sources
- [ ] Collected 1000+ no-fire images from similar environments
- [ ] Images are 224x224 or larger resolution
- [ ] Dataset is balanced (40-60% fire, 40-60% no-fire)

### Data Preprocessing ✅
- [ ] Images resized to consistent dimensions
- [ ] Data augmentation applied (rotation, flip, brightness)
- [ ] Dataset split into train/validation/test sets
- [ ] Images converted to RGB format

### Model Development ✅
- [ ] CNN architecture designed for 96x96 input
- [ ] Binary classification (fire/no-fire)
- [ ] Model trained to >95% accuracy
- [ ] Model size < 100KB
- [ ] Inference time < 500ms

### TensorFlow Lite Conversion ✅
- [ ] Model converted to .tflite format
- [ ] Quantization applied (int8 preferred)
- [ ] Model tested on desktop with TFLite interpreter
- [ ] Input/output specifications verified

### ESP32 Integration ✅
- [ ] TFLite model converted to C header file
- [ ] Model data included in ESP32-CAM firmware
- [ ] Tensor arena size adjusted for model
- [ ] Model operations resolver updated
- [ ] Firmware uploaded and tested

## 🔧 Troubleshooting

### Common Issues

#### 1. Model Too Large
```
Problem: Model exceeds ESP32 flash memory
Solution:
- Reduce model complexity (fewer layers/filters)
- Use quantization (int8 instead of float32)
- Prune model weights
- Use knowledge distillation
```

#### 2. Slow Inference
```
Problem: Inference takes >500ms
Solution:
- Reduce input size (96x96 instead of 224x224)
- Use depthwise separable convolutions
- Apply quantization
- Optimize model architecture
```

#### 3. Memory Issues
```
Problem: ESP32 runs out of RAM during inference
Solution:
- Increase tensor arena size
- Use static memory allocation
- Reduce batch size to 1
- Optimize model for memory efficiency
```

#### 4. Poor Accuracy
```
Problem: Model accuracy <90%
Solution:
- Increase dataset size
- Improve data quality
- Use data augmentation
- Try different architectures
- Use transfer learning
```

##  Getting Help

If you encounter issues with model preparation:

1. **Check the requirements**: Ensure your model meets all specifications
2. **Test on desktop first**: Verify model works with TFLite desktop
3. **Start simple**: Begin with a basic CNN before complex architectures
4. **Use quantization**: Always quantize for ESP32 performance
5. **Monitor resources**: Keep track of model size and inference time

##  Additional Resources

- **TensorFlow Lite Micro**: https://www.tensorflow.org/lite/microcontrollers
- **ESP32 Camera**: https://github.com/espressif/esp32-camera
- **Fire Detection Datasets**: Search Kaggle for "fire detection"
- **Model Optimization**: https://www.tensorflow.org/lite/performance/model_optimization

---

**Ready to start?** Follow the checklist above and use the provided scripts to prepare your AI model for the wildfire surveillance drone system.

