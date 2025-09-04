#!/usr/bin/env python3
"""
ESP32-Optimized Wildfire Detection Training
Trains a lightweight model specifically for ESP32 constraints
"""

import os
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from pathlib import Path

def create_esp32_model():
    """Create a lightweight CNN optimized for ESP32"""
    model = keras.Sequential([
        layers.Input(shape=(96, 96, 3)),

        # First conv block - very lightweight
        layers.Conv2D(8, (3, 3), activation='relu', padding='same'),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.2),

        # Second conv block
        layers.Conv2D(16, (3, 3), activation='relu', padding='same'),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.2),

        # Third conv block - smaller
        layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.2),

        # Dense layers - much smaller
        layers.Flatten(),
        layers.Dense(64, activation='relu'),
        layers.Dropout(0.5),
        layers.Dense(2, activation='softmax')
    ])

    return model

def setup_light_augmentation():
    """Light augmentation suitable for small dataset"""
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=15,          # Reduced rotation
        width_shift_range=0.1,      # Reduced shift
        height_shift_range=0.1,     # Reduced shift
        horizontal_flip=True,
        zoom_range=0.1,            # Reduced zoom
        brightness_range=[0.9, 1.1], # Reduced brightness range
        fill_mode='nearest',
        validation_split=0.2
    )

    val_datagen = ImageDataGenerator(
        rescale=1./255,
        validation_split=0.2
    )

    return train_datagen, val_datagen

def train_esp32_model():
    """Train lightweight model for ESP32"""
    print("=" * 60)
    print("ESP32-OPTIMIZED WILDFIRE DETECTION TRAINING")
    print("=" * 60)

    # Setup data generators with correct size
    train_datagen, val_datagen = setup_light_augmentation()

    # Create generators
    train_generator = train_datagen.flow_from_directory(
        'dataset/',
        target_size=(96, 96),      # ESP32 input size
        batch_size=8,              # Smaller batch size
        class_mode='categorical',
        subset='training',
        shuffle=True,
        seed=42
    )

    validation_generator = val_datagen.flow_from_directory(
        'dataset/',
        target_size=(96, 96),      # ESP32 input size
        batch_size=8,              # Smaller batch size
        class_mode='categorical',
        subset='validation',
        shuffle=False,
        seed=42
    )

    print(f"\nTraining samples: {train_generator.samples}")
    print(f"Validation samples: {validation_generator.samples}")
    print(f"Steps per epoch: {len(train_generator)}")
    print(f"Validation steps: {len(validation_generator)}")

    # Create lightweight model
    model = create_esp32_model()

    # Compile with conservative settings
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.001),
        loss='categorical_crossentropy',
        metrics=['accuracy', keras.metrics.Precision(), keras.metrics.Recall()]
    )

    # Callbacks
    callbacks = [
        EarlyStopping(
            monitor='val_loss',
            patience=15,  # More patience for small dataset
            restore_best_weights=True,
            verbose=1
        ),
        ModelCheckpoint(
            'models/esp32_fire_model.h5',
            monitor='val_accuracy',
            save_best_only=True,
            verbose=1
        )
    ]

    # Model summary
    print("\n" + "=" * 40)
    print("MODEL ARCHITECTURE")
    print("=" * 40)
    model.summary()

    # Calculate estimated model size
    total_params = model.count_params()
    estimated_size_mb = (total_params * 4) / (1024 * 1024)  # 4 bytes per float32
    print(f"Estimated model size: {estimated_size_mb:.2f} MB")
    # Train model
    print("\n" + "=" * 40)
    print("STARTING TRAINING")
    print("=" * 40)

    history = model.fit(
        train_generator,
        epochs=50,  # Reasonable epochs for small model
        validation_data=validation_generator,
        callbacks=callbacks,
        verbose=1
    )

    # Save final model
    model.save('models/esp32_final_fire_model.h5')
    print("\n✅ ESP32-optimized model saved as 'models/esp32_final_fire_model.h5'")

    # Evaluate final model
    print("\n" + "=" * 40)
    print("FINAL MODEL EVALUATION")
    print("=" * 40)

    # Get final metrics
    final_accuracy = history.history['val_accuracy'][-1]
    final_loss = history.history['val_loss'][-1]

    print(".1f")
    print(".4f")

    return model, history

if __name__ == "__main__":
    # Create models directory
    os.makedirs('models', exist_ok=True)

    # Train ESP32-optimized model
    model, history = train_esp32_model()

    print("\n🎉 ESP32 training complete!")
    print("📊 Your lightweight model is ready for ESP32")
    print("\nNext step: Convert to TensorFlow Lite")
    print("Run: python model_converter.py models/esp32_final_fire_model.h5 models/model_data.h")
