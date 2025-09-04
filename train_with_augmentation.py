#!/usr/bin/env python3
"""
Wildfire Detection Training with Data Augmentation
Trains AI model using your 96 images with heavy augmentation
"""

import os
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from pathlib import Path
import matplotlib.pyplot as plt

def create_model():
    """Create CNN model optimized for wildfire detection"""
    model = keras.Sequential([
        layers.Input(shape=(224, 224, 3)),

        # First conv block
        layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        # Second conv block
        layers.Conv2D(64, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        # Third conv block
        layers.Conv2D(128, (3, 3), activation='relu', padding='same'),
        layers.BatchNormalization(),
        layers.MaxPooling2D((2, 2)),
        layers.Dropout(0.25),

        # Dense layers
        layers.Flatten(),
        layers.Dense(256, activation='relu'),
        layers.BatchNormalization(),
        layers.Dropout(0.5),
        layers.Dense(2, activation='softmax')
    ])

    return model

def setup_data_augmentation():
    """Create aggressive data augmentation for small dataset"""
    train_datagen = ImageDataGenerator(
        rescale=1./255,
        rotation_range=40,          # Increased rotation
        width_shift_range=0.3,      # Increased shift
        height_shift_range=0.3,     # Increased shift
        shear_range=0.2,           # Added shear
        zoom_range=0.3,            # Increased zoom
        horizontal_flip=True,
        vertical_flip=False,       # No vertical flip for fire detection
        brightness_range=[0.7, 1.3], # Wider brightness range
        channel_shift_range=0.2,   # Color shift
        fill_mode='nearest',
        validation_split=0.2
    )

    val_datagen = ImageDataGenerator(
        rescale=1./255,
        validation_split=0.2
    )

    return train_datagen, val_datagen

def train_with_augmentation():
    """Train model with heavy data augmentation"""
    print("=" * 60)
    print("WILDFIRE DETECTION TRAINING WITH AUGMENTATION")
    print("=" * 60)

    # Setup data generators
    train_datagen, val_datagen = setup_data_augmentation()

    # Create generators
    train_generator = train_datagen.flow_from_directory(
        'dataset/',
        target_size=(224, 224),
        batch_size=16,  # Small batch size for small dataset
        class_mode='categorical',
        subset='training',
        shuffle=True,
        seed=42
    )

    validation_generator = val_datagen.flow_from_directory(
        'dataset/',
        target_size=(224, 224),
        batch_size=16,
        class_mode='categorical',
        subset='validation',
        shuffle=False,
        seed=42
    )

    print(f"\nTraining samples: {train_generator.samples}")
    print(f"Validation samples: {validation_generator.samples}")
    print(f"Steps per epoch: {len(train_generator)}")
    print(f"Validation steps: {len(validation_generator)}")

    # Create model
    model = create_model()

    # Compile model
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=0.0001),  # Lower learning rate
        loss='categorical_crossentropy',
        metrics=['accuracy', keras.metrics.Precision(), keras.metrics.Recall()]
    )

    # Callbacks
    callbacks = [
        EarlyStopping(
            monitor='val_loss',
            patience=20,  # More patience for small dataset
            restore_best_weights=True,
            verbose=1
        ),
        ModelCheckpoint(
            'models/best_fire_model.h5',
            monitor='val_accuracy',
            save_best_only=True,
            verbose=1
        )
    ]

    # Calculate effective dataset size with augmentation
    effective_size = train_generator.samples * 10  # Each image becomes ~10 variations
    print(f"\nEffective training size with augmentation: ~{effective_size} samples")

    # Train model
    print("\n" + "=" * 40)
    print("STARTING TRAINING")
    print("=" * 40)

    history = model.fit(
        train_generator,
        epochs=100,  # More epochs for small dataset
        validation_data=validation_generator,
        callbacks=callbacks,
        verbose=1
    )

    # Save final model
    model.save('models/final_fire_model.h5')
    print("\n✅ Model saved as 'models/final_fire_model.h5'")

    # Plot training history
    plt.figure(figsize=(12, 4))

    plt.subplot(1, 3, 1)
    plt.plot(history.history['accuracy'], label='Training')
    plt.plot(history.history['val_accuracy'], label='Validation')
    plt.title('Model Accuracy')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 3, 2)
    plt.plot(history.history['loss'], label='Training')
    plt.plot(history.history['val_loss'], label='Validation')
    plt.title('Model Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(1, 3, 3)
    plt.plot(history.history['precision'], label='Training Precision')
    plt.plot(history.history['recall'], label='Training Recall')
    plt.plot(history.history['val_precision'], label='Val Precision')
    plt.plot(history.history['val_recall'], label='Val Recall')
    plt.title('Precision & Recall')
    plt.xlabel('Epoch')
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('models/training_history.png', dpi=300, bbox_inches='tight')
    plt.show()

    # Evaluate final model
    print("\n" + "=" * 40)
    print("FINAL MODEL EVALUATION")
    print("=" * 40)

    # Get final metrics
    final_accuracy = history.history['val_accuracy'][-1]
    final_loss = history.history['val_loss'][-1]

    print(f"Final validation accuracy: {final_accuracy:.1f}")
    print(f"Final validation loss: {final_loss:.4f}")
    # Model summary
    print("\nModel Architecture:")
    model.summary()

    return model, history

if __name__ == "__main__":
    # Create models directory
    os.makedirs('models', exist_ok=True)

    # Train model
    model, history = train_with_augmentation()

    print("\n🎉 Training complete!")
    print("📊 Check 'models/training_history.png' for training plots")
    print("🤖 Your model is saved as 'models/final_fire_model.h5'")
    print("\nNext step: Convert to TensorFlow Lite for ESP32")
    print("Run: python model_converter.py models/final_fire_model.h5 models/model_data.h")
