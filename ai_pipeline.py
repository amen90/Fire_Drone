#!/usr/bin/env python3
"""
Wildfire Detection AI Pipeline
Complete pipeline for training and deploying fire detection model on ESP32
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from tensorflow.keras.applications import MobileNetV2
from tensorflow.keras.optimizers import Adam
import cv2
import glob
from PIL import Image
import time
from pathlib import Path
import argparse
import json
from datetime import datetime

# Configuration
class Config:
    # Dataset paths
    DATASET_PATH = "dataset"
    FIRE_DATASET_PATH = os.path.join(DATASET_PATH, "fire")
    NO_FIRE_DATASET_PATH = os.path.join(DATASET_PATH, "no_fire")

    # Model parameters
    IMG_HEIGHT = 96
    IMG_WIDTH = 96
    BATCH_SIZE = 32
    EPOCHS = 50
    LEARNING_RATE = 0.001

    # Model paths
    MODEL_SAVE_PATH = "models"
    TFLITE_MODEL_PATH = os.path.join(MODEL_SAVE_PATH, "fire_detection.tflite")
    SAVED_MODEL_PATH = os.path.join(MODEL_SAVE_PATH, "saved_model")

    # Training parameters
    VALIDATION_SPLIT = 0.2
    TEST_SPLIT = 0.1
    RANDOM_SEED = 42

    # Data augmentation parameters
    ROTATION_RANGE = 20
    WIDTH_SHIFT_RANGE = 0.2
    HEIGHT_SHIFT_RANGE = 0.2
    HORIZONTAL_FLIP = True
    ZOOM_RANGE = 0.2
    BRIGHTNESS_RANGE = [0.8, 1.2]

class DataManager:
    """Handles dataset loading, preprocessing, and augmentation"""

    def __init__(self, config):
        self.config = config
        self.ensure_directories()

    def ensure_directories(self):
        """Create necessary directories"""
        os.makedirs(self.config.DATASET_PATH, exist_ok=True)
        os.makedirs(self.config.FIRE_DATASET_PATH, exist_ok=True)
        os.makedirs(self.config.NO_FIRE_DATASET_PATH, exist_ok=True)
        os.makedirs(self.config.MODEL_SAVE_PATH, exist_ok=True)

    def load_dataset_info(self):
        """Load dataset statistics"""
        fire_images = glob.glob(os.path.join(self.config.FIRE_DATASET_PATH, "*.jpg")) + \
                     glob.glob(os.path.join(self.config.FIRE_DATASET_PATH, "*.png"))

        no_fire_images = glob.glob(os.path.join(self.config.NO_FIRE_DATASET_PATH, "*.jpg")) + \
                        glob.glob(os.path.join(self.config.NO_FIRE_DATASET_PATH, "*.png"))

        print(f"Fire images: {len(fire_images)}")
        print(f"No-fire images: {len(no_fire_images)}")
        print(f"Total images: {len(fire_images) + len(no_fire_images)}")

        return fire_images, no_fire_images

    def create_data_generators(self):
        """Create training and validation data generators"""

        # Training data generator with augmentation
        train_datagen = ImageDataGenerator(
            rescale=1./255,
            rotation_range=self.config.ROTATION_RANGE,
            width_shift_range=self.config.WIDTH_SHIFT_RANGE,
            height_shift_range=self.config.HEIGHT_SHIFT_RANGE,
            horizontal_flip=self.config.HORIZONTAL_FLIP,
            zoom_range=self.config.ZOOM_RANGE,
            brightness_range=self.config.BRIGHTNESS_RANGE,
            validation_split=self.config.VALIDATION_SPLIT
        )

        # Validation data generator (no augmentation)
        val_datagen = ImageDataGenerator(
            rescale=1./255,
            validation_split=self.config.VALIDATION_SPLIT
        )

        # Training generator
        train_generator = train_datagen.flow_from_directory(
            self.config.DATASET_PATH,
            target_size=(self.config.IMG_HEIGHT, self.config.IMG_WIDTH),
            batch_size=self.config.BATCH_SIZE,
            class_mode='categorical',
            subset='training',
            shuffle=True,
            seed=self.config.RANDOM_SEED
        )

        # Validation generator
        validation_generator = val_datagen.flow_from_directory(
            self.config.DATASET_PATH,
            target_size=(self.config.IMG_HEIGHT, self.config.IMG_WIDTH),
            batch_size=self.config.BATCH_SIZE,
            class_mode='categorical',
            subset='validation',
            shuffle=False,
            seed=self.config.RANDOM_SEED
        )

        return train_generator, validation_generator

    def analyze_dataset(self):
        """Analyze dataset characteristics"""
        fire_images, no_fire_images = self.load_dataset_info()

        # Analyze image sizes
        fire_sizes = []
        no_fire_sizes = []

        for img_path in fire_images[:100]:  # Sample first 100
            try:
                img = Image.open(img_path)
                fire_sizes.append(img.size)
            except:
                continue

        for img_path in no_fire_images[:100]:  # Sample first 100
            try:
                img = Image.open(img_path)
                no_fire_sizes.append(img.size)
            except:
                continue

        print("
Dataset Analysis:")
        print(f"Average fire image size: {np.mean(fire_sizes, axis=0) if fire_sizes else 'N/A'}")
        print(f"Average no-fire image size: {np.mean(no_fire_sizes, axis=0) if no_fire_sizes else 'N/A'}")

        return {
            'fire_count': len(fire_images),
            'no_fire_count': len(no_fire_images),
            'fire_sizes': fire_sizes,
            'no_fire_sizes': no_fire_sizes
        }

class ModelBuilder:
    """Handles model creation, training, and evaluation"""

    def __init__(self, config):
        self.config = config

    def create_model(self, model_type='cnn'):
        """Create the fire detection model"""

        if model_type == 'cnn':
            return self._create_cnn_model()
        elif model_type == 'mobilenet':
            return self._create_mobilenet_model()
        else:
            raise ValueError(f"Unknown model type: {model_type}")

    def _create_cnn_model(self):
        """Create a custom CNN model optimized for TinyML"""

        model = keras.Sequential([
            layers.Input(shape=(self.config.IMG_HEIGHT, self.config.IMG_WIDTH, 3)),

            # First convolutional block
            layers.Conv2D(16, (3, 3), activation='relu', padding='same'),
            layers.BatchNormalization(),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            # Second convolutional block
            layers.Conv2D(32, (3, 3), activation='relu', padding='same'),
            layers.BatchNormalization(),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            # Third convolutional block
            layers.Conv2D(64, (3, 3), activation='relu', padding='same'),
            layers.BatchNormalization(),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            # Dense layers
            layers.Flatten(),
            layers.Dense(128, activation='relu'),
            layers.BatchNormalization(),
            layers.Dropout(0.5),

            # Output layer
            layers.Dense(2, activation='softmax')  # Fire, No-Fire
        ])

        return model

    def _create_mobilenet_model(self):
        """Create MobileNetV2-based model"""

        # Load MobileNetV2 without top layers
        base_model = MobileNetV2(
            weights='imagenet',
            include_top=False,
            input_shape=(self.config.IMG_HEIGHT, self.config.IMG_WIDTH, 3)
        )

        # Freeze base model layers
        base_model.trainable = False

        model = keras.Sequential([
            base_model,
            layers.GlobalAveragePooling2D(),
            layers.Dense(128, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(2, activation='softmax')
        ])

        return model

    def compile_model(self, model):
        """Compile the model with optimizer and loss"""

        optimizer = Adam(learning_rate=self.config.LEARNING_RATE)

        model.compile(
            optimizer=optimizer,
            loss='categorical_crossentropy',
            metrics=['accuracy', keras.metrics.Precision(), keras.metrics.Recall()]
        )

        return model

    def get_callbacks(self):
        """Get training callbacks"""

        callbacks = []

        # Early stopping
        early_stopping = EarlyStopping(
            monitor='val_loss',
            patience=10,
            restore_best_weights=True,
            verbose=1
        )
        callbacks.append(early_stopping)

        # Model checkpoint
        model_checkpoint = ModelCheckpoint(
            os.path.join(self.config.MODEL_SAVE_PATH, 'best_model.h5'),
            monitor='val_accuracy',
            save_best_only=True,
            verbose=1
        )
        callbacks.append(model_checkpoint)

        # Learning rate reduction
        reduce_lr = ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.5,
            patience=5,
            min_lr=1e-6,
            verbose=1
        )
        callbacks.append(reduce_lr)

        return callbacks

    def train_model(self, model, train_generator, validation_generator):
        """Train the model"""

        print("Starting model training...")

        callbacks = self.get_callbacks()

        history = model.fit(
            train_generator,
            epochs=self.config.EPOCHS,
            validation_data=validation_generator,
            callbacks=callbacks,
            verbose=1
        )

        return model, history

    def evaluate_model(self, model, validation_generator):
        """Evaluate model performance"""

        print("Evaluating model...")

        # Get predictions
        predictions = model.predict(validation_generator)
        y_pred = np.argmax(predictions, axis=1)
        y_true = validation_generator.classes

        # Classification report
        class_names = list(validation_generator.class_indices.keys())
        report = classification_report(y_true, y_pred, target_names=class_names)
        print("\nClassification Report:")
        print(report)

        # Confusion matrix
        cm = confusion_matrix(y_true, y_pred)

        return {
            'predictions': predictions,
            'y_pred': y_pred,
            'y_true': y_true,
            'classification_report': report,
            'confusion_matrix': cm,
            'class_names': class_names
        }

class ModelConverter:
    """Handles TensorFlow Lite conversion and optimization"""

    def __init__(self, config):
        self.config = config

    def convert_to_tflite(self, model, quantization='dynamic'):
        """Convert Keras model to TensorFlow Lite"""

        print(f"Converting model to TensorFlow Lite ({quantization} quantization)...")

        converter = tf.lite.TFLiteConverter.from_keras_model(model)

        if quantization == 'dynamic':
            # Dynamic range quantization
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
        elif quantization == 'full_int8':
            # Full integer quantization
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            converter.target_spec.supported_ops = [
                tf.lite.OpsSet.TFLITE_BUILTINS_INT8
            ]
            converter.inference_input_type = tf.int8
            converter.inference_output_type = tf.int8

            # Representative dataset for quantization
            def representative_dataset_gen():
                # Load a few sample images for quantization calibration
                for img_path in glob.glob(os.path.join(self.config.DATASET_PATH, "**/*.jpg"))[:100]:
                    img = tf.io.read_file(img_path)
                    img = tf.image.decode_jpeg(img, channels=3)
                    img = tf.image.resize(img, [self.config.IMG_HEIGHT, self.config.IMG_WIDTH])
                    img = tf.cast(img, tf.float32) / 255.0
                    img = tf.expand_dims(img, 0)
                    yield [img]

            converter.representative_dataset = representative_dataset_gen
        elif quantization == 'float16':
            # Float16 quantization
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            converter.target_spec.supported_types = [tf.float16]

        # Convert model
        tflite_model = converter.convert()

        # Save model
        with open(self.config.TFLITE_MODEL_PATH, 'wb') as f:
            f.write(tflite_model)

        # Get model size
        model_size = len(tflite_model)
        print(".2f")

        return tflite_model, model_size

    def convert_to_c_array(self, tflite_model, array_name='model_data'):
        """Convert TFLite model to C array for embedding"""

        print("Converting to C array...")

        # Convert to hex array
        hex_array = [f'0x{byte:02x}' for byte in tflite_model]

        # Create C file content
        c_content = f'''// Auto-generated TensorFlow Lite model data
// Generated on {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
// Model size: {len(tflite_model)} bytes

#ifndef MODEL_DATA_H_
#define MODEL_DATA_H_

#include <cstdint>

const unsigned int {array_name}_len = {len(tflite_model)};
const unsigned char {array_name}[] = {{
    {', '.join(hex_array[:16])}'''

        # Add remaining bytes in chunks
        for i in range(16, len(hex_array), 16):
            chunk = ', '.join(hex_array[i:i+16])
            c_content += f''',
    {chunk}'''

        c_content += f'''
}};

#endif  // MODEL_DATA_H_
'''

        # Save C file
        c_file_path = os.path.join(self.config.MODEL_SAVE_PATH, 'model_data.h')
        with open(c_file_path, 'w') as f:
            f.write(c_content)

        print(f"C array saved to: {c_file_path}")
        return c_file_path

    def analyze_model(self, tflite_model):
        """Analyze the converted model"""

        print("Analyzing TensorFlow Lite model...")

        # Load TFLite model
        interpreter = tf.lite.Interpreter(model_content=tflite_model)
        interpreter.allocate_tensors()

        # Get input/output details
        input_details = interpreter.get_input_details()[0]
        output_details = interpreter.get_output_details()[0]

        print(f"Input shape: {input_details['shape']}")
        print(f"Input type: {input_details['dtype']}")
        print(f"Output shape: {output_details['shape']}")
        print(f"Output type: {output_details['dtype']}")

        return {
            'input_details': input_details,
            'output_details': output_details,
            'input_shape': input_details['shape'],
            'output_shape': output_details['shape']
        }

class ModelEvaluator:
    """Handles model evaluation and visualization"""

    def __init__(self, config):
        self.config = config

    def plot_training_history(self, history):
        """Plot training history"""

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

        # Accuracy
        ax1.plot(history.history['accuracy'], label='Training')
        ax1.plot(history.history['val_accuracy'], label='Validation')
        ax1.set_title('Model Accuracy')
        ax1.set_xlabel('Epoch')
        ax1.set_ylabel('Accuracy')
        ax1.legend()
        ax1.grid(True)

        # Loss
        ax2.plot(history.history['loss'], label='Training')
        ax2.plot(history.history['val_loss'], label='Validation')
        ax2.set_title('Model Loss')
        ax2.set_xlabel('Epoch')
        ax2.set_ylabel('Loss')
        ax2.legend()
        ax2.grid(True)

        # Precision
        if 'precision' in history.history:
            ax3.plot(history.history['precision'], label='Training')
            ax3.plot(history.history['val_precision'], label='Validation')
            ax3.set_title('Model Precision')
            ax3.set_xlabel('Epoch')
            ax3.set_ylabel('Precision')
            ax3.legend()
            ax3.grid(True)

        # Recall
        if 'recall' in history.history:
            ax4.plot(history.history['recall'], label='Training')
            ax4.plot(history.history['val_recall'], label='Validation')
            ax4.set_title('Model Recall')
            ax4.set_xlabel('Epoch')
            ax4.set_ylabel('Recall')
            ax4.legend()
            ax4.grid(True)

        plt.tight_layout()
        plt.savefig(os.path.join(self.config.MODEL_SAVE_PATH, 'training_history.png'), dpi=300, bbox_inches='tight')
        plt.show()

    def plot_confusion_matrix(self, cm, class_names):
        """Plot confusion matrix"""

        plt.figure(figsize=(8, 6))
        sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                   xticklabels=class_names, yticklabels=class_names)
        plt.title('Confusion Matrix')
        plt.xlabel('Predicted')
        plt.ylabel('True')
        plt.tight_layout()
        plt.savefig(os.path.join(self.config.MODEL_SAVE_PATH, 'confusion_matrix.png'), dpi=300, bbox_inches='tight')
        plt.show()

    def test_inference_speed(self, model, test_images=100):
        """Test model inference speed"""

        print(f"Testing inference speed on {test_images} images...")

        # Get sample images
        image_files = []
        for class_dir in [self.config.FIRE_DATASET_PATH, self.config.NO_FIRE_DATASET_PATH]:
            image_files.extend(glob.glob(os.path.join(class_dir, "*.jpg"))[:test_images//2])

        inference_times = []

        for img_path in image_files[:test_images]:
            # Load and preprocess image
            img = tf.keras.preprocessing.image.load_img(
                img_path, target_size=(self.config.IMG_HEIGHT, self.config.IMG_WIDTH)
            )
            img_array = tf.keras.preprocessing.image.img_to_array(img)
            img_array = tf.expand_dims(img_array, 0)
            img_array = img_array / 255.0

            # Measure inference time
            start_time = time.time()
            predictions = model.predict(img_array, verbose=0)
            inference_time = (time.time() - start_time) * 1000  # Convert to ms

            inference_times.append(inference_time)

        avg_time = np.mean(inference_times)
        min_time = np.min(inference_times)
        max_time = np.max(inference_times)

        print(".2f")
        print(".2f")
        print(".2f")

        return {
            'avg_inference_time': avg_time,
            'min_inference_time': min_time,
            'max_inference_time': max_time,
            'inference_times': inference_times
        }

class Pipeline:
    """Main AI pipeline orchestrator"""

    def __init__(self):
        self.config = Config()
        self.data_manager = DataManager(self.config)
        self.model_builder = ModelBuilder(self.config)
        self.model_converter = ModelConverter(self.config)
        self.model_evaluator = ModelEvaluator(self.config)

    def run_full_pipeline(self, model_type='cnn', quantization='dynamic'):
        """Run the complete AI pipeline"""

        print("=" * 60)
        print("Wildfire Detection AI Pipeline")
        print("=" * 60)

        # Step 1: Analyze dataset
        print("\n1. Analyzing dataset...")
        dataset_info = self.data_manager.analyze_dataset()

        # Step 2: Create data generators
        print("\n2. Creating data generators...")
        train_generator, validation_generator = self.data_manager.create_data_generators()

        # Step 3: Create and compile model
        print("\n3. Creating model...")
        model = self.model_builder.create_model(model_type)
        model = self.model_builder.compile_model(model)

        print("Model Summary:")
        model.summary()

        # Step 4: Train model
        print("\n4. Training model...")
        trained_model, history = self.model_builder.train_model(model, train_generator, validation_generator)

        # Step 5: Evaluate model
        print("\n5. Evaluating model...")
        evaluation_results = self.model_builder.evaluate_model(trained_model, validation_generator)

        # Step 6: Test inference speed
        print("\n6. Testing inference speed...")
        speed_results = self.model_evaluator.test_inference_speed(trained_model)

        # Step 7: Convert to TensorFlow Lite
        print("\n7. Converting to TensorFlow Lite...")
        tflite_model, model_size = self.model_converter.convert_to_tflite(trained_model, quantization)

        # Step 8: Convert to C array
        print("\n8. Converting to C array...")
        c_file_path = self.model_converter.convert_to_c_array(tflite_model)

        # Step 9: Analyze TFLite model
        print("\n9. Analyzing TensorFlow Lite model...")
        tflite_analysis = self.model_converter.analyze_model(tflite_model)

        # Step 10: Create visualizations
        print("\n10. Creating visualizations...")
        self.model_evaluator.plot_training_history(history)
        self.model_evaluator.plot_confusion_matrix(
            evaluation_results['confusion_matrix'],
            evaluation_results['class_names']
        )

        # Save training results
        self.save_training_results(
            dataset_info, history, evaluation_results,
            speed_results, tflite_analysis, model_size
        )

        print("\n" + "=" * 60)
        print("Pipeline completed successfully!")
        print("=" * 60)

        return {
            'model': trained_model,
            'tflite_model': tflite_model,
            'history': history,
            'evaluation': evaluation_results,
            'speed_test': speed_results
        }

    def save_training_results(self, dataset_info, history, evaluation, speed, tflite_analysis, model_size):
        """Save training results to file"""

        results = {
            'timestamp': datetime.now().isoformat(),
            'dataset_info': dataset_info,
            'training_config': {
                'epochs': len(history.history['accuracy']),
                'batch_size': self.config.BATCH_SIZE,
                'learning_rate': self.config.LEARNING_RATE,
                'img_height': self.config.IMG_HEIGHT,
                'img_width': self.config.IMG_WIDTH
            },
            'final_metrics': {
                'accuracy': history.history['accuracy'][-1],
                'val_accuracy': history.history['val_accuracy'][-1],
                'loss': history.history['loss'][-1],
                'val_loss': history.history['val_loss'][-1]
            },
            'inference_speed': speed,
            'model_size_kb': model_size / 1024,
            'tflite_analysis': {
                'input_shape': tflite_analysis['input_shape'].tolist(),
                'output_shape': tflite_analysis['output_shape'].tolist()
            }
        }

        results_path = os.path.join(self.config.MODEL_SAVE_PATH, 'training_results.json')
        with open(results_path, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"Training results saved to: {results_path}")

def main():
    """Main function"""

    parser = argparse.ArgumentParser(description='Wildfire Detection AI Pipeline')
    parser.add_argument('--model-type', choices=['cnn', 'mobilenet'], default='cnn',
                       help='Model type to train')
    parser.add_argument('--quantization', choices=['dynamic', 'full_int8', 'float16'],
                       default='dynamic', help='Quantization type for TFLite conversion')
    parser.add_argument('--analyze-only', action='store_true',
                       help='Only analyze dataset without training')

    args = parser.parse_args()

    # Create pipeline
    pipeline = Pipeline()

    if args.analyze_only:
        # Only analyze dataset
        print("Analyzing dataset only...")
        pipeline.data_manager.analyze_dataset()
    else:
        # Run full pipeline
        results = pipeline.run_full_pipeline(
            model_type=args.model_type,
            quantization=args.quantization
        )

if __name__ == "__main__":
    main()
