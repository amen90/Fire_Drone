#!/usr/bin/env python3
"""
Wildfire Detection Dataset Collection Tool
Utilities for collecting and preparing training data
"""

import os
import cv2
import numpy as np
import requests
from pathlib import Path
import argparse
import time
from datetime import datetime
import shutil
from PIL import Image
import glob

class DatasetCollector:
    """Handles dataset collection from various sources"""

    def __init__(self, base_path="dataset"):
        self.base_path = Path(base_path)
        self.fire_path = self.base_path / "fire"
        self.no_fire_path = self.base_path / "no_fire"
        self.ensure_directories()

    def ensure_directories(self):
        """Create necessary directories"""
        self.fire_path.mkdir(parents=True, exist_ok=True)
        self.no_fire_path.mkdir(parents=True, exist_ok=True)
        print(f"Dataset directories created at: {self.base_path}")

    def collect_from_webcam(self, num_samples=100, class_type="fire", delay=1.0):
        """
        Collect images from webcam
        Args:
            num_samples: Number of images to capture
            class_type: "fire" or "no_fire"
            delay: Delay between captures in seconds
        """

        target_path = self.fire_path if class_type == "fire" else self.no_fire_path

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open webcam")
            return

        print(f"Starting webcam capture for {class_type} images...")
        print("Press 'q' to quit, 's' to save current frame, or wait for automatic capture")

        count = 0
        auto_capture = True

        while count < num_samples:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame")
                break

            # Display the frame
            cv2.imshow(f'Webcam Capture - {class_type} ({count}/{num_samples})', frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s') or auto_capture:
                # Save the frame
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{class_type}_{timestamp}_{count:04d}.jpg"
                filepath = target_path / filename

                cv2.imwrite(str(filepath), frame)
                print(f"Saved: {filepath}")
                count += 1

                if auto_capture:
                    time.sleep(delay)

        cap.release()
        cv2.destroyAllWindows()
        print(f"Collected {count} {class_type} images")

    def download_from_urls(self, urls_file, class_type="fire"):
        """
        Download images from URLs
        Args:
            urls_file: Text file containing image URLs (one per line)
            class_type: "fire" or "no_fire"
        """

        target_path = self.fire_path if class_type == "fire" else self.no_fire_path

        if not os.path.exists(urls_file):
            print(f"Error: URLs file {urls_file} not found")
            return

        with open(urls_file, 'r') as f:
            urls = [line.strip() for line in f if line.strip()]

        print(f"Downloading {len(urls)} {class_type} images...")

        for i, url in enumerate(urls):
            try:
                response = requests.get(url, timeout=10)
                if response.status_code == 200:
                    # Get file extension from URL or default to .jpg
                    ext = '.jpg'
                    if '.' in url.split('/')[-1]:
                        ext = '.' + url.split('/')[-1].split('.')[-1]

                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = f"{class_type}_download_{timestamp}_{i:04d}{ext}"
                    filepath = target_path / filename

                    with open(filepath, 'wb') as f:
                        f.write(response.content)

                    print(f"Downloaded: {filepath}")
                else:
                    print(f"Failed to download: {url} (Status: {response.status_code})")

            except Exception as e:
                print(f"Error downloading {url}: {e}")

            # Add delay to be respectful to servers
            time.sleep(0.5)

    def augment_existing_dataset(self, target_samples=1000):
        """Augment existing dataset with transformations"""

        print("Augmenting existing dataset...")

        for class_type in ["fire", "no_fire"]:
            source_path = self.fire_path if class_type == "fire" else self.no_fire_path
            existing_images = list(source_path.glob("*.jpg")) + list(source_path.glob("*.png"))

            if not existing_images:
                print(f"No existing {class_type} images found")
                continue

            current_count = len(existing_images)
            print(f"Found {current_count} existing {class_type} images")

            if current_count >= target_samples:
                print(f"Already have enough {class_type} images")
                continue

            needed = target_samples - current_count
            print(f"Generating {needed} augmented {class_type} images")

            # Generate augmented images
            self._generate_augmented_images(existing_images, source_path, needed, class_type)

    def _generate_augmented_images(self, source_images, target_path, needed, class_type):
        """Generate augmented images from existing ones"""

        augmentations = 0

        while augmentations < needed:
            # Randomly select source image
            source_img_path = np.random.choice(source_images)
            img = cv2.imread(str(source_img_path))

            if img is None:
                continue

            # Apply random augmentations
            augmented_img = self._apply_random_augmentation(img)

            # Save augmented image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{class_type}_aug_{timestamp}_{augmentations:04d}.jpg"
            filepath = target_path / filename

            cv2.imwrite(str(filepath), augmented_img)
            augmentations += 1

            if augmentations % 100 == 0:
                print(f"Generated {augmentations}/{needed} augmented {class_type} images")

    def _apply_random_augmentation(self, img):
        """Apply random augmentations to image"""

        # Random rotation
        if np.random.random() > 0.5:
            angle = np.random.uniform(-15, 15)
            h, w = img.shape[:2]
            M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1)
            img = cv2.warpAffine(img, M, (w, h))

        # Random brightness/contrast
        if np.random.random() > 0.5:
            alpha = np.random.uniform(0.8, 1.2)  # Contrast
            beta = np.random.uniform(-20, 20)    # Brightness
            img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        # Random flip
        if np.random.random() > 0.5:
            flip_code = np.random.choice([-1, 0, 1])  # -1: both, 0: vertical, 1: horizontal
            img = cv2.flip(img, flip_code)

        # Random crop and resize (simulate zoom)
        if np.random.random() > 0.5:
            h, w = img.shape[:2]
            crop_factor = np.random.uniform(0.9, 1.0)
            crop_h, crop_w = int(h * crop_factor), int(w * crop_factor)
            start_y = np.random.randint(0, h - crop_h + 1)
            start_x = np.random.randint(0, w - crop_w + 1)
            img = img[start_y:start_y+crop_h, start_x:start_x+crop_w]
            img = cv2.resize(img, (w, h))

        return img

    def validate_dataset(self):
        """Validate dataset integrity"""

        print("Validating dataset...")

        issues = []

        for class_type in ["fire", "no_fire"]:
            class_path = self.fire_path if class_type == "fire" else self.no_fire_path
            images = list(class_path.glob("*.jpg")) + list(class_path.glob("*.png"))

            print(f"\n{class_type.upper()} class:")
            print(f"  Total images: {len(images)}")

            valid_images = 0
            corrupted_images = []

            for img_path in images:
                try:
                    img = Image.open(img_path)
                    img.verify()  # Check if image is corrupted
                    valid_images += 1
                except Exception as e:
                    corrupted_images.append(str(img_path))
                    issues.append(f"Corrupted {class_type} image: {img_path}")

            print(f"  Valid images: {valid_images}")
            print(f"  Corrupted images: {len(corrupted_images)}")

            if corrupted_images:
                print("  Corrupted files:")
                for corrupted in corrupted_images[:5]:  # Show first 5
                    print(f"    {corrupted}")
                if len(corrupted_images) > 5:
                    print(f"    ... and {len(corrupted_images) - 5} more")

        if issues:
            print(f"\nFound {len(issues)} issues:")
            for issue in issues[:10]:  # Show first 10
                print(f"  - {issue}")
            if len(issues) > 10:
                print(f"  ... and {len(issues) - 10} more issues")

            # Ask user if they want to fix issues
            fix = input("\nDo you want to remove corrupted images? (y/n): ")
            if fix.lower() == 'y':
                self._fix_dataset_issues(issues)
        else:
            print("\nDataset validation passed! No issues found.")

    def _fix_dataset_issues(self, issues):
        """Remove corrupted images"""

        corrupted_files = []
        for issue in issues:
            if "Corrupted" in issue:
                # Extract file path from issue message
                file_path = issue.split(": ")[1]
                corrupted_files.append(file_path)

        removed_count = 0
        for file_path in corrupted_files:
            try:
                os.remove(file_path)
                removed_count += 1
            except Exception as e:
                print(f"Error removing {file_path}: {e}")

        print(f"Removed {removed_count} corrupted images")

    def get_dataset_stats(self):
        """Get comprehensive dataset statistics"""

        stats = {
            'fire': {},
            'no_fire': {},
            'total': {}
        }

        for class_type in ["fire", "no_fire"]:
            class_path = self.fire_path if class_type == "fire" else self.no_fire_path
            images = list(class_path.glob("*.jpg")) + list(class_path.glob("*.png"))

            sizes = []
            for img_path in images[:100]:  # Sample first 100 for size analysis
                try:
                    img = Image.open(img_path)
                    sizes.append(img.size)
                except:
                    continue

            stats[class_type] = {
                'count': len(images),
                'avg_width': np.mean([s[0] for s in sizes]) if sizes else 0,
                'avg_height': np.mean([s[1] for s in sizes]) if sizes else 0,
                'min_width': min([s[0] for s in sizes]) if sizes else 0,
                'max_width': max([s[0] for s in sizes]) if sizes else 0,
                'min_height': min([s[1] for s in sizes]) if sizes else 0,
                'max_height': max([s[1] for s in sizes]) if sizes else 0
            }

        stats['total'] = {
            'count': stats['fire']['count'] + stats['no_fire']['count'],
            'balance_ratio': stats['fire']['count'] / max(stats['no_fire']['count'], 1)
        }

        return stats

    def print_stats(self):
        """Print dataset statistics"""

        stats = self.get_dataset_stats()

        print("\n" + "="*50)
        print("DATASET STATISTICS")
        print("="*50)

        print(f"\nTotal Images: {stats['total']['count']}")
        print(f"Fire Images: {stats['fire']['count']}")
        print(f"No-Fire Images: {stats['no_fire']['count']}")
        print(f"Balance Ratio: {stats['total']['balance_ratio']:.2f}")

        for class_type in ["fire", "no_fire"]:
            print(f"\n{class_type.upper()} Class:")
            print(f"  Count: {stats[class_type]['count']}")
            if stats[class_type]['count'] > 0:
                print(f"  Average Size: {stats[class_type]['avg_width']:.0f} x {stats[class_type]['avg_height']:.0f}")
                print(f"  Width Range: {stats[class_type]['min_width']} - {stats[class_type]['max_width']}")
                print(f"  Height Range: {stats[class_type]['min_height']} - {stats[class_type]['max_height']}")

        print("\n" + "="*50)

def main():
    """Main function"""

    parser = argparse.ArgumentParser(description='Wildfire Detection Dataset Collection Tool')
    parser.add_argument('--mode', choices=['webcam', 'download', 'augment', 'validate', 'stats'],
                       default='stats', help='Operation mode')
    parser.add_argument('--class-type', choices=['fire', 'no_fire'], default='fire',
                       help='Image class for webcam/download modes')
    parser.add_argument('--num-samples', type=int, default=100,
                       help='Number of samples to collect')
    parser.add_argument('--urls-file', type=str,
                       help='File containing image URLs for download mode')
    parser.add_argument('--delay', type=float, default=1.0,
                       help='Delay between captures in seconds')

    args = parser.parse_args()

    collector = DatasetCollector()

    if args.mode == 'webcam':
        collector.collect_from_webcam(args.num_samples, args.class_type, args.delay)
    elif args.mode == 'download':
        if not args.urls_file:
            print("Error: --urls-file required for download mode")
            return
        collector.download_from_urls(args.urls_file, args.class_type)
    elif args.mode == 'augment':
        collector.augment_existing_dataset(args.num_samples)
    elif args.mode == 'validate':
        collector.validate_dataset()
    elif args.mode == 'stats':
        collector.print_stats()

if __name__ == "__main__":
    main()
