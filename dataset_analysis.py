#!/usr/bin/env python3
"""
Dataset Analysis Script for Wildfire Detection
Analyzes the fire/no-fire dataset before training
"""

import os
import numpy as np
from PIL import Image
from pathlib import Path

def analyze_dataset():
    """Analyze the dataset structure and image properties"""

    dataset_path = Path("dataset")
    fire_path = dataset_path / "fire"
    no_fire_path = dataset_path / "no_fire"

    print("=" * 60)
    print("WILDFIRE DETECTION DATASET ANALYSIS")
    print("=" * 60)

    # Count images
    fire_images = list(fire_path.glob("*"))
    no_fire_images = list(no_fire_path.glob("*"))

    print(f"Fire images: {len(fire_images)}")
    print(f"No-fire images: {len(no_fire_images)}")
    print(f"Total images: {len(fire_images) + len(no_fire_images)}")

    # Analyze image formats
    print("\n" + "=" * 40)
    print("IMAGE FORMAT ANALYSIS")
    print("=" * 40)

    fire_formats = {}
    no_fire_formats = {}

    for img_path in fire_images:
        ext = img_path.suffix.lower()
        fire_formats[ext] = fire_formats.get(ext, 0) + 1

    for img_path in no_fire_images:
        ext = img_path.suffix.lower()
        no_fire_formats[ext] = no_fire_formats.get(ext, 0) + 1

    print("Fire images formats:")
    for fmt, count in fire_formats.items():
        print(f"  {fmt}: {count} images")

    print("\nNo-fire images formats:")
    for fmt, count in no_fire_formats.items():
        print(f"  {fmt}: {count} images")

    # Analyze image sizes (sample of 10 from each class)
    print("\n" + "=" * 40)
    print("IMAGE SIZE ANALYSIS")
    print("=" * 40)

    def analyze_image_sizes(image_paths, label, sample_size=10):
        sizes = []
        corrupted = 0

        sample_paths = image_paths[:sample_size]  # Take first sample_size images

        for img_path in sample_paths:
            try:
                with Image.open(img_path) as img:
                    sizes.append(img.size)  # (width, height)
            except Exception as e:
                corrupted += 1
                print(f"  Warning: Could not read {img_path.name}: {e}")

        if sizes:
            widths = [s[0] for s in sizes]
            heights = [s[1] for s in sizes]

            print(f"{label} images (sample of {len(sizes)}):")
            print(f"  Average size: {np.mean(widths):.0f} x {np.mean(heights):.0f}")
            print(f"  Min size: {min(widths)} x {min(heights)}")
            print(f"  Max size: {max(widths)} x {max(heights)}")

        if corrupted > 0:
            print(f"  Corrupted images: {corrupted}")

        return sizes

    fire_sizes = analyze_image_sizes(fire_images, "Fire")
    no_fire_sizes = analyze_image_sizes(no_fire_images, "No-fire")

    # Dataset balance analysis
    print("\n" + "=" * 40)
    print("DATASET BALANCE ANALYSIS")
    print("=" * 40)

    total_fire = len(fire_images)
    total_no_fire = len(no_fire_images)
    total_images = total_fire + total_no_fire

    fire_percentage = (total_fire / total_images) * 100
    no_fire_percentage = (total_no_fire / total_images) * 100

    print(f"Fire images: {total_fire} ({fire_percentage:.1f}%)")
    print(f"No-fire images: {total_no_fire} ({no_fire_percentage:.1f}%)")

    if abs(fire_percentage - no_fire_percentage) > 20:
        print("⚠️  WARNING: Dataset is imbalanced (>20% difference)")
        print("   Consider collecting more images for the minority class")
    else:
        print("✅ Dataset balance looks good")

    # Training recommendations
    print("\n" + "=" * 40)
    print("TRAINING RECOMMENDATIONS")
    print("=" * 40)

    min_training_samples = 1000

    if total_images < min_training_samples:
        print(f"⚠️  Current dataset: {total_images} images")
        print(f"   Recommended minimum: {min_training_samples} images")
        print("   Consider using data augmentation or collecting more images")

        if total_images < 500:
            print("   🚨 CRITICAL: Dataset too small for reliable training")
        elif total_images < 800:
            print("   ⚠️  WARNING: Dataset small, results may not be reliable")
        else:
            print("   ✅ Dataset size acceptable with augmentation")
    else:
        print(f"✅ Dataset size: {total_images} images (good for training)")

    # Check for common issues
    print("\n" + "=" * 40)
    print("POTENTIAL ISSUES CHECK")
    print("=" * 40)

    issues = []

    # Check for very small images
    small_images = 0
    for img_path in fire_images[:20] + no_fire_images[:20]:  # Sample
        try:
            with Image.open(img_path) as img:
                if min(img.size) < 96:
                    small_images += 1
        except:
            pass

    if small_images > 5:
        issues.append("Some images are very small (<96px)")

    # Check for mixed formats that might cause issues
    formats = set()
    for img_path in fire_images + no_fire_images:
        formats.add(img_path.suffix.lower())

    if len(formats) > 3:
        issues.append("Many different image formats detected")

    if issues:
        print("⚠️  Potential issues found:")
        for issue in issues:
            print(f"   - {issue}")
    else:
        print("✅ No major issues detected")

    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)

    return {
        'fire_count': total_fire,
        'no_fire_count': total_no_fire,
        'fire_formats': fire_formats,
        'no_fire_formats': no_fire_formats,
        'balance_ok': abs(fire_percentage - no_fire_percentage) <= 20,
        'size_ok': total_images >= 500,
        'issues': issues
    }

if __name__ == "__main__":
    analysis = analyze_dataset()

    # Save analysis results
    print("\n💾 Saving analysis results to dataset_analysis.txt")
    with open("dataset_analysis.txt", "w") as f:
        f.write("WILDFIRE DETECTION DATASET ANALYSIS RESULTS\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Fire images: {analysis['fire_count']}\n")
        f.write(f"No-fire images: {analysis['no_fire_count']}\n")
        f.write(f"Dataset balanced: {'Yes' if analysis['balance_ok'] else 'No'}\n")
        f.write(f"Sufficient size: {'Yes' if analysis['size_ok'] else 'No'}\n")

        if analysis['issues']:
            f.write("\nIssues found:\n")
            for issue in analysis['issues']:
                f.write(f"- {issue}\n")

    print("✅ Analysis complete! Check dataset_analysis.txt for results.")
