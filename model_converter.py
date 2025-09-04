#!/usr/bin/env python3
"""
TensorFlow Lite Model Converter for ESP32
Converts .tflite model to C header file for ESP32 embedding
"""

import os
import sys
from pathlib import Path

def convert_tflite_to_c_array(tflite_path, output_path, array_name="model_data"):
    """
    Convert TensorFlow Lite model to C array header file

    Args:
        tflite_path: Path to .tflite model file
        output_path: Path for output .h file
        array_name: Name of the C array
    """

    # Read the .tflite file
    with open(tflite_path, 'rb') as f:
        tflite_data = f.read()

    print(f"Model size: {len(tflite_data)} bytes ({len(tflite_data)/1024:.1f} KB)")

    # Convert to hex array
    hex_array = [f'0x{byte:02x}' for byte in tflite_data]

    # Create C header file content
    c_content = f'''// Auto-generated TensorFlow Lite model data
// Generated from: {os.path.basename(tflite_path)}
// Model size: {len(tflite_data)} bytes
// Generated on: {__import__("datetime").datetime.now().strftime("%Y-%m-%d %H:%M:%S")}

#ifndef MODEL_DATA_H_
#define MODEL_DATA_H_

#include <cstdint>

const unsigned int {array_name}_len = {len(tflite_data)};
const unsigned char {array_name}[] = {{
    {', '.join(hex_array[:16])}'''

    # Add remaining bytes in chunks of 16
    for i in range(16, len(hex_array), 16):
        chunk = ', '.join(hex_array[i:i+16])
        c_content += f''',
    {chunk}'''

    c_content += f'''
}};

#endif  // MODEL_DATA_H_
'''

    # Write to output file
    with open(output_path, 'w') as f:
        f.write(c_content)

    print(f"C header file saved to: {output_path}")
    print(f"Array name: {array_name}")
    print("Include this file in your ESP32-CAM .ino sketch")

def create_model_data_placeholder(output_path, size_kb=100):
    """
    Create a placeholder model data file for testing
    """
    # Create dummy data (all zeros)
    dummy_size = size_kb * 1024
    dummy_data = b'\\x00' * dummy_size

    # Convert to hex array
    hex_array = [f'0x{byte:02x}' for byte in dummy_data]

    c_content = f'''// Placeholder TensorFlow Lite model data
// This is dummy data for testing - replace with your actual model
// Size: {dummy_size} bytes ({size_kb} KB)

#ifndef MODEL_DATA_H_
#define MODEL_DATA_H_

#include <cstdint>

const unsigned int model_data_len = {dummy_size};
const unsigned char model_data[] = {{
    {', '.join(hex_array[:16])}'''

    # Add remaining bytes in chunks of 16 (but limit to avoid huge files)
    max_lines = 1000  # Limit output size for readability
    for i in range(16, min(len(hex_array), 16 + (max_lines-1) * 16), 16):
        chunk = ', '.join(hex_array[i:i+16])
        c_content += f''',
    {chunk}'''

    if len(hex_array) > 16 + (max_lines-1) * 16:
        remaining = len(hex_array) - (16 + (max_lines-1) * 16)
        c_content += f'''
    // ... {remaining} more bytes ...'''

    c_content += f'''
}};

#endif  // MODEL_DATA_H_
'''

    with open(output_path, 'w') as f:
        f.write(c_content)

    print(f"Placeholder C header file saved to: {output_path}")
    print("⚠️  WARNING: This contains dummy data - replace with your actual model!")

def main():
    """Main function"""
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python model_converter.py <tflite_file> <output_header.h>")
        print("  python model_converter.py --placeholder <output_header.h> [size_kb]")
        print()
        print("Examples:")
        print("  python model_converter.py my_model.tflite model_data.h")
        print("  python model_converter.py --placeholder model_data.h 100")
        return

    if sys.argv[1] == "--placeholder":
        output_path = sys.argv[2]
        size_kb = int(sys.argv[3]) if len(sys.argv) > 3 else 100
        create_model_data_placeholder(output_path, size_kb)
    else:
        tflite_path = sys.argv[1]
        output_path = sys.argv[2]

        if not os.path.exists(tflite_path):
            print(f"Error: TFLite file not found: {tflite_path}")
            return

        convert_tflite_to_c_array(tflite_path, output_path)

if __name__ == "__main__":
    main()
