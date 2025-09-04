#!/usr/bin/env python3
"""
LoRa Receiver Module for Ground Station
Handles LoRa communication and packet processing
"""

import serial
import threading
import time
import struct
import logging
from typing import Optional, Dict, Callable
from datetime import datetime

# Protocol constants
ALERT_TYPE = 0x00
TELEMETRY_TYPE = 0x01
HEARTBEAT_TYPE = 0x02

class LoRaReceiver:
    """LoRa receiver for ground station communication"""

    def __init__(self, port: str = "COM3", baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.running = False
        self.packet_callback = None
        self.buffer = bytearray()
        self.logger = logging.getLogger(__name__)

    def set_packet_callback(self, callback: Callable[[Dict], None]):
        """Set callback function for received packets"""
        self.packet_callback = callback

    def connect(self) -> bool:
        """Connect to LoRa receiver"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.connected = True
            self.logger.info(f"Connected to LoRa receiver on {self.port}")
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to LoRa receiver: {e}")
            return False

    def disconnect(self):
        """Disconnect from LoRa receiver"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        self.logger.info("Disconnected from LoRa receiver")

    def start_receiving(self):
        """Start receiving packets"""
        if not self.connected:
            self.logger.error("Not connected to LoRa receiver")
            return False

        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()
        self.logger.info("Started LoRa packet reception")
        return True

    def stop_receiving(self):
        """Stop receiving packets"""
        self.running = False
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=2.0)
        self.logger.info("Stopped LoRa packet reception")

    def _receive_loop(self):
        """Main receive loop"""
        while self.running and self.connected:
            try:
                if self.serial.in_waiting:
                    # Read available data
                    data = self.serial.read(self.serial.in_waiting)
                    self._process_data(data)

                time.sleep(0.01)  # Small delay to prevent busy waiting

            except serial.SerialException as e:
                self.logger.error(f"Serial communication error: {e}")
                self.connected = False
                break
            except Exception as e:
                self.logger.error(f"Unexpected error in receive loop: {e}")
                time.sleep(1.0)

    def _process_data(self, data: bytes):
        """Process incoming serial data"""
        for byte in data:
            # Look for packet headers (first 3 bits indicate version/priority)
            if (byte & 0xE0) in [0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0]:
                # Potential packet start
                if len(self.buffer) >= 8:  # Minimum packet size
                    self._try_parse_packet(bytes(self.buffer))

                self.buffer.clear()

            self.buffer.append(byte)

            # Prevent buffer overflow
            if len(self.buffer) > 128:
                self.logger.warning("Buffer overflow, clearing buffer")
                self.buffer.clear()

        # Try to parse any remaining data in buffer
        if len(self.buffer) >= 8:
            self._try_parse_packet(bytes(self.buffer))
            self.buffer.clear()

    def _try_parse_packet(self, data: bytes):
        """Try to parse a potential packet"""
        try:
            packet = self._parse_packet(data)
            if packet and self.packet_callback:
                self.packet_callback(packet)
        except Exception as e:
            self.logger.debug(f"Failed to parse packet: {e}")

    def _parse_packet(self, data: bytes) -> Optional[Dict]:
        """Parse LoRa packet based on protocol"""

        if len(data) < 4:
            return None

        try:
            # Parse header
            header = data[0]
            version = (header >> 7) & 0x01
            priority = (header >> 6) & 0x01
            packet_type = header & 0x1F

            if packet_type == ALERT_TYPE:
                return self._parse_alert_packet(data, priority)
            elif packet_type == TELEMETRY_TYPE:
                return self._parse_telemetry_packet(data, priority)
            elif packet_type == HEARTBEAT_TYPE:
                return self._parse_heartbeat_packet(data, priority)
            else:
                self.logger.debug(f"Unknown packet type: {packet_type}")
                return None

        except (struct.error, IndexError) as e:
            self.logger.debug(f"Packet parsing error: {e}")
            return None

    def _parse_alert_packet(self, data: bytes, priority: int) -> Optional[Dict]:
        """Parse ALERT packet (32 bytes)"""

        if len(data) != 32:
            return None

        try:
            # Unpack binary data
            header, msg_id, status, lat, lon, timestamp, confidence, temperature, checksum = \
                struct.unpack('>BBBL LdBBH', data)

            # Verify checksum
            calculated_checksum = self._calculate_crc16(data[:-2])
            if calculated_checksum != checksum:
                self.logger.warning(f"ALERT checksum mismatch: {calculated_checksum} != {checksum}")
                return None

            return {
                'type': 'ALERT',
                'message_id': msg_id,
                'latitude': lat,
                'longitude': lon,
                'timestamp': datetime.fromtimestamp(timestamp),
                'confidence': confidence,
                'temperature': temperature,
                'status_flags': self._parse_status_flags(status),
                'priority': 'HIGH' if priority else 'NORMAL',
                'raw_data': data.hex()
            }

        except struct.error as e:
            self.logger.error(f"ALERT packet struct error: {e}")
            return None

    def _parse_telemetry_packet(self, data: bytes, priority: int) -> Optional[Dict]:
        """Parse TELEMETRY packet (28 bytes)"""

        if len(data) != 28:
            return None

        try:
            # Unpack binary data
            header, msg_id, status, lat, lon, alt_msb, alt_lsb, speed, heading, \
            satellites, temperature, voltage, current, checksum = \
                struct.unpack('>BBBLLHHBBBBBBH', data)

            altitude = (alt_msb << 8) | alt_lsb

            return {
                'type': 'TELEMETRY',
                'message_id': msg_id,
                'latitude': lat / 1e7,  # Convert from integer degrees * 1e7
                'longitude': lon / 1e7,
                'altitude': altitude,
                'speed': speed,
                'heading': heading * 2,  # Convert to degrees (0-255 -> 0-510)
                'satellites': satellites,
                'temperature': temperature,
                'voltage': voltage / 10.0,  # Convert to volts
                'current': current / 10.0,  # Convert to amps
                'timestamp': datetime.now(),
                'status_flags': self._parse_status_flags(status),
                'priority': 'HIGH' if priority else 'NORMAL'
            }

        except struct.error as e:
            self.logger.error(f"TELEMETRY packet struct error: {e}")
            return None

    def _parse_heartbeat_packet(self, data: bytes, priority: int) -> Optional[Dict]:
        """Parse HEARTBEAT packet (8 bytes)"""

        if len(data) != 8:
            return None

        try:
            header, msg_id, status, checksum = struct.unpack('>BBBH', data)

            return {
                'type': 'HEARTBEAT',
                'message_id': msg_id,
                'timestamp': datetime.now(),
                'status_flags': self._parse_status_flags(status),
                'priority': 'HIGH' if priority else 'NORMAL'
            }

        except struct.error as e:
            self.logger.error(f"HEARTBEAT packet struct error: {e}")
            return None

    def _parse_status_flags(self, status_byte: int) -> Dict:
        """Parse status flags bitfield"""

        return {
            'fire_detected': bool(status_byte & (1 << 7)),
            'large_fire': bool(status_byte & (1 << 6)),
            'critical_alert': bool(status_byte & (1 << 5)),
            'gps_valid': bool(status_byte & (1 << 4)),
            'battery_low': bool(status_byte & (1 << 3)),
            'system_ok': bool(status_byte & (1 << 2))
        }

    def _calculate_crc16(self, data: bytes) -> int:
        """Calculate CRC-16-CCITT checksum"""

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

    def send_command(self, command: bytes) -> bool:
        """Send command to LoRa module"""
        if not self.connected:
            return False

        try:
            self.serial.write(command)
            self.serial.flush()
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to send command: {e}")
            return False

    def get_connection_status(self) -> Dict:
        """Get connection status information"""
        return {
            'connected': self.connected,
            'running': self.running,
            'port': self.port,
            'baudrate': self.baudrate,
            'buffer_size': len(self.buffer)
        }

# Test function for development
def test_lora_receiver():
    """Test LoRa receiver functionality"""

    def packet_handler(packet):
        print(f"Received packet: {packet['type']}")
        if packet['type'] == 'ALERT':
            print(f"  Fire detected: {packet['status_flags']['fire_detected']}")
            print(".1f")
        print()

    # Create receiver
    receiver = LoRaReceiver("COM3", 9600)
    receiver.set_packet_callback(packet_handler)

    # Connect and start
    if receiver.connect():
        print("Connected to LoRa receiver")
        receiver.start_receiving()

        # Run for 30 seconds
        print("Listening for packets (30 seconds)...")
        time.sleep(30)

        receiver.stop_receiving()
        receiver.disconnect()
        print("Test completed")
    else:
        print("Failed to connect to LoRa receiver")

if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Run test
    test_lora_receiver()
