#!/usr/bin/env python3
"""
Wildfire Surveillance Ground Station
Complete ground station application for receiving and visualizing drone alerts
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import json
import csv
from datetime import datetime, timedelta
from pathlib import Path
import folium
from folium.plugins import MarkerCluster, HeatMap
import webbrowser
import os
import struct
import queue
import logging
from typing import List, Dict, Optional
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# Protocol constants
ALERT_TYPE = 0x00
TELEMETRY_TYPE = 0x01
HEARTBEAT_TYPE = 0x02

class LoRaProtocolParser:
    """LoRa protocol parser for wildfire surveillance"""

    def __init__(self):
        self.message_counter = 0

    def parse_packet(self, raw_data: bytes) -> Optional[Dict]:
        """Parse LoRa packet based on type"""

        if len(raw_data) < 4:
            return None

        try:
            header = raw_data[0]
            packet_type = header & 0x1F
            priority = (header >> 6) & 0x01

            if packet_type == ALERT_TYPE and len(raw_data) == 32:
                return self._parse_alert_packet(raw_data)
            elif packet_type == TELEMETRY_TYPE and len(raw_data) == 28:
                return self._parse_telemetry_packet(raw_data)
            elif packet_type == HEARTBEAT_TYPE and len(raw_data) == 8:
                return self._parse_heartbeat_packet(raw_data)
            else:
                logging.warning(f"Unknown packet type: {packet_type}")
                return None

        except Exception as e:
            logging.error(f"Error parsing packet: {e}")
            return None

    def _parse_alert_packet(self, data: bytes) -> Dict:
        """Parse ALERT packet"""

        try:
            # Unpack ALERT packet (32 bytes)
            header, msg_id, status, lat, lon, timestamp, confidence, temperature, checksum = \
                struct.unpack('>BBBL LdBBH', data)

            # Verify checksum
            calculated_checksum = self.calculate_crc16(data[:-2])
            if calculated_checksum != checksum:
                raise ValueError(f"Checksum mismatch: {calculated_checksum} != {checksum}")

            return {
                'type': 'ALERT',
                'message_id': msg_id,
                'latitude': lat,
                'longitude': lon,
                'timestamp': datetime.fromtimestamp(timestamp),
                'confidence': confidence,
                'temperature': temperature,
                'status_flags': self._parse_status_flags(status),
                'priority': 'HIGH' if (header >> 6) & 0x01 else 'NORMAL',
                'raw_data': data.hex()
            }

        except struct.error as e:
            logging.error(f"ALERT packet parsing error: {e}")
            return None

    def _parse_telemetry_packet(self, data: bytes) -> Dict:
        """Parse TELEMETRY packet"""

        try:
            # Unpack TELEMETRY packet (28 bytes)
            header, msg_id, status, lat, lon, alt_msb, alt_lsb, speed, heading, \
            satellites, temperature, voltage, current, checksum = \
                struct.unpack('>BBBLLHHBBBBBBH', data)

            altitude = (alt_msb << 8) | alt_lsb

            return {
                'type': 'TELEMETRY',
                'message_id': msg_id,
                'latitude': lat,
                'longitude': lon,
                'altitude': altitude,
                'speed': speed,
                'heading': heading,
                'satellites': satellites,
                'temperature': temperature,
                'voltage': voltage / 10.0,  # Convert to volts
                'current': current / 10.0,  # Convert to amps
                'timestamp': datetime.now(),
                'status_flags': self._parse_status_flags(status)
            }

        except struct.error as e:
            logging.error(f"TELEMETRY packet parsing error: {e}")
            return None

    def _parse_heartbeat_packet(self, data: bytes) -> Dict:
        """Parse HEARTBEAT packet"""

        try:
            header, msg_id, status, checksum = struct.unpack('>BBBH', data)

            return {
                'type': 'HEARTBEAT',
                'message_id': msg_id,
                'timestamp': datetime.now(),
                'status_flags': self._parse_status_flags(status)
            }

        except struct.error as e:
            logging.error(f"HEARTBEAT packet parsing error: {e}")
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

    def calculate_crc16(self, data: bytes) -> int:
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

class AlertData:
    """Data structure for fire alerts"""

    def __init__(self, alert_dict: Dict):
        self.type = alert_dict.get('type', 'UNKNOWN')
        self.message_id = alert_dict.get('message_id', 0)
        self.latitude = alert_dict.get('latitude', 0.0)
        self.longitude = alert_dict.get('longitude', 0.0)
        self.timestamp = alert_dict.get('timestamp', datetime.now())
        self.confidence = alert_dict.get('confidence', 0)
        self.temperature = alert_dict.get('temperature', 0)
        self.status_flags = alert_dict.get('status_flags', {})
        self.priority = alert_dict.get('priority', 'NORMAL')

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization"""
        return {
            'type': self.type,
            'message_id': self.message_id,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'timestamp': self.timestamp.isoformat(),
            'confidence': self.confidence,
            'temperature': self.temperature,
            'status_flags': self.status_flags,
            'priority': self.priority
        }

    def get_marker_color(self) -> str:
        """Get marker color based on alert severity"""
        if self.status_flags.get('critical_alert', False):
            return 'red'
        elif self.confidence > 80:
            return 'orange'
        elif self.confidence > 60:
            return 'yellow'
        else:
            return 'blue'

    def get_marker_icon(self) -> str:
        """Get marker icon based on alert type"""
        if self.status_flags.get('fire_detected', False):
            return 'fire'
        else:
            return 'info-sign'

class MapVisualizer:
    """Folium-based map visualizer"""

    def __init__(self, center_lat: float = 37.7749, center_lon: float = -122.4194, zoom: int = 10):
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.zoom = zoom
        self.map = None
        self.markers = []
        self.heat_data = []
        self.create_map()

    def create_map(self):
        """Create initial map"""
        self.map = folium.Map(
            location=[self.center_lat, self.center_lon],
            zoom_start=self.zoom,
            tiles='OpenStreetMap'
        )
        self.marker_cluster = MarkerCluster().add_to(self.map)

    def add_alert_marker(self, alert: AlertData):
        """Add alert marker to map"""
        if not alert.status_flags.get('fire_detected', False):
            return

        # Create popup content
        popup_content = f"""
        <b>Fire Alert #{alert.message_id}</b><br>
        Confidence: {alert.confidence}%<br>
        Temperature: {alert.temperature}°C<br>
        Time: {alert.timestamp.strftime('%H:%M:%S')}<br>
        GPS: {'Valid' if alert.status_flags.get('gps_valid', False) else 'Invalid'}<br>
        Priority: {alert.priority}
        """

        # Add marker
        marker = folium.Marker(
            location=[alert.latitude, alert.longitude],
            popup=popup_content,
            icon=folium.Icon(
                color=alert.get_marker_color(),
                icon=alert.get_marker_icon()
            )
        )

        marker.add_to(self.marker_cluster)
        self.markers.append(marker)

        # Add to heat data for density visualization
        self.heat_data.append([alert.latitude, alert.longitude, alert.confidence / 100.0])

    def clear_markers(self):
        """Clear all markers"""
        self.marker_cluster = MarkerCluster().add_to(self.map)
        self.markers = []
        self.heat_data = []

    def save_map(self, filename: str = 'fire_alerts_map.html'):
        """Save map to HTML file"""
        # Add heatmap if we have data
        if self.heat_data:
            HeatMap(self.heat_data).add_to(self.map)

        self.map.save(filename)
        return filename

    def open_in_browser(self):
        """Open map in web browser"""
        filename = self.save_map()
        webbrowser.open(f'file://{os.path.abspath(filename)}')

class DataLogger:
    """CSV data logger for alerts and telemetry"""

    def __init__(self, log_directory: str = 'logs'):
        self.log_directory = Path(log_directory)
        self.log_directory.mkdir(exist_ok=True)
        self.current_log_file = None
        self.alert_count = 0
        self.create_new_log_file()

    def create_new_log_file(self):
        """Create new log file for current session"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'fire_alerts_{timestamp}.csv'
        self.current_log_file = self.log_directory / filename

        # Write CSV header
        with open(self.current_log_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp', 'type', 'message_id', 'latitude', 'longitude',
                'confidence', 'temperature', 'fire_detected', 'gps_valid',
                'battery_low', 'critical_alert', 'priority'
            ])

    def log_alert(self, alert: AlertData):
        """Log alert to CSV"""
        with open(self.current_log_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                alert.timestamp.isoformat(),
                alert.type,
                alert.message_id,
                alert.latitude,
                alert.longitude,
                alert.confidence,
                alert.temperature,
                alert.status_flags.get('fire_detected', False),
                alert.status_flags.get('gps_valid', False),
                alert.status_flags.get('battery_low', False),
                alert.status_flags.get('critical_alert', False),
                alert.priority
            ])

        self.alert_count += 1

    def get_recent_alerts(self, hours: int = 24) -> List[Dict]:
        """Get recent alerts from log"""
        alerts = []
        cutoff_time = datetime.now() - timedelta(hours=hours)

        if self.current_log_file.exists():
            with open(self.current_log_file, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    alert_time = datetime.fromisoformat(row['timestamp'])
                    if alert_time > cutoff_time:
                        alerts.append(row)

        return alerts

    def export_to_json(self, filename: str = None) -> str:
        """Export current session data to JSON"""
        if filename is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'alerts_export_{timestamp}.json'

        alerts = self.get_recent_alerts(24)  # Last 24 hours

        with open(filename, 'w') as f:
            json.dump({
                'export_time': datetime.now().isoformat(),
                'total_alerts': len(alerts),
                'alerts': alerts
            }, f, indent=2)

        return filename

class SerialReceiver:
    """Serial communication handler for LoRa receiver"""

    def __init__(self, port: str = None, baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.running = False
        self.parser = LoRaProtocolParser()
        self.data_queue = queue.Queue()

    def connect(self) -> bool:
        """Connect to serial port"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connected = True
            logging.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            logging.error(f"Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def start_listening(self):
        """Start listening for packets"""
        if not self.connected:
            return

        self.running = True
        threading.Thread(target=self._listen_thread, daemon=True).start()

    def stop_listening(self):
        """Stop listening for packets"""
        self.running = False

    def _listen_thread(self):
        """Background thread for receiving packets"""
        buffer = bytearray()

        while self.running and self.connected:
            try:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)

                    # Simple packet detection (looking for header patterns)
                    for byte in data:
                        if byte == 0x00 or byte == 0x20 or byte == 0x40:  # Potential headers
                            if len(buffer) >= 4:  # Minimum packet size
                                packet = self.parser.parse_packet(bytes(buffer))
                                if packet:
                                    self.data_queue.put(packet)
                            buffer.clear()

                        buffer.append(byte)

                        # Prevent buffer overflow
                        if len(buffer) > 64:
                            buffer.clear()

                time.sleep(0.01)  # Small delay to prevent busy waiting

            except Exception as e:
                logging.error(f"Error in listen thread: {e}")
                time.sleep(1)

    def get_packet(self) -> Optional[Dict]:
        """Get next packet from queue"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

    @staticmethod
    def list_available_ports():
        """List available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

class GroundStationApp:
    """Main ground station application"""

    def __init__(self, root):
        self.root = root
        self.root.title("Wildfire Surveillance Ground Station")
        self.root.geometry("1400x900")

        # Initialize components
        self.serial_receiver = None
        self.map_visualizer = MapVisualizer()
        self.data_logger = DataLogger()
        self.alerts = []
        self.telemetry_data = []

        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

        # Create GUI
        self.create_widgets()

        # Start update timer
        self.update_timer()

    def create_widgets(self):
        """Create GUI widgets"""

        # Create main frames
        self.top_frame = ttk.Frame(self.root)
        self.top_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.bottom_frame = ttk.Frame(self.root)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

        # Top frame - Connection controls
        ttk.Label(self.top_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self.top_frame, textvariable=self.port_var, width=20)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_ports()

        ttk.Button(self.top_frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5, pady=5)
        ttk.Button(self.top_frame, text="Connect", command=self.connect_serial).grid(row=0, column=3, padx=5, pady=5)
        ttk.Button(self.top_frame, text="Disconnect", command=self.disconnect_serial).grid(row=0, column=4, padx=5, pady=5)

        # Connection status
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(self.top_frame, textvariable=self.status_var, foreground="red").grid(row=0, column=5, padx=20, pady=5)

        # Main frame - Split into left and right panels
        self.left_frame = ttk.Frame(self.main_frame)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.right_frame = ttk.Frame(self.main_frame)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

        # Left frame - Map placeholder
        ttk.Label(self.left_frame, text="Fire Alert Map", font=("Arial", 12, "bold")).pack(pady=5)
        self.map_frame = ttk.Frame(self.left_frame, width=800, height=600)
        self.map_frame.pack(fill=tk.BOTH, expand=True)

        # Right frame - Controls and alerts
        ttk.Label(self.right_frame, text="Alerts & Controls", font=("Arial", 12, "bold")).pack(pady=5)

        # Control buttons
        button_frame = ttk.Frame(self.right_frame)
        button_frame.pack(fill=tk.X, pady=5)

        ttk.Button(button_frame, text="Open Map", command=self.open_map).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Clear Alerts", command=self.clear_alerts).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="Export Data", command=self.export_data).pack(side=tk.LEFT, padx=2)

        # Alerts listbox
        listbox_frame = ttk.Frame(self.right_frame)
        listbox_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        ttk.Label(listbox_frame, text="Recent Alerts:").pack(anchor=tk.W)
        self.alert_listbox = tk.Listbox(listbox_frame, height=15, font=("Courier", 9))
        self.alert_listbox.pack(fill=tk.BOTH, expand=True, pady=5)

        # Statistics
        stats_frame = ttk.Frame(self.right_frame)
        stats_frame.pack(fill=tk.X, pady=5)

        ttk.Label(stats_frame, text="Statistics:", font=("Arial", 10, "bold")).pack(anchor=tk.W)
        self.stats_text = tk.Text(stats_frame, height=6, width=40, font=("Courier", 8))
        self.stats_text.pack(fill=tk.X, pady=2)
        self.update_stats()

        # Bottom frame - Status bar
        self.status_bar = ttk.Label(self.bottom_frame, text="Ready", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(fill=tk.X)

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = SerialReceiver.list_available_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])

    def connect_serial(self):
        """Connect to serial port"""
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please select a serial port")
            return

        self.serial_receiver = SerialReceiver(port, 9600)
        if self.serial_receiver.connect():
            self.serial_receiver.start_listening()
            self.status_var.set(f"Connected to {port}")
            self.status_bar.config(text=f"Connected to {port}")
        else:
            messagebox.showerror("Connection Error", f"Failed to connect to {port}")

    def disconnect_serial(self):
        """Disconnect from serial port"""
        if self.serial_receiver:
            self.serial_receiver.disconnect()
            self.serial_receiver = None

        self.status_var.set("Disconnected")
        self.status_bar.config(text="Disconnected")

    def update_timer(self):
        """Timer for updating GUI with new data"""
        self.process_incoming_packets()
        self.root.after(100, self.update_timer)  # Update every 100ms

    def process_incoming_packets(self):
        """Process incoming packets from serial receiver"""
        if not self.serial_receiver:
            return

        packet = self.serial_receiver.get_packet()
        if packet:
            self.handle_packet(packet)

    def handle_packet(self, packet: Dict):
        """Handle incoming packet"""
        packet_type = packet.get('type', 'UNKNOWN')

        if packet_type == 'ALERT':
            self.handle_alert_packet(packet)
        elif packet_type == 'TELEMETRY':
            self.handle_telemetry_packet(packet)
        elif packet_type == 'HEARTBEAT':
            self.handle_heartbeat_packet(packet)

    def handle_alert_packet(self, packet: Dict):
        """Handle fire alert packet"""
        alert = AlertData(packet)
        self.alerts.append(alert)

        # Log to file
        self.data_logger.log_alert(alert)

        # Add to map
        self.map_visualizer.add_alert_marker(alert)

        # Update GUI
        self.update_alert_display(alert)

        # Sound alert for high-confidence fires
        if alert.confidence > 70:
            self.root.bell()
            if alert.status_flags.get('critical_alert', False):
                messagebox.showwarning("CRITICAL FIRE ALERT",
                                     ".1f")

        self.update_stats()

    def handle_telemetry_packet(self, packet: Dict):
        """Handle telemetry packet"""
        self.telemetry_data.append(packet)

        # Update status bar with latest telemetry
        voltage = packet.get('voltage', 0)
        current = packet.get('current', 0)
        temperature = packet.get('temperature', 0)

        self.status_bar.config(text=".1f")

    def handle_heartbeat_packet(self, packet: Dict):
        """Handle heartbeat packet"""
        # Update connection status
        system_ok = packet.get('status_flags', {}).get('system_ok', False)
        if system_ok:
            self.status_var.set("Connected (Heartbeat OK)")
        else:
            self.status_var.set("Connected (System Warning)")

    def update_alert_display(self, alert: AlertData):
        """Update alert listbox"""
        alert_text = ".4f"
        self.alert_listbox.insert(0, alert_text)

        # Keep only last 50 alerts in listbox
        if self.alert_listbox.size() > 50:
            self.alert_listbox.delete(50, tk.END)

    def update_stats(self):
        """Update statistics display"""
        fire_alerts = len([a for a in self.alerts if a.status_flags.get('fire_detected', False)])
        total_alerts = len(self.alerts)
        avg_confidence = np.mean([a.confidence for a in self.alerts]) if self.alerts else 0

        stats_text = ".1f" \
                    ".1f" \
                    ".1f"

        self.stats_text.delete(1.0, tk.END)
        self.stats_text.insert(tk.END, stats_text)

    def open_map(self):
        """Open map in browser"""
        self.map_visualizer.open_in_browser()

    def clear_alerts(self):
        """Clear all alerts"""
        self.alerts.clear()
        self.map_visualizer.clear_markers()
        self.alert_listbox.delete(0, tk.END)
        self.update_stats()

    def export_data(self):
        """Export data to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("CSV files", "*.csv"), ("All files", "*.*")]
        )

        if filename:
            if filename.endswith('.json'):
                exported_file = self.data_logger.export_to_json(filename)
            else:
                # Export as CSV
                alerts = self.data_logger.get_recent_alerts(24)
                with open(filename, 'w', newline='') as csvfile:
                    if alerts:
                        writer = csv.DictWriter(csvfile, fieldnames=alerts[0].keys())
                        writer.writeheader()
                        writer.writerows(alerts)
                exported_file = filename

            messagebox.showinfo("Export Complete", f"Data exported to {exported_file}")

def main():
    """Main function"""
    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
