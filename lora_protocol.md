# 📡 LoRa Alert System Protocol Specification

## Overview

The LoRa Alert System Protocol defines the communication format for wildfire detection alerts transmitted between drone systems and ground stations. This protocol is designed for long-range, low-power communication in remote areas with limited infrastructure.

### Key Requirements
- **Reliability**: Critical alert information must be transmitted reliably
- **Efficiency**: Minimize power consumption and bandwidth usage
- **Extensibility**: Support for future enhancements and additional data types
- **Robustness**: Error detection and correction capabilities
- **Interoperability**: Standard format for multiple drone systems

## Protocol Architecture

### Communication Model
```
Drone → LoRa Gateway → Ground Station
   ↓         ↓              ↓
ESP32 → SX1278 Module → Python Receiver
   ↓         ↓              ↓
UART  → SPI Interface → Serial Port
```

### Frequency and Parameters
- **Frequency Band**: 433MHz (ISM band)
- **Modulation**: LoRa
- **Spreading Factor**: 9 (balance of range and speed)
- **Bandwidth**: 125kHz
- **Coding Rate**: 4/5
- **Transmit Power**: 20dBm (100mW)
- **Expected Range**: 5-10km (line of sight)

## Message Format Specification

### Packet Structure

All packets follow this general structure:

```
┌─────────────┬─────────────┬─────────────┬─────────────────┐
│   Header    │   Length    │   Type      │     Payload     │
│   (1 byte)  │  (1 byte)   │  (1 byte)   │   (variable)    │
└─────────────┴─────────────┴─────────────┴─────────────────┘
```

### Header Format (1 byte)
```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│   7     │   6     │   5     │   4     │   3     │   2     │   1     │   0     │
│ Version │ Priority│ Reserved│ Reserved│ Reserved│ Reserved│ Reserved│ Type   │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

- **Version (2 bits)**: Protocol version (currently 00)
- **Priority (1 bit)**: 1 = High priority (fire alert), 0 = Normal
- **Type (5 bits)**: Message type identifier

### Message Types

| Type ID | Message Type | Description |
|---------|--------------|-------------|
| 0x00 | ALERT | Fire detection alert |
| 0x01 | TELEMETRY | Flight telemetry data |
| 0x02 | HEARTBEAT | System status ping |
| 0x03 | GPS_UPDATE | GPS position update |
| 0x04 | BATTERY_STATUS | Battery level warning |
| 0x05-0x1F | Reserved | Future use |

## Alert Message Format

### ALERT Packet Structure (32 bytes total)

```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Header  │ Msg ID  │ Status  │ Lat MSB │ Lat LSB │ Lon MSB │ Lon LSB │ Time    │
│ (1 byte)│ (1 byte)│ (1 byte)│ (4 bytes│ (4 bytes│ (4 bytes│ (4 bytes│ (8 bytes│
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
│ Time    │ Conf    │ Temp    │ Checksum│
│ (cont)  │ (1 byte)│ (1 byte)│ (2 bytes)│
└─────────┴─────────┴─────────┴─────────┘
```

### Field Definitions

| Field | Size | Type | Description | Range/Units |
|-------|------|------|-------------|-------------|
| Header | 1 byte | uint8 | Packet type and flags | See header format |
| Msg ID | 1 byte | uint8 | Sequential message counter | 0-255 |
| Status | 1 byte | uint8 | Status flags bitfield | See status flags |
| Latitude | 4 bytes | float32 | GPS latitude | -90.0 to +90.0 |
| Longitude | 4 bytes | float32 | GPS longitude | -180.0 to +180.0 |
| Timestamp | 8 bytes | uint64 | Unix timestamp | Seconds since epoch |
| Confidence | 1 byte | uint8 | AI confidence score | 0-100 (%) |
| Temperature | 1 byte | int8 | Ambient temperature | -128 to +127 (°C) |
| Checksum | 2 bytes | uint16 | CRC-16 checksum | 0-65535 |

### Status Flags Bitfield (1 byte)

```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│   7     │   6     │   5     │   4     │   3     │   2     │   1     │   0     │
│ Fire    │ Large   │ Critical│ GPS     │ Battery │ System  │ Reserved│ Reserved│
│ Detected│ Fire    │ Alert   │ Valid   │ Low     │ OK      │         │         │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

- **Bit 7**: Fire Detected (1 = Fire detected, 0 = No fire)
- **Bit 6**: Large Fire (1 = Large fire >80% confidence, 0 = Small fire)
- **Bit 5**: Critical Alert (1 = Immediate response required)
- **Bit 4**: GPS Valid (1 = GPS fix available, 0 = No GPS fix)
- **Bit 3**: Battery Low (1 = Battery <20%, 0 = Battery OK)
- **Bit 2**: System OK (1 = All systems nominal, 0 = System error)
- **Bit 1**: Reserved
- **Bit 0**: Reserved

## Telemetry Message Format

### TELEMETRY Packet Structure (28 bytes total)

```
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Header  │ Msg ID  │ Status  │ Lat MSB │ Lat LSB │ Lon MSB │ Lon LSB │ Alt     │
│ (1 byte)│ (1 byte)│ (1 byte)│ (4 bytes│ (4 bytes│ (4 bytes│ (4 bytes│ (2 bytes│
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
│ Alt     │ Speed   │ Heading │ Sat     │ Temp    │ Voltage │ Current │ Checksum│
│ (cont)  │ (1 byte)│ (1 byte) │ (1 byte)│ (1 byte)│ (1 byte)│ (1 byte)│ (2 bytes)│
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

### Telemetry Field Definitions

| Field | Size | Type | Description | Units/Range |
|-------|------|------|-------------|-------------|
| Header | 1 byte | uint8 | Packet type and flags | - |
| Msg ID | 1 byte | uint8 | Sequential message counter | 0-255 |
| Status | 1 byte | uint8 | System status flags | - |
| Latitude | 4 bytes | float32 | Current latitude | -90.0 to +90.0 |
| Longitude | 4 bytes | float32 | Current longitude | -180.0 to +180.0 |
| Altitude | 2 bytes | uint16 | Altitude above ground | 0-65535 meters |
| Speed | 1 byte | uint8 | Ground speed | 0-255 km/h |
| Heading | 1 byte | uint8 | Heading direction | 0-255 (0-360°) |
| Satellites | 1 byte | uint8 | GPS satellites in view | 0-255 |
| Temperature | 1 byte | int8 | System temperature | -128 to +127 °C |
| Voltage | 1 byte | uint8 | Battery voltage | 0-25.5V (x10) |
| Current | 1 byte | uint8 | Current draw | 0-25.5A (x10) |
| Checksum | 2 bytes | uint16 | CRC-16 checksum | - |

## Implementation Examples

### ESP32 Alert Packet Creation

```cpp
// Alert packet structure
struct AlertPacket {
    uint8_t header;
    uint8_t message_id;
    uint8_t status;
    float latitude;
    float longitude;
    uint64_t timestamp;
    uint8_t confidence;
    int8_t temperature;
    uint16_t checksum;
};

AlertPacket createFireAlert(float lat, float lon, uint8_t conf, int8_t temp) {
    static uint8_t msg_counter = 0;

    AlertPacket packet;

    // Header: Version 0, High priority, ALERT type
    packet.header = (0 << 7) | (1 << 6) | ALERT_TYPE;

    packet.message_id = msg_counter++;
    packet.status = createStatusFlags(conf);
    packet.latitude = lat;
    packet.longitude = lon;
    packet.timestamp = getUnixTimestamp();
    packet.confidence = conf;
    packet.temperature = temp;
    packet.checksum = calculateCRC16(&packet, sizeof(packet) - 2);

    return packet;
}

uint8_t createStatusFlags(uint8_t confidence) {
    uint8_t flags = 0;

    // Fire detected
    flags |= (1 << 7);

    // Large fire if confidence > 80%
    if (confidence > 80) {
        flags |= (1 << 6);
    }

    // Critical alert if confidence > 90%
    if (confidence > 90) {
        flags |= (1 << 5);
    }

    // GPS valid (assume always valid for this example)
    flags |= (1 << 4);

    // Battery low check
    if (getBatteryLevel() < 20) {
        flags |= (1 << 3);
    }

    // System OK
    flags |= (1 << 2);

    return flags;
}
```

### Python Alert Packet Parser

```python
import struct
import time
from datetime import datetime

class LoRaAlertParser:
    def __init__(self):
        self.message_counter = 0

    def parse_alert_packet(self, raw_data):
        """Parse ALERT packet from raw bytes"""

        if len(raw_data) != 32:
            raise ValueError(f"Invalid packet length: {len(raw_data)} (expected 32)")

        try:
            # Unpack binary data
            header, msg_id, status, lat, lon, timestamp, confidence, temperature, checksum = \
                struct.unpack('>BBBL LdBBH', raw_data)

            # Verify header
            if (header & 0x1F) != ALERT_TYPE:
                raise ValueError(f"Invalid packet type: {header & 0x1F}")

            # Verify checksum
            calculated_checksum = self.calculate_crc16(raw_data[:-2])
            if calculated_checksum != checksum:
                raise ValueError(f"Checksum mismatch: {calculated_checksum} != {checksum}")

            # Parse status flags
            status_flags = self.parse_status_flags(status)

            # Convert timestamp
            timestamp_dt = datetime.fromtimestamp(timestamp)

            return {
                'packet_type': 'ALERT',
                'message_id': msg_id,
                'latitude': lat,
                'longitude': lon,
                'timestamp': timestamp_dt,
                'confidence': confidence,
                'temperature': temperature,
                'status_flags': status_flags,
                'raw_data': raw_data.hex()
            }

        except struct.error as e:
            raise ValueError(f"Packet parsing error: {e}")

    def parse_status_flags(self, status_byte):
        """Parse status flags bitfield"""

        return {
            'fire_detected': bool(status_byte & (1 << 7)),
            'large_fire': bool(status_byte & (1 << 6)),
            'critical_alert': bool(status_byte & (1 << 5)),
            'gps_valid': bool(status_byte & (1 << 4)),
            'battery_low': bool(status_byte & (1 << 3)),
            'system_ok': bool(status_byte & (1 << 2))
        }

    def calculate_crc16(self, data):
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

# Usage example
parser = LoRaAlertParser()

# Simulate receiving packet data
raw_packet = b'\\x00\\x01\\x94\\x42\\x8c\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00'  # Example data

try:
    alert = parser.parse_alert_packet(raw_packet)
    print("Alert parsed successfully:")
    print(f"Fire detected: {alert['fire_detected']}")
    print(f"Confidence: {alert['confidence']}%")
    print(f"Location: {alert['latitude']:.6f}, {alert['longitude']:.6f}")
    print(f"Status: {alert['status_flags']}")
except ValueError as e:
    print(f"Error parsing packet: {e}")
```

### LoRa Transmission Implementation

```cpp
// ESP32 LoRa transmission example
#include <LoRa.h>
#include <SPI.h>

// LoRa pins
#define LORA_CS 5
#define LORA_RST 4
#define LORA_DIO0 2

class LoRaTransmitter {
private:
    bool initialized;

public:
    LoRaTransmitter() : initialized(false) {}

    bool init() {
        SPI.begin(18, 19, 23, 5);  // SCK, MISO, MOSI, CS
        LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

        if (!LoRa.begin(433E6)) {
            Serial.println("LoRa initialization failed!");
            return false;
        }

        // Configure LoRa parameters
        LoRa.setSpreadingFactor(9);
        LoRa.setSignalBandwidth(125E3);
        LoRa.setCodingRate4(5);
        LoRa.setTxPower(20);

        initialized = true;
        Serial.println("LoRa initialized successfully");
        return true;
    }

    bool sendAlert(const AlertPacket& packet) {
        if (!initialized) {
            return false;
        }

        LoRa.beginPacket();
        LoRa.write((uint8_t*)&packet, sizeof(packet));

        int result = LoRa.endPacket();
        if (result == 1) {
            Serial.println("Alert packet sent successfully");
            return true;
        } else {
            Serial.println("Failed to send alert packet");
            return false;
        }
    }

    // Get transmission statistics
    int getLastPacketRSSI() {
        return LoRa.packetRssi();
    }

    float getLastPacketSNR() {
        return LoRa.packetSnr();
    }
};

// Usage in main code
LoRaTransmitter lora;

void setup() {
    Serial.begin(115200);

    if (!lora.init()) {
        Serial.println("LoRa init failed");
        while(1);
    }
}

void loop() {
    // Create and send fire alert
    AlertPacket alert = createFireAlert(37.7749, -122.4194, 85, 25);

    if (lora.sendAlert(alert)) {
        Serial.print("RSSI: ");
        Serial.print(lora.getLastPacketRSSI());
        Serial.print(" dBm, SNR: ");
        Serial.print(lora.getLastPacketSNR());
        Serial.println(" dB");
    }

    delay(30000);  // Send every 30 seconds
}
```

## Error Handling and Reliability

### Packet Validation
1. **Length Check**: Verify packet is correct length for its type
2. **Header Validation**: Check version and message type
3. **Checksum Verification**: CRC-16 validation
4. **Range Validation**: Check coordinate and value ranges

### Transmission Reliability
1. **Acknowledgment System**: Optional ACK packets for critical alerts
2. **Retransmission**: Automatic retry for failed transmissions
3. **Redundancy**: Send critical alerts multiple times
4. **Error Correction**: Forward error correction for noisy environments

### Recovery Procedures
1. **Packet Loss**: Sequence number tracking for gap detection
2. **Corruption**: Checksum failure triggers retransmission request
3. **Out of Range**: RSSI monitoring for link quality assessment
4. **System Reset**: Automatic reset on persistent communication failures

## Protocol Extensions

### Future Message Types

#### GPS_UPDATE Packet (16 bytes)
- Header, Msg ID, Latitude, Longitude, Altitude, Timestamp, Checksum

#### BATTERY_STATUS Packet (12 bytes)
- Header, Msg ID, Voltage, Current, Temperature, Capacity, Checksum

#### SYSTEM_CONFIG Packet (Variable length)
- Header, Msg ID, Config Data, Checksum

### Enhanced Features
1. **Compression**: LZ77 compression for large payloads
2. **Encryption**: AES-128 encryption for secure communications
3. **Multi-hop**: Mesh networking for extended range
4. **Quality of Service**: Priority queuing for different message types

## Testing and Validation

### Packet Format Testing
```python
def test_packet_formats():
    """Test packet creation and parsing"""

    # Test ALERT packet
    alert_data = {
        'latitude': 37.7749,
        'longitude': -122.4194,
        'confidence': 85,
        'temperature': 25
    }

    # Create packet (ESP32 side)
    packet = create_alert_packet(alert_data)

    # Parse packet (Python side)
    parsed = parse_alert_packet(packet)

    # Verify data integrity
    assert abs(parsed['latitude'] - alert_data['latitude']) < 0.0001
    assert abs(parsed['longitude'] - alert_data['longitude']) < 0.0001
    assert parsed['confidence'] == alert_data['confidence']
    assert parsed['temperature'] == alert_data['temperature']

    print("Packet format test passed!")
```

### Range Testing
```python
def test_communication_range():
    """Test LoRa communication range"""

    distances = [100, 500, 1000, 2000, 5000]  # meters
    results = {}

    for distance in distances:
        print(f"Testing at {distance}m...")

        # Move to test location
        input(f"Move to {distance}m position and press Enter")

        # Send test packets
        success_count = 0
        rssi_values = []
        snr_values = []

        for i in range(10):  # Send 10 packets
            success, rssi, snr = send_test_packet()
            if success:
                success_count += 1
                rssi_values.append(rssi)
                snr_values.append(snr)

        results[distance] = {
            'success_rate': success_count / 10,
            'avg_rssi': sum(rssi_values) / len(rssi_values) if rssi_values else None,
            'avg_snr': sum(snr_values) / len(snr_values) if snr_values else None
        }

    return results
```

## Performance Metrics

### Bandwidth Analysis
- **ALERT Packet**: 32 bytes every 30 seconds = 8.53 bps
- **TELEMETRY Packet**: 28 bytes every 10 seconds = 22.4 bps
- **HEARTBEAT Packet**: 8 bytes every 60 seconds = 1.07 bps
- **Total Average**: ~32 bps (well within LoRa capacity)

### Power Consumption
- **Transmit Current**: 120mA @ 3.3V (20dBm)
- **Receive Current**: 10mA @ 3.3V
- **Sleep Current**: <1μA
- **Energy per ALERT**: ~4mJ (132ms transmit time)

### Reliability Metrics
- **Packet Loss Rate**: <5% within 5km range
- **False Positive Rate**: <2% (with proper AI model)
- **Detection Time**: <2 seconds (capture to transmission)
- **Geolocation Accuracy**: ±5m (with GPS)

This protocol specification provides a robust foundation for reliable wildfire detection alerts in challenging remote environments.
