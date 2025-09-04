# 🔌 Complete Wiring Diagrams & Schematics

## System Overview

```
Power Flow:     Battery → PDB → ESCs → Motors
                Battery → PDB → ESP32-FC → Sensors
                Battery → PDB → ESP32-CAM → Camera

Communication:  ESP32-FC ↔ ESP32-CAM (I2C)
                ESP32-FC ↔ GPS (UART)
                ESP32-FC ↔ LoRa (SPI)
                ESP32-FC → ESCs (PWM)
                LoRa → Ground Station (RF)
```

## 1. ESP32 Flight Controller Pin Mapping

### ESP32-DevKitC Pin Assignments

```
ESP32-DevKitC V4 Pinout (Top View):
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│                EN ○           ○ 23                         │
│                VP ○           ○ 22                         │
│                VN ○           ○ 1                           │
│                34 ○           ○ 3                           │
│                35 ○           ○ 21                         │
│                32 ○           ○ GND                        │
│                33 ○           ○ GND                        │
│                25 ○           ○ 19                         │
│                26 ○           ○ 18                         │
│                27 ○           ○ 5                          │
│                14 ○           ○ 17                         │
│                12 ○           ○ 16                         │
│                13 ○           ○ 4                          │
│                GND ○          ○ 0                          │
│                VIN ○          ○ 2                          │
│                36 ○           ○ 15                         │
│                                                             │
│  USB/Micro-USB                          ○ GND ○ 13 ○ 12 ○ │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Flight Controller Pin Configuration

| ESP32 Pin | Function | Connected To | Notes |
|-----------|----------|--------------|-------|
| **Motor Control** | | | |
| GPIO 12 | Motor 1 PWM | ESC 1 Signal | Front Left |
| GPIO 13 | Motor 2 PWM | ESC 2 Signal | Front Right |
| GPIO 14 | Motor 3 PWM | ESC 3 Signal | Rear Right |
| GPIO 15 | Motor 4 PWM | ESC 4 Signal | Rear Left |
| **Communication** | | | |
| GPIO 16 | GPS RX | GPS TX | Serial data from GPS |
| GPIO 17 | GPS TX | GPS RX | Serial data to GPS |
| GPIO 18 | LoRa SCK | LoRa SCK | SPI clock |
| GPIO 19 | LoRa MISO | LoRa MISO | SPI data in |
| GPIO 23 | LoRa MOSI | LoRa MOSI | SPI data out |
| GPIO 5 | LoRa CS | LoRa CS | SPI chip select |
| GPIO 4 | LoRa RST | LoRa RST | Reset pin |
| GPIO 2 | LoRa DIO0 | LoRa DIO0 | Interrupt |
| GPIO 21 | I2C SDA | ESP32-CAM SDA | Inter-ESP communication |
| GPIO 22 | I2C SCL | ESP32-CAM SCL | Inter-ESP clock |
| **Sensors & Power** | | | |
| GPIO 34 | Battery ADC | PDB Voltage Divider | Analog input |
| GPIO 35 | Current ADC | Current Sensor | Analog input |
| GPIO 25 | LED Status | Status LED | Digital output |
| GPIO 26 | Buzzer | Piezo Buzzer | PWM output |
| **Reserved** | | | |
| GPIO 0 | Reserved | - | Boot pin |
| GPIO 1 | Reserved | USB TX | Serial debug |
| GPIO 3 | Reserved | USB RX | Serial debug |
| GPIO 27 | Reserved | Future IMU | I2C SDA |
| GPIO 32 | Reserved | Future IMU | I2C SCL |
| GPIO 33 | Reserved | Future IMU | Interrupt |

## 2. ESP32-CAM Pin Mapping

### ESP32-CAM Pin Assignments

```
ESP32-CAM Pinout (Bottom View):
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│  GND ○ 3.3V ○ 5V ○ GND ○ GND ○              ○ GND ○ 5V ○   │
│                                                             │
│  U0R ○ U0T ○ GPIO16 ○ GPIO0 ○ GPIO2 ○ GPIO4 ○ GPIO15 ○ GPIO13 │
│                                                             │
│  GPIO12 ○ GPIO14 ○ GPIO3 ○ GPIO1 ○ GPIO5 ○ GPIO17 ○ GPIO18 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### ESP32-CAM Pin Configuration

| ESP32 Pin | Function | Connected To | Notes |
|-----------|----------|--------------|-------|
| **Camera Interface** | | | |
| GPIO 0 | Camera PWDN | OV2640 PWDN | Power down control |
| GPIO 32 | Camera RESET | OV2640 RESET | Camera reset |
| GPIO 4 | Camera XCLK | OV2640 XCLK | External clock |
| GPIO 18 | Camera PCLK | OV2640 PCLK | Pixel clock |
| GPIO 33 | Camera VSYNC | OV2640 VSYNC | Vertical sync |
| GPIO 19 | Camera HREF | OV2640 HREF | Horizontal reference |
| GPIO 36 | Camera D0 | OV2640 D0 | Data bit 0 |
| GPIO 39 | Camera D1 | OV2640 D1 | Data bit 1 |
| GPIO 34 | Camera D2 | OV2640 D2 | Data bit 2 |
| GPIO 35 | Camera D3 | OV2640 D3 | Data bit 3 |
| GPIO 25 | Camera D4 | OV2640 D4 | Data bit 4 |
| GPIO 26 | Camera D5 | OV2640 D5 | Data bit 5 |
| GPIO 27 | Camera D6 | OV2640 D6 | Data bit 6 |
| GPIO 14 | Camera D7 | OV2640 D7 | Data bit 7 |
| **Communication** | | | |
| GPIO 12 | I2C SDA | ESP32-FC SDA | Data to flight controller |
| GPIO 13 | I2C SCL | ESP32-FC SCL | Clock for I2C |
| GPIO 15 | Ready Signal | ESP32-FC GPIO 0 | AI processing complete |
| **Power & Control** | | | |
| 5V | Power Input | PDB 5V | Main power |
| 3.3V | Regulated | Internal regulator | Logic power |
| GND | Ground | Common ground | System ground |

## 3. Power Distribution Board (PDB) Schematic

### PDB Block Diagram

```
┌─────────────────┐
│   XT60 Input    │ ← Battery (11.1V)
│   (11.1V)       │
└─────────┬───────┘
          │
          ├─────────────────┬─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
┌─────────┴───────┐ ┌───────┴─────────┐ ┌─────┴──────────┐
│  5V Regulator   │ │ 3.3V Regulator  │ │ ESC Power      │
│  (5V @ 3A)      │ │ (3.3V @ 1A)     │ │ (11.1V)        │
└─────────┬───────┘ └─────────┬───────┘ └─────────┬──────┘
          │                   │                   │
          ├─────────┬─────────┼─────────┬─────────┼─────────┐
          │         │         │         │         │         │
          ▼         ▼         ▼         ▼         ▼         ▼
     ESP32-CAM  Status   ESP32-FC   GPS     LoRa    ESC 1  ESC 2
       (5V)      LED     (3.3V)    (3.3V)  (3.3V)  (11.1V) (11.1V)
```

### PDB Detailed Schematic

```
Battery Input (XT60):
+ ────────────────────────────────────────────────────────────────── +11.1V
- ────────────────────────────────────────────────────────────────── GND

Voltage Divider (for battery monitoring):
+11.1V ── 10kΩ ── ADC ── 10kΩ ── GND

5V Regulator (AMS1117-5.0):
+11.1V ── IN ── AMS1117-5.0 ── OUT ── +5V
          │                        │
          ├── GND ── GND           ├── GND (to ESP32-CAM)
          └── C1: 10μF             └── C2: 10μF

3.3V Regulator (AMS1117-3.3):
+11.1V ── IN ── AMS1117-3.3 ── OUT ── +3.3V
          │                        │
          ├── GND ── GND           ├── GND (to ESP32-FC, GPS, LoRa)
          └── C1: 10μF             └── C2: 10μF

ESC Power Distribution:
+11.1V ── Fuse (30A) ── ESC1 (+)
+11.1V ── Fuse (30A) ── ESC2 (+)
+11.1V ── Fuse (30A) ── ESC3 (+)
+11.1V ── Fuse (30A) ── ESC4 (+)

Signal Connections:
ESP32 GPIO 12 ── ESC1 Signal
ESP32 GPIO 13 ── ESC2 Signal
ESP32 GPIO 14 ── ESC3 Signal
ESP32 GPIO 15 ── ESC4 Signal

All GND connections are connected to common ground plane
```

## 4. Motor and ESC Wiring

### Quadcopter Motor Configuration

```
Motor Layout (Top View):
    Front
   Motor 2 (CW)     Motor 1 (CCW)
      ↗                   ↙
       \                 /
        \   Battery    /
         \           /
          \       /
           \   /
            X
           / \
          /   \
         /     \
        /       \
       /         \
      ↙           ↘
   Motor 3 (CCW)     Motor 4 (CW)
     Rear
```

### ESC Connection Details

```
ESC Pinout (Standard):
┌─────────────────────┐
│  ○ ○ ○              │
│  Signal  +  -       │
│                     │
│  Black   Red  White │  (from battery side)
└─────────────────────┘

ESC to Motor Wiring:
ESC (+) ── Motor Phase A
ESC (-) ── Motor Phase B
ESC (Signal) ── Motor Phase C

ESC to PDB Wiring:
ESC (+) ── PDB (+11.1V)
ESC (-) ── PDB (GND)
ESC (Signal) ── ESP32 PWM Pin

ESC to Battery:
ESC (+) ── Battery (+)
ESC (-) ── Battery (-)
```

### Propeller Installation

```
Motor Rotation and Propeller Direction:

Clockwise (CW) Motors:      Counter-Clockwise (CCW) Motors:
Motor 2 & Motor 4           Motor 1 & Motor 3

Propeller Installation:
- CW Motors: Normal rotation propellers
- CCW Motors: Reverse rotation propellers (R)

Torque Values:
- M3 Motor Mount Screws: 1.0-1.2 Nm
- Propeller Adapter: 0.8-1.0 Nm
- Propeller to Adapter: Hand tight + thread lock
```

## 5. Communication Module Wiring

### LoRa SX1278 Module Connections

```
SX1278 Pinout:
┌─────────────────────────────────┐
│ ANT ○ GND ○ 3.3V ○ DIO3 ○ DIO4  │
│                                 │
│ DIO2 ○ DIO1 ○ DIO0 ○ RST ○ NSS  │
│                                 │
│ MOSI ○ MISO ○ SCK ○ GND ○ 3.3V │
└─────────────────────────────────┘

ESP32 to SX1278 Wiring:
ESP32 GPIO 18 ── SX1278 SCK
ESP32 GPIO 19 ── SX1278 MISO
ESP32 GPIO 23 ── SX1278 MOSI
ESP32 GPIO 5  ── SX1278 NSS (CS)
ESP32 GPIO 4  ── SX1278 RST
ESP32 GPIO 2  ── SX1278 DIO0
ESP32 3.3V    ── SX1278 3.3V
ESP32 GND     ── SX1278 GND

Antenna Connection:
SX1278 ANT ── 433MHz Antenna (SMA)
```

### GPS NEO-6M Module Connections

```
NEO-6M Pinout:
┌─────────────────────────────┐
│ VCC ○ GND ○ TX ○ RX ○ PPS  │
│                             │
│ ○ ○ ○ ○ ○                   │
└─────────────────────────────┘

ESP32 to NEO-6M Wiring:
ESP32 GPIO 16 ── NEO-6M TX
ESP32 GPIO 17 ── NEO-6M RX
ESP32 3.3V    ── NEO-6M VCC
ESP32 GND     ── NEO-6M GND

Optional Connections:
NEO-6M PPS ── ESP32 GPIO 36 (Pulse per second timing)
```

### Inter-ESP32 Communication (I2C)

```
ESP32-FC (Master) to ESP32-CAM (Slave):

ESP32-FC GPIO 21 ── ESP32-CAM GPIO 12 (SDA)
ESP32-FC GPIO 22 ── ESP32-CAM GPIO 13 (SCL)
ESP32-FC GND     ── ESP32-CAM GND
ESP32-FC 3.3V    ── ESP32-CAM 3.3V (optional, separate power better)

I2C Pull-up Resistors (4.7kΩ):
ESP32-FC SDA ── 4.7kΩ ── 3.3V
ESP32-FC SCL ── 4.7kΩ ── 3.3V
```

## 6. Complete System Wiring Diagram

### ASCII Art System Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    DRONE SYSTEM WIRING                          │
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐           │
│  │   Battery   │    │     PDB     │    │   ESP32-FC  │           │
│  │  11.1V/2.2A │    │             │    │             │           │
│  │  XT60 Conn  │    │ ┌─────────┐ │    │ ┌─────────┐ │           │
│  │             │    │ │5V Reg   │ │    │ │Motor PWM│ │           │
│  │  +     -    │    │ │3.3V Reg │ │    │ │GPIO12-15│ │           │
│  └─────┬───────┘    │ └─────────┘ │    │ └─────────┘ │           │
│        │            │      │      │    │      │      │           │
│        │            └──────┼──────┼────┼──────┼──────┘           │
│        │                   │      │      │      │                 │
│        ▼                   ▼      ▼      ▼      ▼                 │
│  ┌─────┴─────┐   ┌─────┴─┐  ┌─┴────┐   ┌┴─────┐  ┌─────────────┐ │
│  │ESC1 Signal│   │ESP32-│  │ GPS  │   │ LoRa  │  │   Motors     │ │
│  │GPIO12     │   │CAM   │  │NEO-6M│   │SX1278 │  │             │ │
│  └───────────┘   └──────┘  └──────┘   └──────┘  └─────────────┘ │
│                                                                 │
│  I2C Bus: ESP32-FC ↔ ESP32-CAM (GPIO21/22 ↔ GPIO12/13)         │
│  SPI Bus: ESP32-FC → LoRa (GPIO18/19/23/5)                      │
│  UART: ESP32-FC ↔ GPS (GPIO16/17)                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Detailed Connection Summary

#### Power Connections
```
Battery (+) ── PDB Input (+)
Battery (-) ── PDB Input (-)
PDB (+) ── All ESC Power Inputs
PDB 5V ── ESP32-CAM Power
PDB 3.3V ── ESP32-FC, GPS, LoRa Power
PDB GND ── All Components Ground
```

#### Signal Connections
```
ESP32-FC PWM Pins ── ESC Signal Pins
ESP32-FC UART ── GPS TX/RX
ESP32-FC SPI ── LoRa SPI Pins
ESP32-FC I2C ── ESP32-CAM I2C Pins
ESP32-FC ADC ── PDB Voltage Divider
```

#### Communication Connections
```
ESP32-FC GPIO 21 (SDA) ── ESP32-CAM GPIO 12 (SDA)
ESP32-FC GPIO 22 (SCL) ── ESP32-CAM GPIO 13 (SCL)
ESP32-FC GPIO 16 (RX) ── GPS TX
ESP32-FC GPIO 17 (TX) ── GPS RX
ESP32-FC GPIO 18 (SCK) ── LoRa SCK
ESP32-FC GPIO 19 (MISO) ── LoRa MISO
ESP32-FC GPIO 23 (MOSI) ── LoRa MOSI
ESP32-FC GPIO 5 (CS) ── LoRa NSS
ESP32-FC GPIO 4 (RST) ── LoRa RST
ESP32-FC GPIO 2 (DIO0) ── LoRa DIO0
```

## 7. Wiring Best Practices

### Cable Management
- Use zip ties to secure cables to frame
- Avoid sharp bends in wires
- Keep signal wires away from power wires
- Use cable sleeving for professional appearance

### Soldering Guidelines
- Use 60/40 rosin core solder
- Pre-tin all connections
- Use heat shrink tubing for insulation
- Test continuity after soldering

### Power Wiring
- Use appropriate wire gauge (16-18 AWG for power)
- Crimp connectors properly
- Use bullet connectors for ESC/motor connections
- Balance wire lengths for even resistance

### Signal Integrity
- Keep PWM wires short (<30cm)
- Use twisted pair for I2C connections
- Ground signal returns properly
- Avoid running signal wires parallel to power wires

### Safety Considerations
- Double-check all connections before power-on
- Use fuses on power distribution
- Implement proper grounding
- Test voltage levels before connecting sensitive components

### Testing Procedure
1. **Power Test**: Verify PDB outputs correct voltages
2. **Continuity Test**: Check all signal connections
3. **Motor Test**: Verify motor rotation directions
4. **Sensor Test**: Confirm GPS and LoRa communication
5. **Integration Test**: Full system power-on test

This comprehensive wiring guide provides all the necessary information to properly connect and integrate the wildfire surveillance drone system components.
