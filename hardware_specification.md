# 📋 Hardware Specification & Selection Guide

## Detailed Component Specifications

### 1. Microcontroller Units

#### ESP32-WROOM-32 (Flight Controller)
**Recommended Model**: ESP32-DevKitC V4
- **MCU**: Xtensa dual-core 32-bit LX6 microprocessor
- **CPU Frequency**: Up to 240MHz
- **SRAM**: 520KB
- **Flash**: External 4MB (upgradeable)
- **WiFi**: 802.11 b/g/n (2.4GHz)
- **Bluetooth**: v4.2 BR/EDR and BLE
- **GPIO**: 38 programmable pins
- **ADC**: 18 channels (12-bit resolution)
- **DAC**: 2 channels (8-bit resolution)
- **UART**: 3 interfaces
- **SPI**: 4 interfaces
- **I2C**: 2 interfaces
- **PWM**: 16 channels
- **Operating Voltage**: 2.2V - 3.6V
- **Operating Current**: 80mA (average), 500mA (peak)
- **Dimensions**: 52.3mm x 28.3mm
- **Weight**: 8g

**Justification**:
- Powerful enough for real-time flight control algorithms
- Extensive peripheral support for sensors and communication
- Low power consumption suitable for battery operation
- Mature ecosystem with excellent Arduino support
- Integrated WiFi/Bluetooth for future expansions

**Suppliers**:
- Amazon: ESP32-DevKitC (~$8)
- AliExpress: ESP32 Development Board (~$5)
- Digi-Key: ESP32-DevKitC-V4 (~$12)

#### ESP32-CAM (AI Vision Module)
**Recommended Model**: AI-Thinker ESP32-CAM
- **MCU**: Xtensa single-core 32-bit LX6 microprocessor
- **CPU Frequency**: Up to 240MHz
- **SRAM**: 320KB
- **PSRAM**: 4MB (external)
- **Flash**: 4MB (onboard)
- **Camera**: OV2640 CMOS sensor
- **Resolution**: 1600x1200 (UXGA), 800x600 (SVGA), 640x480 (VGA)
- **Video Formats**: YUV422, YUV420, RGB565, JPEG
- **Frame Rate**: 15fps at UXGA, 60fps at QVGA
- **Lens**: 1/4" fixed focus (adjustable)
- **Field of View**: 65° diagonal
- **Interface**: 8-bit DVP parallel interface
- **GPIO**: 9 programmable pins (limited due to camera)
- **Operating Voltage**: 3.0V - 3.6V
- **Operating Current**: 150mA (active), 10μA (deep sleep)
- **Dimensions**: 27mm x 40.5mm
- **Weight**: 5g

**Justification**:
- Integrated camera for computer vision applications
- Sufficient processing power for TinyML inference
- PSRAM for frame buffering during AI processing
- Compact size ideal for drone integration
- Compatible with ESP32 flight controller for easy communication

**Suppliers**:
- Amazon: ESP32-CAM with OV2640 (~$7)
- AliExpress: AI-Thinker ESP32-CAM (~$5)
- Banggood: ESP32-CAM Development Board (~$6)

### 2. Communication Modules

#### LoRa Transceiver Module
**Recommended Model**: SX1278 433MHz Ra-02 Module
- **Chipset**: Semtech SX1278
- **Frequency Range**: 410-525MHz (433MHz version)
- **Modulation**: LoRa, FSK, GFSK, MSK, GMSK, OOK
- **Max Output Power**: +20dBm (100mW)
- **Sensitivity**: -148dBm (LoRa mode)
- **Data Rate**: 0.018-37.5kbps (LoRa), up to 300kbps (FSK)
- **Range**: Up to 10-20km (line of sight)
- **Interface**: SPI
- **Operating Voltage**: 1.8-3.7V
- **Operating Current**: 10mA (receive), 120mA (transmit @20dBm)
- **Antenna**: SMA connector (50Ω impedance)
- **Dimensions**: 17mm x 16mm
- **Weight**: 2g

**Alternative Options**:
- **RFM95W**: Similar specs, slightly different pinout
- **SX1276**: 868/915MHz versions for different regions

**Justification**:
- Excellent range for remote wildfire monitoring
- Low power consumption ideal for battery operation
- Robust in harsh environments (mountains, forests)
- Mature technology with extensive documentation
- Cost-effective compared to cellular alternatives

**Suppliers**:
- Amazon: SX1278 LoRa Module 433MHz (~$5)
- AliExpress: Ra-02 SX1278 LoRa Module (~$3)
- Digi-Key: SX1278IMLTRT (~$6)

#### GPS Module
**Recommended Model**: NEO-6M GPS Module
- **Chipset**: u-blox NEO-6M
- **Channels**: 50 tracking, 22 simultaneous
- **Sensitivity**: -161dBm (tracking), -148dBm (acquisition)
- **Accuracy**: 2.5m CEP (Circular Error Probable)
- **Time to First Fix**: Cold start: 27s, Hot start: 1s
- **Update Rate**: Default 1Hz, configurable up to 5Hz
- **Protocol**: NMEA 0183 v3.01
- **Interface**: UART (TTL level)
- **Baud Rate**: 4800-230400 bps
- **Antenna**: Active ceramic antenna (25x25mm)
- **Operating Voltage**: 2.7-3.6V
- **Operating Current**: 45mA (tracking), 25mA ( standby)
- **Backup Power**: For hot start capability
- **Dimensions**: 25mm x 35mm (module), 35mm x 35mm (with antenna)
- **Weight**: 8g

**Justification**:
- Excellent performance for drone navigation
- Reliable positioning in mountainous terrain
- Low power consumption
- Industry-standard NMEA protocol
- Good balance of cost and performance

**Suppliers**:
- Amazon: NEO-6M GPS Module (~$10)
- AliExpress: GY-NEO6MV2 GPS Module (~$8)
- SparkFun: GPS-15210 (~$15)

### 3. Power System Components

#### LiPo Battery
**Recommended Model**: 3S 2200mAh 25C LiPo
- **Configuration**: 3 cells in series
- **Capacity**: 2200mAh (2.2Ah)
- **Voltage**: 11.1V nominal (12.6V fully charged, 9.0V discharged)
- **Discharge Rate**: 25C continuous (55A max)
- **Energy Capacity**: 24.42Wh
- **Weight**: 180g
- **Dimensions**: 105mm x 35mm x 20mm
- **Connector**: XT60 (standard drone connector)
- **Balance Connector**: JST-XH 4-pin
- **Operating Temperature**: -10°C to 60°C
- **Storage Temperature**: -20°C to 35°C
- **Cycle Life**: 300-500 cycles

**Alternative Options**:
- **4S 1500mAh**: Higher voltage (14.8V), shorter flight time
- **3S 3000mAh**: Longer flight time, heavier weight

**Justification**:
- Optimal balance of flight time and weight
- Sufficient discharge rate for quadcopter propulsion
- Widely available and cost-effective
- Good energy density for the application

**Suppliers**:
- Amazon: 3S 2200mAh LiPo Battery (~$15)
- HobbyKing: Turnigy 2200mAh 3S (~$12)
- Banggood: 3S 2200mAh LiPo Pack (~$13)

#### Power Distribution Board
**Recommended Model**: Custom design or generic PDB
- **Input Voltage**: 7-25V (supports 2S-6S LiPo)
- **Output Channels**: 4x ESC power, 2x 5V@3A, 2x 3.3V@1A
- **Current Capacity**: 30A continuous
- **Voltage Regulation**: ±5% for regulated outputs
- **Protection**: Over-current, short-circuit, reverse polarity
- **Monitoring**: Battery voltage ADC input
- **Connectors**: XT60 input, servo connectors for ESCs
- **Dimensions**: 50mm x 50mm
- **Weight**: 15g

**Justification**:
- Clean power distribution to sensitive electronics
- Voltage monitoring for low battery detection
- Protection circuits for system safety
- Compact size suitable for small drones

**Suppliers**:
- Custom PCB: JLCPCB (~$5 for 5 boards)
- Amazon: Quadcopter Power Distribution Board (~$8)
- AliExpress: KK PDB Board (~$6)

### 4. Propulsion System

#### Brushless Motors
**Recommended Model**: 2205 2300KV Brushless Motor
- **Size**: 2205 (22mm diameter, 5mm shaft length)
- **KV Rating**: 2300 RPM/Volt
- **Max Power**: 180W
- **Max Current**: 15A
- **Internal Resistance**: 0.08Ω
- **Efficiency**: >75%
- **Weight**: 28g
- **Shaft Diameter**: 3mm
- **Mounting Pattern**: 16mm M3 holes
- **Operating Voltage**: 7-12V (2S-3S LiPo)
- **Operating Temperature**: -20°C to 50°C

**Justification**:
- High power-to-weight ratio for agile flight
- Suitable for 250mm frame size
- Good efficiency for extended flight time
- Widely used in drone community with good support

**Suppliers**:
- Amazon: 2205 2300KV Brushless Motor (~$10 each)
- AliExpress: Emax 2205 2300KV (~$8 each)
- Banggood: 2205 Brushless Motor Set (~$35 for 4)

#### Electronic Speed Controllers
**Recommended Model**: 20A BLHeli_32 ESC
- **Current Rating**: 20A continuous, 25A burst
- **Voltage Range**: 2-4S LiPo (7-16V)
- **Protocol**: DShot600, DShot300, Multishot, Oneshot125
- **BEC**: None (for separate PDB)
- **Firmware**: BLHeli_32 (32-bit processor)
- **Weight**: 8g each
- **Dimensions**: 25mm x 25mm x 6mm
- **Connector**: 3.5mm bullet connectors
- **Timing**: Auto-timing
- **Active Braking**: Yes
- **Throttle Range**: 1000-2000μs

**Justification**:
- Sufficient current capacity for 2205 motors
- Advanced 32-bit processor for better performance
- Modern protocols for precise motor control
- Lightweight and compact design

**Suppliers**:
- Amazon: 20A BLHeli_32 ESC (~$10 each)
- AliExpress: 20A 32-bit ESC (~$8 each)
- Banggood: BLHeli_32 ESC 20A (~$9 each)

#### Propellers
**Recommended Model**: 5045 Polycarbonate Propellers
- **Size**: 5x4.5 inch (127x114mm)
- **Pitch**: 4.5 inch
- **Material**: Polycarbonate (PC)
- **Weight**: 3.5g per propeller
- **Color**: Black
- **Hub**: 5mm (M5 thread)
- **Balance**: Factory balanced
- **Thrust**: ~400g per propeller at 10,000 RPM

**Alternative Options**:
- **5040**: Lower pitch for efficiency
- **5050**: Higher pitch for speed
- **Carbon Fiber**: Lighter but more expensive

**Justification**:
- Good balance of thrust and efficiency
- Durable polycarbonate material
- Lightweight for better flight performance
- Standard size for 2205 motors

**Suppliers**:
- Amazon: 5045 PC Propellers (~$2.50 for 4 pairs)
- AliExpress: 5045 Propeller Set (~$2 for 4 pairs)
- HobbyKing: APC 5x4.5 Propellers (~$3 for 4 pairs)

### 5. Frame and Mechanical Components

#### Quadcopter Frame
**Recommended Model**: 250mm Carbon Fiber Frame
- **Wheelbase**: 250mm (motor-to-motor diagonal)
- **Material**: 3K Carbon Fiber
- **Thickness**: 2mm top/bottom plates, 1.5mm arms
- **Weight**: 120g
- **Mounting**: M3 threaded inserts
- **Compatibility**: Standard 2205 mount pattern
- **Landing Gear**: Integrated 4-point landing gear
- **Camera Mount**: Standard 1/4-20 thread

**Justification**:
- Lightweight carbon fiber construction
- Good strength-to-weight ratio
- Standard size for balanced performance
- Durable for outdoor operation

**Suppliers**:
- Amazon: 250mm Carbon Fiber Frame (~$20)
- AliExpress: 250mm CF Quadcopter Frame (~$15)
- Banggood: 250mm Carbon Frame Kit (~$18)

### 6. Additional Components

#### Antennas
**LoRa Antenna**:
- **Type**: 433MHz Quarter-wave whip antenna
- **Gain**: 2dBi
- **Impedance**: 50Ω
- **Connector**: SMA male
- **Length**: 165mm
- **Weight**: 5g

**GPS Antenna**:
- **Type**: Active GPS antenna with LNA
- **Frequency**: 1575.42MHz (GPS L1)
- **Gain**: 28dB
- **Voltage**: 3-5V
- **Connector**: SMA male
- **Cable Length**: 3m
- **Weight**: 15g

#### Connectors and Cables
- **XT60 Connectors**: Battery connections
- **Servo Connectors**: ESC connections
- **JST Connectors**: Sensor connections
- **Dupont Wires**: Prototyping connections

#### Tools and Accessories
- **Soldering Iron**: 30W temperature-controlled
- **Multimeter**: For voltage/current measurement
- **ESC Programmer**: For BLHeli configuration
- **Battery Charger**: iMAX B6 or similar
- **LiPo Safe Bag**: For storage and transport
- **Zip Ties**: For cable management

## Component Integration Considerations

### Thermal Management
- ESP32 modules: Operate within 0-50°C range
- LiPo Battery: Monitor temperature during discharge
- Motors: Adequate cooling through propeller airflow
- Power Regulator: Heat sink if operating near max current

### Vibration Isolation
- GPS module: Mount on vibration-damping foam
- ESP32-CAM: Isolate from frame vibrations
- LoRa module: Position away from power lines

### Electromagnetic Compatibility
- Separate power and signal grounds where possible
- Route high-current wires away from sensitive signals
- Use ferrite beads on power lines if interference occurs
- Shield antenna cables if necessary

### Weight Distribution
- Balance weight distribution for stable flight
- Position heavier components (battery) near center
- Consider payload weight in center of gravity calculations

## Procurement Strategy

### Primary Suppliers
1. **Amazon**: Fast shipping, reliable quality, good returns
2. **AliExpress**: Lowest cost, longer shipping times
3. **Banggood**: Good balance of cost and speed

### Quality Considerations
- Purchase from reputable sellers with good reviews
- Check component specifications against requirements
- Verify pin compatibility before ordering
- Consider counterfeit risks on very cheap components

### Backup Components
- Order 10-20% extra of critical components
- Have backup microcontrollers available
- Stock extra propellers and batteries for testing

### Cost Optimization
- Buy in bulk for better pricing (motors, ESCs, propellers)
- Consider kit options where available
- Source locally when possible to reduce shipping costs

## Testing and Validation Equipment

### Basic Testing Tools
- **Multimeter**: Voltage, current, resistance measurement
- **Oscilloscope**: Signal analysis (optional but useful)
- **Logic Analyzer**: Debug digital signals
- **USB-to-Serial Adapter**: Program ESP32 modules

### Flight Testing Equipment
- **RC Transmitter**: For manual control during testing
- **FPV Goggles**: For visual line-of-sight operation
- **Telemetry Module**: Monitor flight data in real-time

### Software Tools
- **Arduino IDE**: ESP32 programming
- **PlatformIO**: Advanced development environment
- **Python**: AI model development and ground station
- **QGroundControl**: Flight planning and monitoring

This detailed hardware specification provides everything needed to source, integrate, and test the wildfire surveillance drone system. The component selections balance performance, cost, and reliability for the intended application.
