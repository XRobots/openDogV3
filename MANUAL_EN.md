# OpenDog V3 Quadruped Robot - Program Manual

## Table of Contents
1. [Overview](#overview)
2. [System Requirements](#system-requirements)
3. [Hardware Configuration](#hardware-configuration)
4. [Software Installation](#software-installation)
5. [Configuration and Calibration](#configuration-and-calibration)
6. [Robot Operation](#robot-operation)
7. [Remote Controller](#remote-controller)
8. [Testing and Diagnostics](#testing-and-diagnostics)
9. [Troubleshooting](#troubleshooting)
10. [API Reference](#api-reference)

---

## Overview

OpenDog V3 is a quadruped walking robot with a complete Python implementation converted from Arduino C++ code, providing better extensibility and maintainability. The system includes the following key features:

- **Real-time Inverse Kinematics Control** - 100Hz control loop
- **IMU-based Stability Control** - Gyroscope and accelerometer feedback
- **nRF24L01 Wireless Communication** - Remote controller communication
- **Modular Architecture** - Independent components
- **Comprehensive Test Suite** - Automated testing and validation

### Main Components

- **Main Controller** (`opendog_v3_controller.py`) - Core robot control system
- **Inverse Kinematics Module** (`kinematics.py`) - Leg position calculations
- **Motor Control** (`odrive_interface.py`) - ODrive motor controller interface
- **IMU Interface** (`imu_interface.py`) - Inertial measurement unit control
- **Input Processing** (`input_processing.py`) - User input and gait patterns
- **Remote Controller** (`remote_controller.py`) - Wireless remote control
- **Configuration Management** (`config.py`) - System parameter management

---

## System Requirements

### Hardware Requirements

#### Main Computer (Robot Onboard)
- **Raspberry Pi 4 Model B** (4GB+ recommended)
- **microSD Card** 64GB+ (Class 10)
- **Power Supply** 5V 3A+

#### Remote Controller
- **Raspberry Pi Zero W** or Pi 4
- **nRF24L01 Wireless Module**
- **20x4 LCD Display** (I2C)
- **Joysticks and Buttons** (Analog/Digital inputs)

#### Robot Hardware
- **ODrive Motor Controllers** v3.6 (2x)
- **Brushless DC Motors** 8x (2 per leg)
- **IMU Sensor** (MPU6050 or compatible)
- **nRF24L01 Wireless Module**
- **Power System** (LiPo battery recommended)

### Software Requirements

#### Operating System
- **Raspberry Pi OS** (Bullseye or later)
- **Python 3.8+**

#### Required Python Packages
```
numpy>=1.21.0
odrive>=0.6.0
pyserial>=3.5
RPi.GPIO>=0.7.1
smbus2>=0.4.1
spidev>=3.5
nrf24>=0.1.0
```

---

## Hardware Configuration

### Pin Layout

#### Main Controller (Raspberry Pi 4)

##### GPIO Pin Assignment
```
Pin  | Function        | Description
-----|-----------------|----------------------------------
2    | SDA (I2C)      | IMU communication
3    | SCL (I2C)      | IMU communication
8    | CE0 (SPI)      | nRF24L01 CSN
10   | MOSI (SPI)     | nRF24L01 MOSI
9    | MISO (SPI)     | nRF24L01 MISO
11   | SCLK (SPI)     | nRF24L01 SCK
18   | nRF24 CE       | nRF24L01 CE
24   | nRF24 IRQ      | nRF24L01 IRQ (optional)
```

##### UART Connections
```
UART 0: ODrive 1 (GPIO 14, 15)
UART 1: ODrive 2 (GPIO 0, 1)
```

#### Remote Controller Pin Layout

##### Digital Inputs
```
Pin  | Function       | Description
-----|----------------|----------------------------------
2    | Menu Down      | Menu down button
3    | Menu Up        | Menu up button
0    | Toggle 2       | Toggle button 2
1    | Select         | Select button
4    | Motor Enable   | Motor enable switch
5    | IMU Enable     | IMU enable switch
6    | Toggle 1       | Toggle button 1
7    | Walk Reverse   | Reverse walking switch
```

##### Analog Inputs (ADC required)
```
Channel | Function          | Description
--------|-------------------|----------------------------------
A1      | Left Trigger      | Left trigger
A2      | Left L/R Stick    | Left stick left/right
A3      | Left F/B Stick    | Left stick forward/back
A4      | Right L/R Stick   | Right stick left/right
A5      | Right F/B Stick   | Right stick forward/back
A6      | Right Trigger     | Right trigger
```

### Wiring Diagram

#### Main System Wiring
```
[Raspberry Pi 4] ---- UART ---- [ODrive 1]
                |                    |
                ---- UART ---- [ODrive 2]
                |
                ---- I2C ----- [IMU (MPU6050)]
                |
                ---- SPI ----- [nRF24L01]
```

#### Power Distribution
```
[Battery Pack] ---- [Power Distribution Board]
                              |
                              |-- 24V --> ODrive Controllers
                              |-- 12V --> Motor Power
                              |-- 5V  --> Raspberry Pi
                              |-- 3.3V -> Sensors
```

---

## Software Installation

### 1. Basic System Setup

#### Raspberry Pi OS Installation
```bash
# Flash Raspberry Pi OS image to SD card
# Enable SSH, I2C, and SPI
sudo raspi-config
```

#### System Update
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3-pip git
```

### 2. Python Environment Setup

#### Create Virtual Environment
```bash
cd /home/pi
python3 -m venv opendog_env
source opendog_env/bin/activate
```

#### Install Required Packages
```bash
pip install numpy
pip install odrive
pip install pyserial
pip install RPi.GPIO
pip install smbus2
pip install spidev
```

#### Install nRF24L01 Library
```bash
# CircuitPython nRF24L01 library
pip install circuitpython-nrf24l01
```

### 3. Project Download

```bash
git clone <repository-url> opendog_v3
cd opendog_v3
```

### 4. Permission Setup

```bash
# GPIO access permissions
sudo usermod -a -G gpio pi
sudo usermod -a -G spi pi
sudo usermod -a -G i2c pi

# Serial port permissions
sudo usermod -a -G dialout pi
```

### 5. System Service Setup (Optional)

#### Create systemd Service
```bash
sudo nano /etc/systemd/system/opendog.service
```

```ini
[Unit]
Description=OpenDog V3 Controller
After=multi-user.target

[Service]
Type=idle
User=pi
ExecStart=/home/pi/opendog_env/bin/python /home/pi/opendog_v3/Code/openDogV3_experimental_stability/opendog_v3_controller.py
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable opendog.service
```

---

## Configuration and Calibration

### 1. Hardware Verification

#### Run System Diagnostics
```bash
cd Code/openDogV3_experimental_stability
python3 hardware_setup.py
```

This script verifies:
- ODrive controller connections
- IMU sensor communication
- nRF24L01 wireless module
- GPIO pin status
- System resources

#### Expected Output
```
=== OpenDog V3 Hardware Setup ===
✓ ODrive 1 detected on /dev/ttyACM0
✓ ODrive 2 detected on /dev/ttyACM1
✓ IMU detected on I2C address 0x68
✓ nRF24L01 radio initialized
✓ GPIO pins configured
✓ System resources OK

Hardware setup complete!
```

### 2. ODrive Motor Controller Setup

#### Automatic Configuration
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.configure_odrives()
"
```

#### Manual Configuration (if needed)
```bash
# Run ODrive configuration tool
odrivetool
```

Commands to run in ODrivetool:
```python
# Basic configuration
odrv0.axis0.motor.config.pole_pairs = 14
odrv0.axis0.motor.config.resistance_calib_max_voltage = 2
odrv0.axis0.motor.config.requested_current_range = 25
odrv0.axis0.motor.config.current_control_bandwidth = 100

# Encoder configuration
odrv0.axis0.encoder.config.cpr = 8192

# Controller configuration
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02
odrv0.axis0.controller.config.vel_integrator_gain = 0.1

# Save configuration
odrv0.save_configuration()
odrv0.reboot()
```

### 3. IMU Calibration

#### Automatic Calibration
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"
```

#### Calibration Process
1. **Place robot on flat surface**
2. **Static calibration** (5 seconds stationary)
3. **Calculate and save offsets**
4. **Verification test**

### 4. Wireless Communication Setup

#### Check nRF24L01 Configuration
```bash
python3 -c "
from remote_controller import RadioInterface
radio = RadioInterface(simulation_mode=False)
print('Radio initialized successfully')
"
```

### 5. System Configuration File Editing

#### Check and Modify config.py
```bash
nano config.py
```

Key configuration items:
```python
# Hardware configuration
ODRIVE_SERIAL_NUMBERS = ['serial1', 'serial2']
IMU_I2C_ADDRESS = 0x68
RADIO_CHANNEL = 96

# Control parameters
CONTROL_FREQUENCY = 100  # Hz
SAFETY_TIMEOUT = 1.0     # seconds

# Kinematics parameters
LEG_LENGTH_1 = 110  # mm
LEG_LENGTH_2 = 120  # mm
BODY_WIDTH = 160    # mm
BODY_LENGTH = 280   # mm

# Motor limits
MAX_MOTOR_CURRENT = 20  # A
MAX_MOTOR_VELOCITY = 50 # rad/s
```

---

## Robot Operation

### 1. Basic Startup Procedure

#### System Startup
```bash
# Activate virtual environment
source /home/pi/opendog_env/bin/activate

# Navigate to main directory
cd /home/pi/opendog_v3/Code/openDogV3_experimental_stability

# Start robot controller
python3 opendog_v3_controller.py
```

#### Expected Startup Log
```
2025-05-30 10:00:00 - OpenDogController - INFO - Initializing OpenDog V3 Controller
2025-05-30 10:00:01 - ODriveInterface - INFO - ODrive 1 connected on /dev/ttyACM0
2025-05-30 10:00:01 - ODriveInterface - INFO - ODrive 2 connected on /dev/ttyACM1
2025-05-30 10:00:02 - IMUInterface - INFO - IMU initialized successfully
2025-05-30 10:00:02 - RadioInterface - INFO - nRF24L01 radio initialized
2025-05-30 10:00:03 - OpenDogController - INFO - Control loop started at 100Hz
2025-05-30 10:00:03 - OpenDogController - INFO - Robot ready for operation
```

### 2. Operation Modes

#### Safety Checklist
- [ ] All hardware connections verified
- [ ] Battery voltage checked (>22V)
- [ ] Clear workspace (minimum 2m x 2m)
- [ ] Remote controller connection verified
- [ ] Emergency stop button ready

#### Startup Sequence
1. **System Boot** - Initialize all components
2. **Hardware Verification** - Check sensors and actuators
3. **Home Position** - Move to initial pose
4. **Standby Mode** - Wait for remote control input

### 3. Control Modes

#### Mode 1: Manual Control
- **Purpose**: Direct leg position control
- **Input**: Individual leg control via joysticks
- **Use**: Testing, calibration, precise control

#### Mode 2: Walking Control
- **Purpose**: Automatic gait patterns
- **Input**: Direction and speed commands
- **Patterns**: Trot, walk, bound

#### Mode 3: Pose Control
- **Purpose**: Static pose maintenance
- **Input**: Body position and orientation commands
- **Function**: Roll, pitch, yaw control

#### Mode 4: Balance Control
- **Purpose**: Dynamic balance maintenance
- **Sensor**: IMU feedback
- **Algorithm**: PID control

### 4. Gait Patterns

#### Trot Gait
```
Leg order: FL+RR, FR+RL
Period: 0.8 seconds
Max speed: 0.5 m/s
Stability: High
```

#### Walk Gait
```
Leg order: FL, RR, FR, RL
Period: 1.2 seconds
Max speed: 0.3 m/s
Stability: Highest
```

#### Bound Gait
```
Leg order: FL+FR, RL+RR
Period: 0.6 seconds
Max speed: 1.0 m/s
Stability: Low
```

---

## Remote Controller

### 1. Remote Controller Startup

#### Hardware Connection Check
```bash
# Scan I2C devices (LCD)
i2cdetect -y 1

# Check GPIO pin status
gpio readall
```

#### Run Controller
```bash
cd /home/pi/opendog_v3/Code/openDogV3_experimental_stability
python3 remote_controller.py
```

### 2. Controller Interface

#### LCD Display (20x4)
```
Line 0: "Remote Control v3.0 "
Line 1: "Mode: XX  Count: XXXX"
Line 2: "RFB:XXXX RLR:XXXX"
Line 3: "LFB:XXXX LLR:XXXX"
```

#### Button Layout
```
[Menu Up]    [Select]     [Menu Down]
   (3)         (1)           (2)

[Toggle 1]   [Toggle 2]
    (6)         (0)

Switches:
[Motor En]  [IMU En]  [Toggle1]  [Reverse]
   (4)        (5)       (6)        (7)
```

#### Joystick Mapping
```
Left Stick:
- X-axis (A2): Body rotation (yaw)
- Y-axis (A3): Forward/backward

Right Stick:
- X-axis (A4): Side movement
- Y-axis (A5): Body height

Triggers:
- Left (A1): Body roll
- Right (A6): Body pitch
```

### 3. Communication Protocol

#### Transmit Data Structure (28 bytes)
```c
struct SendData {
    int16_t menu_down;     // Menu down button
    int16_t select;        // Select button
    int16_t menu_up;       // Menu up button
    int16_t toggle_bottom; // IMU enable
    int16_t toggle_top;    // Motor enable
    int16_t toggle1;       // Toggle 1
    int16_t toggle2;       // Toggle 2
    int16_t mode;          // Control mode
    int16_t rlr;           // Right stick L/R
    int16_t rfb;           // Right stick F/B
    int16_t rt;            // Right trigger
    int16_t llr;           // Left stick L/R
    int16_t lfb;           // Left stick F/B
    int16_t lt;            // Left trigger
};
```

#### Receive Data Structure (4 bytes)
```c
struct ReceiveData {
    int16_t mode;          // Current mode
    int16_t count;         // Packet counter
};
```

### 4. Control Commands

#### Basic Control
- **Forward/Backward**: Left stick Y-axis
- **Left/Right turn**: Left stick X-axis
- **Side movement**: Right stick X-axis
- **Height control**: Right stick Y-axis

#### Pose Control
- **Body roll**: Left trigger
- **Body pitch**: Right trigger
- **Body yaw**: Combined with side movement

#### Mode Switching
- **Menu Up/Down**: Mode selection
- **Select**: Mode confirmation
- **Motor Enable**: Motor activation/deactivation
- **IMU Enable**: Balance control activation/deactivation

---

## Testing and Diagnostics

### 1. Integrated Test Execution

#### Complete Test Suite
```bash
cd Code/openDogV3_experimental_stability
python3 test_suite.py
```

#### Test Categories
1. **Unit Tests** - Individual module functionality
2. **Integration Tests** - Inter-module interactions
3. **Performance Tests** - Real-time requirements
4. **Communication Tests** - Wireless communication stability
5. **Safety Tests** - Emergency stop and limits

### 2. Individual Component Tests

#### Inverse Kinematics Test
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_kinematics()
"
```

#### Motor Control Test
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_odrive_interface()
"
```

#### IMU Test
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_imu_interface()
"
```

### 3. Performance Benchmarks

#### Control Loop Timing
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_control_loop_timing()
"
```

Expected results:
```
Control Loop Performance:
- Average cycle time: 9.8ms
- Maximum cycle time: 12.1ms
- Minimum cycle time: 9.2ms
- Jitter: 0.9ms
- Target: 10ms (100Hz)
Status: PASS
```

#### Inverse Kinematics Performance
```bash
python3 -c "
from test_suite import TestSuite
suite = TestSuite()
suite.test_kinematics_performance()
"
```

### 4. Diagnostic Tools

#### System Status Monitoring
```bash
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.system_diagnostics()
"
```

Example output:
```
=== System Diagnostics ===
CPU Usage: 45.2%
Memory Usage: 67.8%
Temperature: 52.3°C
Disk Usage: 34.1%

ODrive Status:
- ODrive 1: Connected, Voltage: 24.1V
- ODrive 2: Connected, Voltage: 24.2V

IMU Status:
- Temperature: 28.5°C
- Gyro Bias: X:0.02, Y:-0.01, Z:0.03
- Accel Bias: X:0.15, Y:-0.08, Z:9.78

Radio Status:
- Signal Strength: -45dBm
- Packet Loss: 0.1%
- Latency: 2.3ms
```

### 5. Log Analysis

#### Log File Locations
```
/var/log/opendog/
├── system.log          # System-wide logs
├── control.log         # Control loop logs
├── odrive.log          # Motor control logs
├── imu.log             # IMU data logs
└── communication.log   # Communication logs
```

#### Log Monitoring
```bash
# Real-time log monitoring
tail -f /var/log/opendog/system.log

# Error log search
grep "ERROR" /var/log/opendog/*.log

# Performance analysis
grep "Performance" /var/log/opendog/control.log
```

---

## Troubleshooting

### 1. Common Issues

#### System Won't Start
**Symptoms**: Robot fails to initialize
**Causes**: 
- Hardware connection issues
- Permission problems
- Missing dependencies

**Solutions**:
```bash
# Check hardware connections
python3 hardware_setup.py

# Check permissions
groups $USER

# Reinstall packages
pip install --force-reinstall -r requirements.txt
```

#### ODrive Connection Failed
**Symptoms**: "ODrive not found" error
**Causes**:
- USB connection issues
- Serial port permissions
- ODrive firmware problems

**Solutions**:
```bash
# Check serial ports
ls -la /dev/ttyACM*

# Add permissions
sudo usermod -a -G dialout $USER

# Reboot ODrive
python3 -c "
import odrive
odrv = odrive.find_any()
odrv.reboot()
"
```

#### IMU Reading Failure
**Symptoms**: IMU data is 0 or NaN
**Causes**:
- I2C connection issues
- Wrong address
- Sensor initialization failure

**Solutions**:
```bash
# Scan I2C devices
i2cdetect -y 1

# Reinitialize IMU
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"
```

#### Wireless Communication Dropped
**Symptoms**: Remote control not responding
**Causes**:
- Antenna issues
- Interference
- Power shortage

**Solutions**:
```bash
# Check signal strength
python3 -c "
from remote_controller import RadioInterface
radio = RadioInterface()
# Signal test code
"

# Change channel
# Modify RADIO_CHANNEL in config.py
```

### 2. Performance Issues

#### Control Loop Delay
**Symptoms**: Control cycle exceeds 10ms
**Causes**:
- High CPU load
- Memory shortage
- Inefficient code

**Solutions**:
```bash
# Check CPU usage
htop

# Check memory usage
free -h

# Adjust process priority
sudo renice -10 $(pgrep python3)
```

#### Unstable Walking
**Symptoms**: Robot falls or shakes
**Causes**:
- Inaccurate calibration
- PID gain settings
- Mechanical issues

**Solutions**:
```bash
# Recalibrate IMU
python3 -c "
from hardware_setup import HardwareSetup
setup = HardwareSetup()
setup.calibrate_imu()
"

# Adjust PID gains (config.py)
BALANCE_PID_GAINS = {
    'kp': 1.0,  # Reduce proportional gain
    'ki': 0.1,  # Reduce integral gain
    'kd': 0.05  # Adjust derivative gain
}
```

### 3. Hardware Issues

#### Motor Overheating
**Symptoms**: Motors get hot and performance degrades
**Causes**:
- Overcurrent
- Poor cooling
- Mechanical load

**Solutions**:
```bash
# Check current limit
python3 -c "
import odrive
odrv = odrive.find_any()
print(f'Current limit: {odrv.axis0.motor.config.current_lim}')
"

# Reduce current limit
odrv.axis0.motor.config.current_lim = 15  # Reduce from 20 to 15
```

#### Short Battery Life
**Symptoms**: Operating time shorter than expected
**Causes**:
- High power consumption
- Battery aging
- Inefficient control

**Solutions**:
- Implement power saving mode
- Optimize motor efficiency
- Monitor battery status

### 4. Software Issues

#### Memory Leak
**Symptoms**: Memory usage increases over time
**Causes**:
- Poor object cleanup
- Circular references
- Large data accumulation

**Solutions**:
```bash
# Memory profiling
python3 -m memory_profiler opendog_v3_controller.py

# Force garbage collection
python3 -c "
import gc
gc.collect()
print(f'Objects: {len(gc.get_objects())}')
"
```

#### Exception Handling
**Symptoms**: Unexpected termination
**Causes**:
- Unhandled exceptions
- Resource cleanup failure

**Solutions**:
```python
# Increase logging level
logging.basicConfig(level=logging.DEBUG)

# Enable exception tracking
import traceback
try:
    # Code execution
    pass
except Exception as e:
    logging.error(f"Error: {e}")
    traceback.print_exc()
```

---

## API Reference

### 1. Main Controller API

#### OpenDogController Class

```python
class OpenDogController:
    def __init__(self, config_file: str = "config.py"):
        """
        Initialize OpenDog V3 main controller
        
        Args:
            config_file: Configuration file path
        """
    
    def start(self) -> None:
        """Start controller"""
    
    def stop(self) -> None:
        """Stop controller"""
    
    def set_mode(self, mode: int) -> None:
        """
        Set control mode
        
        Args:
            mode: Control mode (0: manual, 1: walking, 2: pose, 3: balance)
        """
    
    def get_status(self) -> Dict[str, Any]:
        """Return current status"""
    
    def emergency_stop(self) -> None:
        """Emergency stop"""
```

### 2. Inverse Kinematics API

#### Kinematics Class

```python
class Kinematics:
    def inverse_kinematics(self, x: float, y: float, z: float) -> Tuple[float, float]:
        """
        Inverse kinematics calculation
        
        Args:
            x, y, z: Target position (mm)
            
        Returns:
            (theta1, theta2): Joint angles (radians)
        """
    
    def forward_kinematics(self, theta1: float, theta2: float) -> Tuple[float, float, float]:
        """
        Forward kinematics calculation
        
        Args:
            theta1, theta2: Joint angles (radians)
            
        Returns:
            (x, y, z): Foot position (mm)
        """
    
    def calculate_body_ik(self, body_pos: List[float], body_rot: List[float]) -> List[List[float]]:
        """
        Body inverse kinematics calculation
        
        Args:
            body_pos: [x, y, z] body position
            body_rot: [roll, pitch, yaw] body rotation
            
        Returns:
            List of foot positions for 4 legs
        """
```

### 3. Motor Control API

#### ODriveInterface Class

```python
class ODriveInterface:
    def __init__(self):
        """Initialize ODrive interface"""
    
    def connect(self) -> bool:
        """Connect to ODrive"""
    
    def set_position(self, axis: int, position: float) -> None:
        """
        Position control
        
        Args:
            axis: Axis number (0-7)
            position: Target position (radians)
        """
    
    def set_velocity(self, axis: int, velocity: float) -> None:
        """
        Velocity control
        
        Args:
            axis: Axis number (0-7)
            velocity: Target velocity (rad/s)
        """
    
    def get_position(self, axis: int) -> float:
        """Return current position (radians)"""
    
    def get_velocity(self, axis: int) -> float:
        """Return current velocity (rad/s)"""
    
    def enable_motor(self, axis: int) -> None:
        """Enable motor"""
    
    def disable_motor(self, axis: int) -> None:
        """Disable motor"""
```

### 4. IMU API

#### IMUInterface Class

```python
class IMUInterface:
    def __init__(self):
        """Initialize IMU interface"""
    
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read gyroscope (rad/s)"""
    
    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer (m/s²)"""
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """Return current orientation (roll, pitch, yaw)"""
    
    def calibrate(self) -> bool:
        """IMU calibration"""
    
    def set_offsets(self, gyro_offsets: List[float], accel_offsets: List[float]) -> None:
        """Set offsets"""
```

### 5. Remote Control API

#### RemoteController Class

```python
class RemoteController:
    def __init__(self, simulation_mode: bool = False):
        """Initialize remote controller"""
    
    def start(self) -> None:
        """Start controller"""
    
    def stop(self) -> None:
        """Stop controller"""
    
    def read_inputs(self) -> bool:
        """Read inputs"""
    
    def transmit_data(self) -> bool:
        """Transmit data"""
    
    def receive_data(self) -> bool:
        """Receive data"""
    
    def get_status(self) -> Dict[str, Any]:
        """Return status"""
```

### 6. Configuration Management API

#### ConfigManager Class

```python
class ConfigManager:
    def load_config(self, file_path: str) -> Dict[str, Any]:
        """Load configuration file"""
    
    def save_config(self, config: Dict[str, Any], file_path: str) -> None:
        """Save configuration file"""
    
    def get_parameter(self, key: str) -> Any:
        """Return parameter"""
    
    def set_parameter(self, key: str, value: Any) -> None:
        """Set parameter"""
    
    def validate_config(self) -> bool:
        """Validate configuration"""
```

### 7. Test API

#### TestSuite Class

```python
class TestSuite:
    def run_all_tests(self) -> Dict[str, bool]:
        """Run all tests"""
    
    def test_kinematics(self) -> bool:
        """Test inverse kinematics"""
    
    def test_odrive_interface(self) -> bool:
        """Test ODrive interface"""
    
    def test_imu_interface(self) -> bool:
        """Test IMU interface"""
    
    def test_communication(self) -> bool:
        """Test communication"""
    
    def test_integration(self) -> bool:
        """Test integration"""
    
    def performance_benchmark(self) -> Dict[str, float]:
        """Performance benchmark"""
```

---

## Appendix

### A. Hardware Specifications

#### ODrive v3.6 Specifications
- **Voltage**: 12-56V
- **Current**: Up to 60A continuous
- **Control Method**: FOC (Field Oriented Control)
- **Interface**: USB, UART, CAN
- **Encoder**: Incremental encoder support

#### MPU6050 IMU Specifications
- **Gyroscope**: ±250, ±500, ±1000, ±2000 °/s
- **Accelerometer**: ±2g, ±4g, ±8g, ±16g
- **Interface**: I2C
- **Update Rate**: Up to 1kHz

#### nRF24L01 Wireless Module Specifications
- **Frequency**: 2.4GHz ISM band
- **Data Rate**: 250kbps, 1Mbps, 2Mbps
- **Range**: Up to 100m (open space)
- **Interface**: SPI

### B. Electrical Characteristics

#### Power Requirements
- **Main Battery**: 24V Lithium Polymer (6S)
- **Capacity**: Minimum 5000mAh recommended
- **Discharge Rate**: 20C or higher
- **Protection**: BMS required

#### Current Consumption
- **Raspberry Pi 4**: 0.6-1.2A @ 5V
- **ODrive (idle)**: 0.1A @ 24V
- **ODrive (operating)**: 1-5A @ 24V (load dependent)
- **Motor (each)**: 1-20A @ 24V (load dependent)

### C. Mechanical Specifications

#### Robot Dimensions
- **Length**: 400mm
- **Width**: 200mm  
- **Height**: 150-300mm (variable)
- **Weight**: 3-5kg (including battery)

#### Leg Specifications
- **Upper leg length**: 110mm
- **Lower leg length**: 120mm
- **Working radius**: 80-180mm
- **Max load**: 15kg per leg

### D. Software Dependencies

#### Python Package Versions
```
numpy==1.21.0
odrive==0.6.7
pyserial==3.5
RPi.GPIO==0.7.1
smbus2==0.4.1
spidev==3.5
circuitpython-nrf24l01==1.2.3
```

#### System Libraries
```
libatlas-base-dev
libopenblas-dev
python3-dev
python3-pip
i2c-tools
```

### E. Safety Guidelines

#### General Safety
- Power off before work
- Wear appropriate protective equipment
- Ensure adequate workspace
- Prepare emergency stop button

#### Electrical Safety
- Handle batteries carefully
- Prevent short circuits
- Use appropriate fuses
- Check insulation

#### Mechanical Safety
- Beware of moving parts
- Set appropriate torque
- Regular inspections
- Replace worn parts

---

## Contact and Support

### Technical Support
- **GitHub Repository**: [Link]
- **Documentation**: [Link]
- **Issue Tracker**: [Link]

### Community
- **Discord**: [Link]
- **Reddit**: [Link]
- **YouTube**: [Link]

### Contributing
- **Code Contributions**: Pull requests welcome
- **Bug Reports**: Create issues
- **Documentation Improvements**: Suggestions welcome

---

*This manual is part of the OpenDog V3 project. Check the official repository for the latest version.*

**Version**: 1.0
**Last Updated**: May 30, 2025
**License**: MIT License
