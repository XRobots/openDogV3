#!/usr/bin/env python3
"""
OpenDog V3 Hardware Integration Utilities

This module provides utilities for integrating with real hardware including:
- ODrive motor controller setup and calibration
- IMU sensor calibration and setup
- nRF24L01 radio configuration
- System monitoring and diagnostics

Author: OpenDog V3 Project
Date: 2025
"""

import sys
import os
import time
import json
import logging
import subprocess
from typing import Dict, List, Tuple, Optional, Any
import serial
import serial.tools.list_ports

# Add the current directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config import RobotConfig


class ODriveHardwareSetup:
    """Setup and calibration utilities for ODrive motor controllers"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)
        self.odrives = {}  # Will store ODrive connections
    
    def find_odrives(self) -> List[str]:
        """Find all connected ODrive controllers"""
        odrives_found = []
        
        # List all serial ports
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            try:
                # Try to connect and identify ODrive
                ser = serial.Serial(port.device, 115200, timeout=1)
                ser.write(b'i\n')  # Request info
                time.sleep(0.1)
                response = ser.read(1000).decode('utf-8', errors='ignore')
                
                if 'ODrive' in response:
                    odrives_found.append(port.device)
                    self.logger.info(f"Found ODrive on {port.device}")
                
                ser.close()
                
            except Exception as e:
                # Not an ODrive or connection failed
                continue
        
        return odrives_found
    
    def connect_odrive(self, port: str, controller_id: int) -> bool:
        """Connect to a specific ODrive controller"""
        try:
            ser = serial.Serial(port, 115200, timeout=2)
            self.odrives[controller_id] = ser
            self.logger.info(f"Connected to ODrive {controller_id} on {port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect to ODrive on {port}: {e}")
            return False
    
    def send_command(self, controller_id: int, command: str) -> str:
        """Send a command to an ODrive controller"""
        if controller_id not in self.odrives:
            return ""
        
        try:
            ser = self.odrives[controller_id]
            ser.write(f"{command}\n".encode())
            time.sleep(0.1)
            response = ser.read(1000).decode('utf-8', errors='ignore')
            return response.strip()
        except Exception as e:
            self.logger.error(f"Command failed on ODrive {controller_id}: {e}")
            return ""
    
    def configure_odrive(self, controller_id: int, axis: int, motor_type: str) -> bool:
        """Configure an ODrive axis for the robot"""
        commands = []
        
        # Basic configuration based on motor type
        if motor_type == "hip":
            commands.extend([
                f"w axis{axis}.controller.config.pos_gain {self.config.motor_control['pos_gain_hips']}",
                f"w axis{axis}.controller.config.vel_gain {self.config.motor_control['vel_gain']}",
                f"w axis{axis}.controller.config.vel_integrator_gain {self.config.motor_control['integrator_gain']}",
                f"w axis{axis}.motor.config.current_lim 20.0"
            ])
        elif motor_type == "knee":
            commands.extend([
                f"w axis{axis}.controller.config.pos_gain {self.config.motor_control['pos_gain_knee']}",
                f"w axis{axis}.controller.config.vel_gain {self.config.motor_control['vel_gain']}",
                f"w axis{axis}.controller.config.vel_integrator_gain {self.config.motor_control['integrator_gain']}",
                f"w axis{axis}.motor.config.current_lim 20.0"
            ])
        elif motor_type == "shoulder":
            commands.extend([
                f"w axis{axis}.controller.config.pos_gain {self.config.motor_control['pos_gain_shoulder']}",
                f"w axis{axis}.controller.config.vel_gain {self.config.motor_control['vel_gain']}",
                f"w axis{axis}.controller.config.vel_integrator_gain {self.config.motor_control['integrator_gain']}",
                f"w axis{axis}.motor.config.current_lim 20.0"
            ])
        
        # Send all commands
        success = True
        for cmd in commands:
            response = self.send_command(controller_id, cmd)
            if "error" in response.lower():
                self.logger.error(f"Command failed: {cmd}")
                success = False
        
        return success
    
    def calibrate_encoders(self, controller_id: int) -> bool:
        """Run encoder calibration for both axes"""
        self.logger.info(f"Starting encoder calibration for ODrive {controller_id}")
        
        # Calibrate both axes
        for axis in [0, 1]:
            self.logger.info(f"Calibrating axis {axis}")
            
            # Request calibration
            response = self.send_command(controller_id, f"w axis{axis}.requested_state 3")
            
            # Wait for calibration to complete
            timeout = 30  # 30 seconds timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                state = self.send_command(controller_id, f"r axis{axis}.current_state")
                try:
                    state_num = int(state)
                    if state_num == 1:  # IDLE state = calibration complete
                        self.logger.info(f"Axis {axis} calibration complete")
                        break
                except ValueError:
                    pass
                
                time.sleep(1)
            else:
                self.logger.error(f"Axis {axis} calibration timeout")
                return False
        
        return True
    
    def setup_all_odrives(self) -> bool:
        """Setup all ODrive controllers automatically"""
        self.logger.info("Starting ODrive setup...")
        
        # Find all ODrives
        ports = self.find_odrives()
        if len(ports) < 6:
            self.logger.warning(f"Expected 6 ODrives, found {len(ports)}")
        
        # Connect to each ODrive
        for i, port in enumerate(ports[:6]):  # Only use first 6
            if not self.connect_odrive(port, i + 1):
                return False
        
        # Configure each ODrive based on robot configuration
        odrive_configs = [
            (1, 0, "hip"), (1, 1, "hip"),      # ODrive 1: hips
            (2, 0, "knee"), (2, 1, "shoulder"), # ODrive 2: knee + shoulder
            (3, 0, "knee"), (3, 1, "shoulder"), # ODrive 3: knee + shoulder
            (4, 0, "hip"), (4, 1, "hip"),      # ODrive 4: hips
            (5, 0, "knee"), (5, 1, "shoulder"), # ODrive 5: knee + shoulder
            (6, 0, "knee"), (6, 1, "shoulder")  # ODrive 6: knee + shoulder
        ]
        
        for controller_id, axis, motor_type in odrive_configs:
            if controller_id in self.odrives:
                self.configure_odrive(controller_id, axis, motor_type)
        
        self.logger.info("ODrive setup complete")
        return True
    
    def enable_all_motors(self) -> bool:
        """Enable closed-loop control on all motors"""
        self.logger.info("Enabling all motors...")
        
        for controller_id in self.odrives:
            for axis in [0, 1]:
                response = self.send_command(controller_id, f"w axis{axis}.requested_state 8")
                if "error" in response.lower():
                    self.logger.error(f"Failed to enable ODrive {controller_id} axis {axis}")
                    return False
        
        self.logger.info("All motors enabled")
        return True
    
    def disable_all_motors(self) -> bool:
        """Disable all motors (set to idle)"""
        self.logger.info("Disabling all motors...")
        
        for controller_id in self.odrives:
            for axis in [0, 1]:
                self.send_command(controller_id, f"w axis{axis}.requested_state 1")
        
        self.logger.info("All motors disabled")
        return True
    
    def get_motor_status(self) -> Dict[str, Any]:
        """Get status of all motors"""
        status = {}
        
        for controller_id in self.odrives:
            status[f"odrive_{controller_id}"] = {}
            
            for axis in [0, 1]:
                axis_status = {}
                
                # Get current state
                state = self.send_command(controller_id, f"r axis{axis}.current_state")
                axis_status['state'] = state
                
                # Get position
                pos = self.send_command(controller_id, f"r axis{axis}.encoder.pos_estimate")
                axis_status['position'] = pos
                
                # Get errors
                error = self.send_command(controller_id, f"r axis{axis}.error")
                axis_status['error'] = error
                
                status[f"odrive_{controller_id}"][f"axis_{axis}"] = axis_status
        
        return status


class IMUHardwareSetup:
    """Setup and calibration utilities for IMU sensor"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def find_imu(self) -> Optional[str]:
        """Find IMU on I2C bus"""
        try:
            # Try to import I2C libraries
            import smbus
            bus = smbus.SMBus(1)
            
            # Try common IMU addresses
            imu_addresses = [0x68, 0x69]  # MPU6050 addresses
            
            for addr in imu_addresses:
                try:
                    # Try to read WHO_AM_I register
                    who_am_i = bus.read_byte_data(addr, 0x75)
                    if who_am_i == 0x68:  # MPU6050 WHO_AM_I value
                        self.logger.info(f"Found MPU6050 at address 0x{addr:02x}")
                        return f"i2c-1-{addr:02x}"
                except:
                    continue
            
            return None
            
        except ImportError:
            self.logger.warning("I2C libraries not available")
            return None
    
    def calibrate_imu(self, device: str, samples: int = 1000) -> Dict[str, float]:
        """Calibrate IMU accelerometer and gyroscope"""
        self.logger.info(f"Calibrating IMU with {samples} samples...")
        
        try:
            import smbus
            bus = smbus.SMBus(1)
            addr = int(device.split('-')[-1], 16)
            
            # Initialize MPU6050
            bus.write_byte_data(addr, 0x6B, 0)  # Wake up
            time.sleep(0.1)
            
            # Collect calibration data
            accel_offsets = [0, 0, 0]
            gyro_offsets = [0, 0, 0]
            
            for i in range(samples):
                # Read accelerometer data
                accel_data = []
                for reg in [0x3B, 0x3D, 0x3F]:  # ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H
                    high = bus.read_byte_data(addr, reg)
                    low = bus.read_byte_data(addr, reg + 1)
                    value = (high << 8) | low
                    if value > 32767:
                        value -= 65536
                    accel_data.append(value)
                
                # Read gyroscope data
                gyro_data = []
                for reg in [0x43, 0x45, 0x47]:  # GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H
                    high = bus.read_byte_data(addr, reg)
                    low = bus.read_byte_data(addr, reg + 1)
                    value = (high << 8) | low
                    if value > 32767:
                        value -= 65536
                    gyro_data.append(value)
                
                # Accumulate for averaging
                for j in range(3):
                    accel_offsets[j] += accel_data[j]
                    gyro_offsets[j] += gyro_data[j]
                
                if i % 100 == 0:
                    self.logger.info(f"Calibration progress: {i}/{samples}")
                
                time.sleep(0.001)
            
            # Calculate averages
            for i in range(3):
                accel_offsets[i] /= samples
                gyro_offsets[i] /= samples
            
            # Accelerometer Z should read ~16384 (1g), adjust offset
            accel_offsets[2] -= 16384
            
            calibration = {
                'accel_offset_x': accel_offsets[0],
                'accel_offset_y': accel_offsets[1],
                'accel_offset_z': accel_offsets[2],
                'gyro_offset_x': gyro_offsets[0],
                'gyro_offset_y': gyro_offsets[1],
                'gyro_offset_z': gyro_offsets[2]
            }
            
            self.logger.info("IMU calibration complete")
            return calibration
            
        except Exception as e:
            self.logger.error(f"IMU calibration failed: {e}")
            return {}


class RadioHardwareSetup:
    """Setup and configuration utilities for nRF24L01 radio"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def setup_radio(self) -> bool:
        """Setup nRF24L01 radio module"""
        try:
            # This would configure the actual radio hardware
            # For now, just log the configuration
            self.logger.info("Radio configuration:")
            self.logger.info(f"  Channel: {self.config.communication['radio_channel']}")
            self.logger.info(f"  Data rate: {self.config.communication['radio_data_rate']}")
            self.logger.info(f"  Power level: {self.config.communication['radio_power_level']}")
            self.logger.info(f"  Addresses: {self.config.communication['radio_addresses']}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Radio setup failed: {e}")
            return False


class SystemDiagnostics:
    """System monitoring and diagnostic utilities"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.logger = logging.getLogger(__name__)
    
    def check_system_resources(self) -> Dict[str, Any]:
        """Check system resources (CPU, memory, disk)"""
        try:
            import psutil
            
            return {
                'cpu_percent': psutil.cpu_percent(interval=1),
                'memory_percent': psutil.virtual_memory().percent,
                'disk_percent': psutil.disk_usage('/').percent,
                'temperature': self.get_cpu_temperature()
            }
        except ImportError:
            return {'error': 'psutil not available'}
    
    def get_cpu_temperature(self) -> Optional[float]:
        """Get CPU temperature (Raspberry Pi)"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
                return temp
        except:
            return None
    
    def check_gpio_pins(self) -> Dict[str, Any]:
        """Check GPIO pin status"""
        try:
            import RPi.GPIO as GPIO
            
            # Check specific pins used by the robot
            pins_to_check = [9, 10, 14, 15, 18, 24, 25]  # Example pins
            pin_status = {}
            
            GPIO.setmode(GPIO.BCM)
            
            for pin in pins_to_check:
                try:
                    GPIO.setup(pin, GPIO.IN)
                    value = GPIO.input(pin)
                    pin_status[f"pin_{pin}"] = value
                except:
                    pin_status[f"pin_{pin}"] = "error"
            
            return pin_status
            
        except ImportError:
            return {'error': 'GPIO not available'}
    
    def test_serial_ports(self) -> List[Dict[str, Any]]:
        """Test all available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_status = []
        
        for port in ports:
            status = {
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'accessible': False
            }
            
            try:
                ser = serial.Serial(port.device, 115200, timeout=1)
                ser.close()
                status['accessible'] = True
            except:
                pass
            
            port_status.append(status)
        
        return port_status
    
    def run_full_diagnostics(self) -> Dict[str, Any]:
        """Run complete system diagnostics"""
        self.logger.info("Running full system diagnostics...")
        
        diagnostics = {
            'timestamp': time.time(),
            'system_resources': self.check_system_resources(),
            'gpio_pins': self.check_gpio_pins(),
            'serial_ports': self.test_serial_ports(),
        }
        
        # Test hardware components
        odrive_setup = ODriveHardwareSetup(self.config)
        imu_setup = IMUHardwareSetup(self.config)
        radio_setup = RadioHardwareSetup(self.config)
        
        diagnostics['odrives_found'] = odrive_setup.find_odrives()
        diagnostics['imu_found'] = imu_setup.find_imu() is not None
        diagnostics['radio_setup'] = radio_setup.setup_radio()
        
        return diagnostics


def main():
    """Main function for hardware setup and diagnostics"""
    import argparse
    
    parser = argparse.ArgumentParser(description='OpenDog V3 Hardware Utilities')
    parser.add_argument('--setup-odrives', action='store_true', help='Setup ODrive controllers')
    parser.add_argument('--calibrate-imu', action='store_true', help='Calibrate IMU sensor')
    parser.add_argument('--setup-radio', action='store_true', help='Setup radio module')
    parser.add_argument('--diagnostics', action='store_true', help='Run system diagnostics')
    parser.add_argument('--enable-motors', action='store_true', help='Enable all motors')
    parser.add_argument('--disable-motors', action='store_true', help='Disable all motors')
    parser.add_argument('--save-config', help='Save configuration to file')
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Load configuration
    config = RobotConfig()
    
    if args.setup_odrives:
        odrive_setup = ODriveHardwareSetup(config)
        success = odrive_setup.setup_all_odrives()
        print(f"ODrive setup: {'SUCCESS' if success else 'FAILED'}")
    
    if args.calibrate_imu:
        imu_setup = IMUHardwareSetup(config)
        device = imu_setup.find_imu()
        if device:
            calibration = imu_setup.calibrate_imu(device)
            if calibration:
                print("IMU calibration results:")
                for key, value in calibration.items():
                    print(f"  {key}: {value:.3f}")
        else:
            print("No IMU found")
    
    if args.setup_radio:
        radio_setup = RadioHardwareSetup(config)
        success = radio_setup.setup_radio()
        print(f"Radio setup: {'SUCCESS' if success else 'FAILED'}")
    
    if args.enable_motors:
        odrive_setup = ODriveHardwareSetup(config)
        ports = odrive_setup.find_odrives()
        for i, port in enumerate(ports[:6]):
            odrive_setup.connect_odrive(port, i + 1)
        success = odrive_setup.enable_all_motors()
        print(f"Motor enable: {'SUCCESS' if success else 'FAILED'}")
    
    if args.disable_motors:
        odrive_setup = ODriveHardwareSetup(config)
        ports = odrive_setup.find_odrives()
        for i, port in enumerate(ports[:6]):
            odrive_setup.connect_odrive(port, i + 1)
        success = odrive_setup.disable_all_motors()
        print(f"Motor disable: {'SUCCESS' if success else 'FAILED'}")
    
    if args.diagnostics:
        diagnostics = SystemDiagnostics(config)
        results = diagnostics.run_full_diagnostics()
        
        print("System Diagnostics Results:")
        print("=" * 50)
        print(json.dumps(results, indent=2, default=str))
    
    if args.save_config:
        config.save_to_file(args.save_config)
        print(f"Configuration saved to {args.save_config}")
    
    # If no arguments, show help
    if not any(vars(args).values()):
        parser.print_help()


if __name__ == "__main__":
    main()
