#!/usr/bin/env python3
"""
IMU Interface Module for OpenDog V3
Handles IMU sensor communication and data processing for robot stabilization

This module provides:
- IMU sensor communication (accelerometer, gyroscope, magnetometer)
- Complementary filter for attitude estimation
- Roll, pitch, yaw calculations
- Data smoothing and filtering
- Calibration support

Based on the Arduino implementation using complementary filtering
for real-time attitude estimation at 100Hz update rate.

Author: Python IMU interface for OpenDog V3
Date: 2024
"""

import serial
import time
import math
import logging
import threading
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from collections import deque

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class IMUData:
    """Raw IMU sensor data"""
    # Accelerometer (m/s²)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    # Gyroscope (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Magnetometer (µT)
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    
    # Temperature (°C)
    temperature: float = 0.0
    
    # Timestamp
    timestamp: float = 0.0

@dataclass
class AttitudeData:
    """Processed attitude data"""
    roll: float = 0.0       # Roll angle in degrees
    pitch: float = 0.0      # Pitch angle in degrees
    yaw: float = 0.0        # Yaw angle in degrees
    
    # Angular velocities (deg/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    
    # Timestamp
    timestamp: float = 0.0

@dataclass
class IMUCalibration:
    """IMU calibration parameters"""
    # Accelerometer bias (m/s²)
    accel_bias_x: float = 0.0
    accel_bias_y: float = 0.0
    accel_bias_z: float = 0.0
    
    # Gyroscope bias (rad/s)
    gyro_bias_x: float = 0.0
    gyro_bias_y: float = 0.0
    gyro_bias_z: float = 0.0
    
    # Magnetometer bias (µT)
    mag_bias_x: float = 0.0
    mag_bias_y: float = 0.0
    mag_bias_z: float = 0.0
    
    # Scale factors
    accel_scale: float = 1.0
    gyro_scale: float = 1.0
    mag_scale: float = 1.0

class ComplementaryFilter:
    """
    Complementary filter for attitude estimation
    Combines accelerometer and gyroscope data for stable attitude estimation
    """
    
    def __init__(self, alpha: float = 0.98, dt: float = 0.01):
        """
        Initialize complementary filter
        
        Args:
            alpha: Filter coefficient (0-1, higher = more gyro influence)
            dt: Sample time in seconds
        """
        self.alpha = alpha
        self.dt = dt
        
        # Current attitude estimates
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Previous timestamp
        self.last_time = time.time()
        
        logger.info(f"Complementary filter initialized with alpha={alpha}, dt={dt}")
    
    def update(self, imu_data: IMUData) -> AttitudeData:
        """
        Update attitude estimate with new IMU data
        
        Args:
            imu_data: Raw IMU sensor data
            
        Returns:
            Updated attitude data
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Calculate accelerometer angles
        accel_roll = math.atan2(imu_data.accel_y, 
                               math.sqrt(imu_data.accel_x**2 + imu_data.accel_z**2))
        accel_pitch = math.atan2(-imu_data.accel_x,
                                math.sqrt(imu_data.accel_y**2 + imu_data.accel_z**2))
        
        # Convert gyro data to degrees and integrate
        gyro_roll_rate = math.degrees(imu_data.gyro_x)
        gyro_pitch_rate = math.degrees(imu_data.gyro_y)
        gyro_yaw_rate = math.degrees(imu_data.gyro_z)
        
        # Apply complementary filter
        self.roll = self.alpha * (self.roll + gyro_roll_rate * dt) + \
                   (1 - self.alpha) * math.degrees(accel_roll)
        
        self.pitch = self.alpha * (self.pitch + gyro_pitch_rate * dt) + \
                    (1 - self.alpha) * math.degrees(accel_pitch)
        
        # Yaw integration (no accelerometer correction for yaw)
        self.yaw += gyro_yaw_rate * dt
        
        # Wrap yaw to [-180, 180]
        self.yaw = ((self.yaw + 180) % 360) - 180
        
        return AttitudeData(
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
            roll_rate=gyro_roll_rate,
            pitch_rate=gyro_pitch_rate,
            yaw_rate=gyro_yaw_rate,
            timestamp=current_time
        )
    
    def reset(self):
        """Reset filter state"""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        logger.info("Complementary filter reset")

class IMUInterface:
    """
    IMU sensor interface for OpenDog V3
    Handles communication with IMU sensor and attitude estimation
    """
    
    def __init__(self, 
                 port: str = "/dev/ttyUSB7",
                 baudrate: int = 115200,
                 timeout: float = 0.1,
                 filter_alpha: float = 0.98):
        """
        Initialize IMU interface
        
        Args:
            port: Serial port for IMU communication
            baudrate: Serial communication speed
            timeout: Serial timeout in seconds
            filter_alpha: Complementary filter coefficient
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        # Serial connection
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        
        # Data processing
        self.complementary_filter = ComplementaryFilter(filter_alpha)
        self.calibration = IMUCalibration()
        
        # Current data
        self.raw_data = IMUData()
        self.attitude_data = AttitudeData()
        
        # Data history for smoothing
        self.data_history_size = 10
        self.roll_history = deque(maxlen=self.data_history_size)
        self.pitch_history = deque(maxlen=self.data_history_size)
        self.yaw_history = deque(maxlen=self.data_history_size)
        
        # Threading for continuous reading
        self.reading_thread: Optional[threading.Thread] = None
        self.reading_active = False
        self.data_lock = threading.Lock()
        
        # Statistics
        self.sample_count = 0
        self.error_count = 0
        self.last_update_time = 0.0
        
        logger.info(f"IMU interface initialized on port {port}")
    
    def connect(self) -> bool:
        """
        Connect to IMU sensor
        
        Returns:
            True if connection successful
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            
            # Wait for connection to stabilize
            time.sleep(0.1)
            
            # Test communication
            if self._test_communication():
                self.is_connected = True
                logger.info(f"IMU connected successfully on {self.port}")
                return True
            else:
                logger.error("IMU communication test failed")
                self.disconnect()
                return False
                
        except Exception as e:
            logger.error(f"Failed to connect to IMU on {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from IMU sensor"""
        self.stop_reading()
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        self.is_connected = False
        logger.info("IMU disconnected")
    
    def start_reading(self) -> bool:
        """
        Start continuous IMU data reading in background thread
        
        Returns:
            True if reading started successfully
        """
        if not self.is_connected:
            logger.error("IMU not connected")
            return False
        
        if self.reading_active:
            logger.warning("IMU reading already active")
            return True
        
        self.reading_active = True
        self.reading_thread = threading.Thread(target=self._reading_loop, daemon=True)
        self.reading_thread.start()
        
        logger.info("IMU continuous reading started")
        return True
    
    def stop_reading(self):
        """Stop continuous IMU data reading"""
        self.reading_active = False
        
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=1.0)
        
        logger.info("IMU continuous reading stopped")
    
    def read_raw_data(self) -> Optional[IMUData]:
        """
        Read raw IMU data once
        
        Returns:
            Raw IMU data or None if read failed
        """
        if not self.is_connected or not self.serial_connection:
            return None
        
        try:
            # In a real implementation, this would parse actual IMU sensor data
            # For now, we'll simulate with placeholder data
            current_time = time.time()
            
            # Simulate IMU data (replace with actual sensor parsing)
            raw_data = IMUData(
                accel_x=0.0,  # Would be parsed from sensor
                accel_y=0.0,
                accel_z=9.81,
                gyro_x=0.0,
                gyro_y=0.0,
                gyro_z=0.0,
                mag_x=0.0,
                mag_y=0.0,
                mag_z=0.0,
                temperature=25.0,
                timestamp=current_time
            )
            
            return raw_data
            
        except Exception as e:
            logger.error(f"Failed to read IMU data: {e}")
            self.error_count += 1
            return None
    
    def process_data(self, raw_data: IMUData) -> AttitudeData:
        """
        Process raw IMU data to get attitude
        
        Args:
            raw_data: Raw IMU sensor data
            
        Returns:
            Processed attitude data
        """
        # Apply calibration
        calibrated_data = self._apply_calibration(raw_data)
        
        # Update complementary filter
        attitude = self.complementary_filter.update(calibrated_data)
        
        # Apply smoothing
        attitude = self._apply_smoothing(attitude)
        
        return attitude
    
    def get_attitude(self) -> AttitudeData:
        """
        Get current attitude data (thread-safe)
        
        Returns:
            Current attitude data
        """
        with self.data_lock:
            return AttitudeData(
                roll=self.attitude_data.roll,
                pitch=self.attitude_data.pitch,
                yaw=self.attitude_data.yaw,
                roll_rate=self.attitude_data.roll_rate,
                pitch_rate=self.attitude_data.pitch_rate,
                yaw_rate=self.attitude_data.yaw_rate,
                timestamp=self.attitude_data.timestamp
            )
    
    def get_raw_data(self) -> IMUData:
        """
        Get current raw data (thread-safe)
        
        Returns:
            Current raw IMU data
        """
        with self.data_lock:
            return IMUData(
                accel_x=self.raw_data.accel_x,
                accel_y=self.raw_data.accel_y,
                accel_z=self.raw_data.accel_z,
                gyro_x=self.raw_data.gyro_x,
                gyro_y=self.raw_data.gyro_y,
                gyro_z=self.raw_data.gyro_z,
                mag_x=self.raw_data.mag_x,
                mag_y=self.raw_data.mag_y,
                mag_z=self.raw_data.mag_z,
                temperature=self.raw_data.temperature,
                timestamp=self.raw_data.timestamp
            )
    
    def calibrate(self, duration: float = 5.0) -> bool:
        """
        Perform IMU calibration (robot should be stationary)
        
        Args:
            duration: Calibration duration in seconds
            
        Returns:
            True if calibration successful
        """
        if not self.is_connected:
            logger.error("IMU not connected for calibration")
            return False
        
        logger.info(f"Starting IMU calibration for {duration} seconds...")
        logger.info("Keep robot stationary during calibration")
        
        # Collect calibration data
        accel_x_sum = 0.0
        accel_y_sum = 0.0
        accel_z_sum = 0.0
        gyro_x_sum = 0.0
        gyro_y_sum = 0.0
        gyro_z_sum = 0.0
        
        sample_count = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            raw_data = self.read_raw_data()
            if raw_data:
                accel_x_sum += raw_data.accel_x
                accel_y_sum += raw_data.accel_y
                accel_z_sum += raw_data.accel_z
                gyro_x_sum += raw_data.gyro_x
                gyro_y_sum += raw_data.gyro_y
                gyro_z_sum += raw_data.gyro_z
                sample_count += 1
            
            time.sleep(0.01)  # 100Hz sampling
        
        if sample_count < 10:
            logger.error("Insufficient calibration samples")
            return False
        
        # Calculate bias values
        self.calibration.accel_bias_x = accel_x_sum / sample_count
        self.calibration.accel_bias_y = accel_y_sum / sample_count
        self.calibration.accel_bias_z = (accel_z_sum / sample_count) - 9.81  # Remove gravity
        
        self.calibration.gyro_bias_x = gyro_x_sum / sample_count
        self.calibration.gyro_bias_y = gyro_y_sum / sample_count
        self.calibration.gyro_bias_z = gyro_z_sum / sample_count
        
        logger.info(f"IMU calibration completed with {sample_count} samples")
        logger.info(f"Accel bias: {self.calibration.accel_bias_x:.4f}, "
                   f"{self.calibration.accel_bias_y:.4f}, "
                   f"{self.calibration.accel_bias_z:.4f}")
        logger.info(f"Gyro bias: {self.calibration.gyro_bias_x:.6f}, "
                   f"{self.calibration.gyro_bias_y:.6f}, "
                   f"{self.calibration.gyro_bias_z:.6f}")
        
        return True
    
    def reset_attitude(self):
        """Reset attitude estimation"""
        self.complementary_filter.reset()
        with self.data_lock:
            self.attitude_data = AttitudeData()
        logger.info("IMU attitude reset")
    
    def get_status(self) -> Dict:
        """
        Get IMU interface status
        
        Returns:
            Dictionary with status information
        """
        return {
            'connected': self.is_connected,
            'reading_active': self.reading_active,
            'sample_count': self.sample_count,
            'error_count': self.error_count,
            'error_rate': self.error_count / max(1, self.sample_count),
            'last_update': self.last_update_time,
            'data_age': time.time() - self.last_update_time if self.last_update_time > 0 else 0,
            'filter_alpha': self.complementary_filter.alpha,
            'calibrated': self._is_calibrated()
        }
    
    def _test_communication(self) -> bool:
        """Test IMU communication"""
        try:
            # In real implementation, would send test command to IMU
            # For now, just check if serial port is open
            return self.serial_connection and self.serial_connection.is_open
        except Exception:
            return False
    
    def _reading_loop(self):
        """Main reading loop for continuous data acquisition"""
        logger.info("IMU reading loop started")
        
        while self.reading_active:
            try:
                # Read raw data
                raw_data = self.read_raw_data()
                if raw_data:
                    # Process data
                    attitude = self.process_data(raw_data)
                    
                    # Update shared data (thread-safe)
                    with self.data_lock:
                        self.raw_data = raw_data
                        self.attitude_data = attitude
                        self.last_update_time = time.time()
                    
                    self.sample_count += 1
                
                # Control loop timing (aim for 100Hz)
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Error in IMU reading loop: {e}")
                self.error_count += 1
                time.sleep(0.1)  # Longer delay on error
        
        logger.info("IMU reading loop stopped")
    
    def _apply_calibration(self, raw_data: IMUData) -> IMUData:
        """Apply calibration to raw data"""
        return IMUData(
            accel_x=(raw_data.accel_x - self.calibration.accel_bias_x) * self.calibration.accel_scale,
            accel_y=(raw_data.accel_y - self.calibration.accel_bias_y) * self.calibration.accel_scale,
            accel_z=(raw_data.accel_z - self.calibration.accel_bias_z) * self.calibration.accel_scale,
            gyro_x=(raw_data.gyro_x - self.calibration.gyro_bias_x) * self.calibration.gyro_scale,
            gyro_y=(raw_data.gyro_y - self.calibration.gyro_bias_y) * self.calibration.gyro_scale,
            gyro_z=(raw_data.gyro_z - self.calibration.gyro_bias_z) * self.calibration.gyro_scale,
            mag_x=(raw_data.mag_x - self.calibration.mag_bias_x) * self.calibration.mag_scale,
            mag_y=(raw_data.mag_y - self.calibration.mag_bias_y) * self.calibration.mag_scale,
            mag_z=(raw_data.mag_z - self.calibration.mag_bias_z) * self.calibration.mag_scale,
            temperature=raw_data.temperature,
            timestamp=raw_data.timestamp
        )
    
    def _apply_smoothing(self, attitude: AttitudeData) -> AttitudeData:
        """Apply smoothing to attitude data"""
        # Add to history
        self.roll_history.append(attitude.roll)
        self.pitch_history.append(attitude.pitch)
        self.yaw_history.append(attitude.yaw)
        
        # Calculate smoothed values
        if len(self.roll_history) > 1:
            smoothed_roll = sum(self.roll_history) / len(self.roll_history)
            smoothed_pitch = sum(self.pitch_history) / len(self.pitch_history)
            smoothed_yaw = sum(self.yaw_history) / len(self.yaw_history)
            
            return AttitudeData(
                roll=smoothed_roll,
                pitch=smoothed_pitch,
                yaw=smoothed_yaw,
                roll_rate=attitude.roll_rate,
                pitch_rate=attitude.pitch_rate,
                yaw_rate=attitude.yaw_rate,
                timestamp=attitude.timestamp
            )
        
        return attitude
    
    def _is_calibrated(self) -> bool:
        """Check if IMU is calibrated"""
        return (abs(self.calibration.gyro_bias_x) > 1e-6 or
                abs(self.calibration.gyro_bias_y) > 1e-6 or
                abs(self.calibration.gyro_bias_z) > 1e-6)

# Example usage and testing
if __name__ == "__main__":
    # Initialize IMU interface
    imu = IMUInterface("/dev/ttyUSB7")
    
    print("IMU Interface Test")
    print("=" * 30)
    
    # Test connection (will fail without actual hardware)
    if imu.connect():
        print("IMU connected successfully")
        
        # Test calibration
        print("Starting calibration...")
        if imu.calibrate(duration=2.0):
            print("Calibration completed")
        
        # Start continuous reading
        if imu.start_reading():
            print("Continuous reading started")
            
            # Read data for a few seconds
            start_time = time.time()
            while time.time() - start_time < 5.0:
                attitude = imu.get_attitude()
                print(f"Attitude - Roll: {attitude.roll:.2f}°, "
                      f"Pitch: {attitude.pitch:.2f}°, "
                      f"Yaw: {attitude.yaw:.2f}°")
                time.sleep(0.5)
            
            imu.stop_reading()
        
        imu.disconnect()
    else:
        print("Failed to connect to IMU (expected without hardware)")
    
    # Show status
    status = imu.get_status()
    print(f"\nIMU Status:")
    for key, value in status.items():
        print(f"  {key}: {value}")
    
    print(f"\nIMU interface test completed")
