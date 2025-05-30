#!/usr/bin/env python3
"""
OpenDogV3 Experimental Stability - Python Implementation
Arduino C++ code converted to Python for quadruped robot control

Original Arduino code by: OpenDog Team
Converted to Python by: Assistant

Description:
This module provides Python implementation of the OpenDogV3 experimental stability system,
including inverse kinematics, ODrive motor control, IMU stabilization, and gait control.

Key Features:
- 6-DOF inverse kinematics (X, Y, Z, Roll, Pitch, Yaw)
- Real-time 100Hz control loop
- IMU-based dynamic stabilization
- Trot gait implementation
- ODrive motor control with safety systems
- Remote control integration
- Interpolation system for smooth motion
"""

import time
import math
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import numpy as np
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class RemoteControlData:
    """Structure for remote control data"""
    RLR: float = 0.0        # Right stick left/right
    RFB: float = 0.0        # Right stick forward/backward  
    RT: float = 340.0       # Right trigger
    LLR: float = 0.0        # Left stick left/right
    LFB: float = 0.0        # Left stick forward/backward
    LT: float = 0.0         # Left trigger
    toggleTop: int = 0      # Motor enable switch
    toggleBottom: int = 0   # IMU enable switch
    Select: int = 0         # Select button
    
@dataclass
class Vector3:
    """3D vector structure"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class JointAngles:
    """Joint angles for one leg"""
    hip: float = 0.0
    shoulder: float = 0.0 
    knee: float = 0.0

class Filter:
    """Simple low-pass filter implementation"""
    
    def __init__(self, alpha: float = 0.8):
        self.alpha = alpha
        self.prev_value = None
        
    def update(self, new_value: float) -> float:
        """Apply filter to new value"""
        if self.prev_value is None:
            self.prev_value = new_value
            return new_value
            
        filtered = self.alpha * self.prev_value + (1 - self.alpha) * new_value
        self.prev_value = filtered
        return filtered

class Interpolation:
    """Interpolation class for smooth motion control"""
    
    def __init__(self):
        self.start_value = 0.0
        self.end_value = 0.0
        self.start_time = 0.0
        self.duration = 0.0
        self.is_active = False
        
    def start(self, start_val: float, end_val: float, duration_ms: float):
        """Start interpolation from start_val to end_val over duration_ms"""
        self.start_value = start_val
        self.end_value = end_val
        self.start_time = time.time() * 1000  # Convert to milliseconds
        self.duration = duration_ms
        self.is_active = True
        
    def update(self) -> float:
        """Update interpolation and return current value"""
        if not self.is_active:
            return self.end_value
            
        current_time = time.time() * 1000
        elapsed = current_time - self.start_time
        
        if elapsed >= self.duration:
            self.is_active = False
            return self.end_value
            
        # Linear interpolation
        progress = elapsed / self.duration
        return self.start_value + (self.end_value - self.start_value) * progress

class ODriveController:
    """ODrive motor controller interface"""
    
    def __init__(self, port: str):
        self.port = port
        self.connected = False
        logger.info(f"ODrive controller initialized on {port}")
        
    def connect(self):
        """Connect to ODrive"""
        # TODO: Implement actual ODrive connection
        logger.info(f"Connecting to ODrive on {self.port}")
        self.connected = True
        
    def set_position(self, axis: int, position: float):
        """Set motor position"""
        if not self.connected:
            logger.warning(f"ODrive {self.port} not connected")
            return
            
        # TODO: Implement actual position setting
        logger.debug(f"ODrive {self.port} axis {axis}: position = {position:.3f}")
        
    def run_state(self, axis: int, state: int, wait: bool = False):
        """Set motor state"""
        if not self.connected:
            logger.warning(f"ODrive {self.port} not connected")
            return
            
        logger.debug(f"ODrive {self.port} axis {axis}: state = {state}")

class IMU:
    """IMU (MPU6050) interface for motion sensing"""
    
    def __init__(self):
        self.connected = False
        self.accel_data = Vector3()
        self.gyro_data = Vector3()
        self.pitch = 0.0
        self.roll = 0.0
        
        # Complementary filter variables
        self.mix_x = 0.0
        self.mix_y = 0.0
        
        # Filter constant
        self.GYR_GAIN = 0.00763358
        
    def initialize(self):
        """Initialize IMU"""
        logger.info("Initializing IMU (MPU6050)")
        # TODO: Implement actual IMU initialization
        self.connected = True
        
    def read_data(self):
        """Read raw IMU data"""
        if not self.connected:
            return
            
        # TODO: Implement actual IMU data reading
        # For now, return simulated data
        pass
        
    def update_attitude(self, dt: float = 0.01):
        """Update roll/pitch using complementary filter"""
        if not self.connected:
            self.pitch = 0.0
            self.roll = 0.0
            return
            
        # TODO: Implement actual IMU reading
        # Simulated accelerometer data (in degrees)
        accel_y = 0.0  # Pitch from accelerometer
        accel_x = 0.0  # Roll from accelerometer
        
        # Simulated gyroscope data (in rad/s)
        gyro_x = 0.0
        gyro_y = 0.0
        
        # Complementary filter
        K = 0.9
        A = K / (K + dt)
        
        self.mix_x = A * (self.mix_x + gyro_x * dt) + (1 - A) * accel_y
        self.mix_y = A * (self.mix_y + gyro_y * dt) + (1 - A) * accel_x
        
        # Apply IMU trim offsets
        self.pitch = self.mix_x + 2.7
        self.roll = self.mix_y - 5.0

class OpenDogV3Controller:
    """Main controller for OpenDogV3 quadruped robot"""
    
    # Robot physical parameters
    BODY_WIDTH = 59.0          # Half distance between hip pivots (mm)
    BODY_LENGTH = 150.0        # Distance from front to back hips (mm)
    HIP_OFFSET = 108.0         # Distance from hip pivot to leg center (mm)
    SHIN_LENGTH = 200.0        # Lower leg segment length (mm)
    THIGH_LENGTH = 200.0       # Upper leg segment length (mm)
    
    # Motor conversion factor (degrees to motor turns)
    MOTOR_CONVERSION = 0.02777777777777777777777777777778
    
    def __init__(self):
        """Initialize robot controller"""
        
        # Control timing
        self.control_frequency = 100  # Hz
        self.control_period = 1.0 / self.control_frequency
        
        # Remote control data
        self.remote_data = RemoteControlData()
        self.prev_remote_data = RemoteControlData()
        
        # Filtered remote data
        self.filtered_data = RemoteControlData()
        
        # Control modes
        self.run_mode = 0
        self.mode_confirm = 0
        self.mode_confirm_flag = 0
        
        # IMU system
        self.imu = IMU()
        self.imu_enabled = False
        
        # Initialize ODrive controllers (6 controllers for 12 motors)
        self.odrives = {
            1: ODriveController("Serial1"),  # Hip motors 1,4
            2: ODriveController("Serial2"),  # Right front leg
            3: ODriveController("Serial3"),  # Right rear leg  
            4: ODriveController("Serial4"),  # Hip motors 2,3
            5: ODriveController("Serial5"),  # Left front leg
            6: ODriveController("Serial6"),  # Left rear leg
        }
        
        # Motor offset values (calibrated for each motor)
        self.motor_offsets = {
            # Knee motors
            20: -0.1,   # ODrive 2, axis 0 - right front knee
            30: -0.45,  # ODrive 3, axis 0 - right rear knee
            50: -0.05,  # ODrive 5, axis 0 - left front knee  
            60: -0.4,   # ODrive 6, axis 0 - left rear knee
            
            # Shoulder motors
            21: -0.1,   # ODrive 2, axis 1 - right front shoulder
            31: 0.45,   # ODrive 3, axis 1 - right rear shoulder
            51: 0.66,   # ODrive 5, axis 1 - left front shoulder
            61: -0.08,  # ODrive 6, axis 1 - left rear shoulder
            
            # Hip motors
            10: 0.27,   # ODrive 1, axis 0 - right front hip
            11: 0.1,    # ODrive 1, axis 1 - right rear hip
            40: 0.07,   # ODrive 4, axis 0 - left front hip
            41: 0.35,   # ODrive 4, axis 1 - left rear hip
        }
        
        # PID gain values
        self.pos_gain_knee = 20.0
        self.pos_gain_shoulder = 20.0
        self.pos_gain_hips = 30.0
        self.vel_gain = 0.16
        self.integrator_gain = 0.32
        
        # Input filters
        self.filters = {
            'RFB': Filter(0.6),
            'RLR': Filter(0.6), 
            'RT': Filter(0.6),
            'LFB': Filter(0.6),
            'LLR': Filter(0.6),
            'LT': Filter(0.6),
        }
        
        # Walking gait variables
        self.gait_timer1 = 0  # FB gait timer
        self.gait_timer2 = 0  # LR gait timer
        self.gait_timer3 = 0  # Yaw gait timer
        self.timer_scale = 0.0
        
        # Leg positions for walking
        self.leg_positions = {
            'fr': {'x': 0.0, 'y': 0.0, 'z': 340.0},  # Front right
            'fl': {'x': 0.0, 'y': 0.0, 'z': 340.0},  # Front left
            'br': {'x': 0.0, 'y': 0.0, 'z': 340.0},  # Back right
            'bl': {'x': 0.0, 'y': 0.0, 'z': 340.0},  # Back left
        }
        
        # Interpolation objects for smooth motion
        self.interpolators = {
            'fr_x': Interpolation(), 'fr_y': Interpolation(), 'fr_z': Interpolation(),
            'fl_x': Interpolation(), 'fl_y': Interpolation(), 'fl_z': Interpolation(),
            'br_x': Interpolation(), 'br_y': Interpolation(), 'br_z': Interpolation(),
            'bl_x': Interpolation(), 'bl_y': Interpolation(), 'bl_z': Interpolation(),
        }
        
        # Dynamic stability variables
        self.leg_trans_x = 0.0
        self.leg_trans_y = 0.0
        self.leg_roll = 0.0
        self.leg_pitch = 0.0
        
        # Stability filters
        self.stability_filters = {
            'trans_x': Filter(0.5),
            'trans_y': Filter(0.5),
            'roll': Filter(0.4),
            'pitch': Filter(0.4),
        }
        
        # Control loop thread
        self.control_thread = None
        self.running = False
        
        logger.info("OpenDogV3 Controller initialized")
        
    def initialize_system(self):
        """Initialize all robot systems"""
        logger.info("Initializing robot systems...")
        
        # Initialize ODrives
        for odrive_id, odrive in self.odrives.items():
            odrive.connect()
            time.sleep(0.1)
            
        # Initialize IMU
        self.imu.initialize()
        
        # Apply initial motor offsets
        self.apply_initial_offsets()
        
        # Modify PID gains
        self.modify_gains()
        
        logger.info("System initialization complete")
        
    def apply_initial_offsets(self):
        """Apply initial position offsets to all motors"""
        logger.info("Applying initial motor offsets...")
        
        # Apply hip offsets
        self.odrives[1].set_position(0, self.motor_offsets[10])  # Right front hip
        self.odrives[1].set_position(1, self.motor_offsets[11])  # Right rear hip
        self.odrives[4].set_position(0, self.motor_offsets[40])  # Left front hip
        self.odrives[4].set_position(1, self.motor_offsets[41])  # Left rear hip
        
        # Apply shoulder and knee offsets
        self.odrives[2].set_position(1, self.motor_offsets[21])  # Right front shoulder
        self.odrives[3].set_position(1, self.motor_offsets[31])  # Right rear shoulder
        self.odrives[5].set_position(1, self.motor_offsets[51])  # Left front shoulder
        self.odrives[6].set_position(1, self.motor_offsets[61])  # Left rear shoulder
        
        self.odrives[2].set_position(0, self.motor_offsets[20])  # Right front knee
        self.odrives[3].set_position(0, self.motor_offsets[30])  # Right rear knee
        self.odrives[5].set_position(0, self.motor_offsets[50])  # Left front knee
        self.odrives[6].set_position(0, self.motor_offsets[60])  # Left rear knee
        
    def modify_gains(self):
        """Set PID gains for all motors"""
        logger.info("Setting PID gains...")
        
        # TODO: Implement actual gain setting commands to ODrives
        # This would involve sending serial commands like:
        # "w axis0.controller.config.pos_gain 20.0"
        # "w axis0.controller.config.vel_gain 0.16"
        # etc.
        
        logger.info("PID gains configured")
        
    def drive_joints(self, joint_id: int, position: float):
        """Drive individual joint to specified position"""
        
        if not self.remote_data.toggleTop:
            return  # Safety: only move if motor enable is on
            
        # Constrain position to safe limits
        position = max(-2.5, min(2.5, position))
        
        # Apply offsets and direction corrections for each joint
        if joint_id == 20:  # Right front knee
            self.odrives[2].set_position(0, position + self.motor_offsets[20])
        elif joint_id == 30:  # Right rear knee
            self.odrives[3].set_position(0, (-position) + self.motor_offsets[30])
        elif joint_id == 50:  # Left front knee
            self.odrives[5].set_position(0, (-position) + self.motor_offsets[50])
        elif joint_id == 60:  # Left rear knee
            self.odrives[6].set_position(0, position + self.motor_offsets[60])
            
        elif joint_id == 21:  # Right front shoulder
            self.odrives[2].set_position(1, (-position) + self.motor_offsets[21])
        elif joint_id == 31:  # Right rear shoulder
            self.odrives[3].set_position(1, position + self.motor_offsets[31])
        elif joint_id == 51:  # Left front shoulder
            self.odrives[5].set_position(1, position + self.motor_offsets[51])
        elif joint_id == 61:  # Left rear shoulder
            self.odrives[6].set_position(1, (-position) + self.motor_offsets[61])
            
        elif joint_id == 10:  # Right front hip
            self.odrives[1].set_position(0, position + self.motor_offsets[10])
        elif joint_id == 11:  # Right rear hip
            self.odrives[1].set_position(1, (-position) + self.motor_offsets[11])
        elif joint_id == 40:  # Left front hip
            self.odrives[4].set_position(0, position + self.motor_offsets[40])
        elif joint_id == 41:  # Left rear hip
            self.odrives[4].set_position(1, (-position) + self.motor_offsets[41])
            
    def kinematics(self, leg: int, x_in: float, y_in: float, z_in: float, 
                  roll: float, pitch: float, yaw: float, 
                  interp_on: int = 0, duration: int = 0) -> JointAngles:
        """
        Inverse kinematics calculation for one leg
        
        Args:
            leg: Leg number (1=front left, 2=front right, 3=back left, 4=back right)
            x_in: Forward/backward position (mm)
            y_in: Left/right position (mm) 
            z_in: Height position (mm)
            roll: Roll angle (degrees)
            pitch: Pitch angle (degrees)
            yaw: Yaw angle (degrees)
            interp_on: Interpolation enable flag
            duration: Interpolation duration (ms)
            
        Returns:
            JointAngles: Calculated joint angles
        """
        
        # Copy input values for local processing
        x, y, z = x_in, y_in, z_in
        
        # === YAW AXIS TRANSFORMATION ===
        yaw_angle = math.radians(yaw)
        
        # Add body offsets to work out radius from robot center
        if leg == 1:    # Front left
            y -= (self.BODY_WIDTH + self.HIP_OFFSET)
            x -= self.BODY_LENGTH
        elif leg == 2:  # Front right
            y += (self.BODY_WIDTH + self.HIP_OFFSET)
            x -= self.BODY_LENGTH
        elif leg == 3:  # Back left
            y -= (self.BODY_WIDTH + self.HIP_OFFSET)
            x += self.BODY_LENGTH
        elif leg == 4:  # Back right
            y += (self.BODY_WIDTH + self.HIP_OFFSET)
            x += self.BODY_LENGTH
            
        # Calculate existing angle and radius from center
        existing_angle = math.atan2(y, x)
        radius = y / math.sin(existing_angle) if math.sin(existing_angle) != 0 else 0
        
        # Calculate new position based on yaw demand
        demand_yaw = existing_angle + yaw_angle
        xx3 = radius * math.cos(demand_yaw)
        yy3 = radius * math.sin(demand_yaw)
        
        # Remove offsets to pivot around 0,0
        if leg == 1:    # Front left
            yy3 += (self.BODY_WIDTH + self.HIP_OFFSET)
            xx3 += self.BODY_LENGTH
        elif leg == 2:  # Front right
            yy3 -= (self.BODY_WIDTH + self.HIP_OFFSET)
            xx3 += self.BODY_LENGTH
        elif leg == 3:  # Back left
            yy3 += (self.BODY_WIDTH + self.HIP_OFFSET)
            xx3 -= self.BODY_LENGTH
        elif leg == 4:  # Back right
            yy3 -= (self.BODY_WIDTH + self.HIP_OFFSET)
            xx3 -= self.BODY_LENGTH
            
        # === PITCH AXIS TRANSFORMATION ===
        if leg in [1, 2]:  # Front legs
            pitch = -pitch
            xx3 = -xx3
            
        pitch_angle = math.radians(pitch)
        leg_diff_pitch = math.sin(pitch_angle) * self.BODY_LENGTH
        body_diff_pitch = math.cos(pitch_angle) * self.BODY_LENGTH
        
        zz2a = z - leg_diff_pitch
        foot_displacement_pitch = (body_diff_pitch - self.BODY_LENGTH) - xx3
        foot_displacement_angle_pitch = math.atan2(foot_displacement_pitch, zz2a)
        zz2 = zz2a / math.cos(foot_displacement_angle_pitch)
        xx1 = zz2a * math.tan(foot_displacement_angle_pitch)
        
        # === ROLL AXIS TRANSFORMATION ===
        roll_angle = math.radians(roll)
        leg_diff_roll = math.sin(roll_angle) * self.BODY_WIDTH
        body_diff_roll = math.cos(roll_angle) * self.BODY_WIDTH
        
        leg_diff_roll = zz2 - leg_diff_roll
        foot_displacement_roll = (((body_diff_roll - self.BODY_WIDTH) * -1) + self.HIP_OFFSET) - yy3
        foot_displacement_angle_roll = math.atan2(foot_displacement_roll, leg_diff_roll)
        zz1 = leg_diff_roll / math.cos(foot_displacement_angle_roll)
        yy1 = leg_diff_roll * math.tan(foot_displacement_angle_roll)
        
        # === HIP JOINT CALCULATION ===
        hip_offset = self.HIP_OFFSET
        if leg in [1, 4]:  # Left legs
            hip_offset *= -1
            yy1 *= -1
            
        yy1 += hip_offset
        hip_angle_1a = math.atan2(yy1, zz1)
        hip_angle_1_degrees = math.degrees(hip_angle_1a)
        hip_hyp = zz1 / math.cos(hip_angle_1a)
        
        # Second triangle for hip
        hip_angle_1b = math.asin(self.HIP_OFFSET / hip_hyp) if hip_hyp != 0 else 0
        hip_angle_1c = hip_angle_1a - hip_angle_1b
        hip_angle_1_degrees = math.degrees(hip_angle_1c)
        
        z3 = hip_hyp * math.cos(hip_angle_1b)
        
        # === SHOULDER AND KNEE CALCULATION ===
        # Calculate shoulder angle from forward/backward displacement
        shoulder_angle_2 = math.atan2(xx1, z3)
        shoulder_angle_2_degrees = math.degrees(shoulder_angle_2)
        z2 = z3 / math.cos(shoulder_angle_2)
        
        # Two-link inverse kinematics for shoulder and knee
        # Using law of cosines
        leg_length_2d = z2
        
        # Constrain to reachable workspace
        max_reach = self.SHIN_LENGTH + self.THIGH_LENGTH
        min_reach = abs(self.SHIN_LENGTH - self.THIGH_LENGTH)
        leg_length_2d = max(min_reach + 1, min(max_reach - 1, leg_length_2d))
        
        # Calculate shoulder angle
        cos_shoulder = (self.THIGH_LENGTH**2 + leg_length_2d**2 - self.SHIN_LENGTH**2) / (2 * self.THIGH_LENGTH * leg_length_2d)
        cos_shoulder = max(-1, min(1, cos_shoulder))  # Clamp to valid range
        shoulder_angle_1 = math.acos(cos_shoulder)
        shoulder_angle_1_degrees = math.degrees(shoulder_angle_1)
        
        # Calculate knee angle
        cos_knee = (self.THIGH_LENGTH**2 + self.SHIN_LENGTH**2 - leg_length_2d**2) / (2 * self.THIGH_LENGTH * self.SHIN_LENGTH)
        cos_knee = max(-1, min(1, cos_knee))  # Clamp to valid range
        knee_angle = math.pi - math.acos(cos_knee)
        knee_angle_degrees = math.degrees(knee_angle)
        
        # === CONVERT TO MOTOR COMMANDS ===
        # Convert angles to motor counts and send to motors
        shoulder_angle_1_counts = (shoulder_angle_1_degrees - 45) * self.MOTOR_CONVERSION
        shoulder_angle_2_counts = shoulder_angle_2_degrees * self.MOTOR_CONVERSION
        knee_angle_counts = (knee_angle_degrees - 90) * self.MOTOR_CONVERSION
        hip_angle_counts = hip_angle_1_degrees * self.MOTOR_CONVERSION
        
        # Apply leg-specific transformations and send to motors
        if leg == 1:    # Front left
            shoulder_angle_counts = shoulder_angle_1_counts + shoulder_angle_2_counts
            self.drive_joints(21, shoulder_angle_counts)  # Shoulder
            self.drive_joints(20, knee_angle_counts)      # Knee
            self.drive_joints(10, hip_angle_counts)       # Hip
            
        elif leg == 2:  # Front right  
            shoulder_angle_counts = shoulder_angle_1_counts + shoulder_angle_2_counts
            self.drive_joints(51, shoulder_angle_counts)  # Shoulder
            self.drive_joints(50, knee_angle_counts)      # Knee
            self.drive_joints(40, hip_angle_counts)       # Hip
            
        elif leg == 3:  # Back left
            shoulder_angle_counts = shoulder_angle_1_counts - shoulder_angle_2_counts
            self.drive_joints(61, shoulder_angle_counts)  # Shoulder
            self.drive_joints(60, knee_angle_counts)      # Knee
            self.drive_joints(41, hip_angle_counts)       # Hip
            
        elif leg == 4:  # Back right
            shoulder_angle_counts = shoulder_angle_1_counts - shoulder_angle_2_counts
            self.drive_joints(31, shoulder_angle_counts)  # Shoulder
            self.drive_joints(30, knee_angle_counts)      # Knee
            self.drive_joints(11, hip_angle_counts)       # Hip
            
        return JointAngles(
            hip=hip_angle_1_degrees,
            shoulder=shoulder_angle_1_degrees, 
            knee=knee_angle_degrees
        )
        
    def map_value(self, value: float, from_min: float, from_max: float, 
                  to_min: float, to_max: float) -> float:
        """Map value from one range to another"""
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min
        
    def constrain(self, value: float, min_val: float, max_val: float) -> float:
        """Constrain value within specified range"""
        return max(min_val, min(max_val, value))
        
    def update_filters(self):
        """Update all input filters"""
        self.filtered_data.RFB = self.filters['RFB'].update(self.remote_data.RFB)
        self.filtered_data.RLR = self.filters['RLR'].update(self.remote_data.RLR)
        self.filtered_data.RT = self.filters['RT'].update(self.remote_data.RT)
        self.filtered_data.LFB = self.filters['LFB'].update(self.remote_data.LFB)
        self.filtered_data.LLR = self.filters['LLR'].update(self.remote_data.LLR)
        self.filtered_data.LT = self.filters['LT'].update(self.remote_data.LT)
        
    def update_imu_stabilization(self):
        """Update IMU-based stabilization"""
        if not self.remote_data.toggleBottom:
            # IMU disabled
            self.leg_trans_x = 0.0
            self.leg_trans_y = 0.0
            self.leg_roll = 0.0 
            self.leg_pitch = 0.0
            return
            
        # Update IMU attitude
        self.imu.update_attitude(self.control_period)
        
        # Calculate stabilization values
        self.leg_trans_x = self.imu.pitch * -2.0
        self.leg_trans_y = self.imu.roll * -2.0
        self.leg_roll = self.imu.roll * -0.5
        self.leg_pitch = self.imu.pitch * 0.5
        
        # Apply filters
        self.leg_trans_x = self.stability_filters['trans_x'].update(self.leg_trans_x)
        self.leg_trans_y = self.stability_filters['trans_y'].update(self.leg_trans_y)
        self.leg_roll = self.stability_filters['roll'].update(self.leg_roll)
        self.leg_pitch = self.stability_filters['pitch'].update(self.leg_pitch)
        
    def home_position(self):
        """Move robot to home/stand position"""
        logger.info("Moving to home position")
        
        offset = 70
        home_height = 270
        
        # Move all legs to home position
        self.kinematics(1, -offset, 0, home_height, 0, 0, 0)  # Front left
        self.kinematics(2, -offset, 0, home_height, 0, 0, 0)  # Front right  
        self.kinematics(3, offset, 0, home_height, 0, 0, 0)   # Back left
        self.kinematics(4, offset, 0, home_height, 0, 0, 0)   # Back right
        
    def inverse_kinematics_demo(self):
        """Inverse kinematics demonstration mode"""
        
        # Scale remote control inputs to appropriate ranges
        rfb = self.map_value(self.remote_data.RFB, -462, 462, -100, 100)
        rlr = self.map_value(self.remote_data.RLR, -462, 462, -100, 100)
        rt = self.map_value(self.remote_data.RT, -462, 462, 240, 440)
        rt = self.constrain(rt, 240, 380)
        lfb = self.map_value(self.remote_data.LFB, -462, 462, -15, 15)
        llr = self.map_value(self.remote_data.LLR, -462, 462, -15, 15)
        lt = self.map_value(self.remote_data.LT, -462, 462, -20, 20)
        
        # Update remote data for filtering
        self.remote_data.RFB = rfb
        self.remote_data.RLR = rlr
        self.remote_data.RT = rt
        self.remote_data.LFB = lfb
        self.remote_data.LLR = llr
        self.remote_data.LT = lt
        
        # Apply filters
        self.update_filters()
        
        # Apply same position to all legs
        for leg in range(1, 5):
            self.kinematics(
                leg, 
                self.filtered_data.RFB,  # X position
                self.filtered_data.RLR,  # Y position  
                self.filtered_data.RT,   # Z position
                self.filtered_data.LLR,  # Roll
                self.filtered_data.LFB,  # Pitch
                self.filtered_data.LT    # Yaw
            )
            
    def simple_walking_mode(self):
        """Simple walking gait implementation"""
        
        # Scale inputs for walking
        rfb = self.map_value(self.remote_data.RFB, -462, 462, -50, 50)   # mm
        rlr = self.map_value(self.remote_data.RLR, -462, 462, -25, 25)   # mm  
        lt = self.map_value(self.remote_data.LT, -462, 462, -25, 25)     # degrees
        
        # Apply filters
        self.remote_data.RFB = rfb
        self.remote_data.RLR = rlr
        self.remote_data.LT = lt
        
        self.filtered_data.RFB = self.filters['RFB'].update(rfb)
        self.filtered_data.RLR = self.filters['RLR'].update(rlr)
        self.filtered_data.LT = self.filters['LT'].update(lt)
        
        # Base leg height
        long_leg = 340
        short_leg = 280
        
        # Update gait timers
        self.gait_timer1 += 3  # FB gait
        self.gait_timer2 += 3  # LR gait
        self.gait_timer3 += 4  # Yaw gait
        
        # Wrap timers
        if self.gait_timer1 >= 360:
            self.gait_timer1 = 0
        if self.gait_timer2 >= 360:
            self.gait_timer2 = 0
        if self.gait_timer3 >= 360:
            self.gait_timer3 = 0
            
        # Calculate gait phases for each leg (trot gait)
        # Front right and back left move together
        # Front left and back right move together
        
        # Leg height modulation (up/down movement)
        fr_height = long_leg + 30 * math.sin(math.radians(self.gait_timer1))
        fl_height = long_leg + 30 * math.sin(math.radians(self.gait_timer1 + 180))
        br_height = long_leg + 30 * math.sin(math.radians(self.gait_timer1 + 180))  
        bl_height = long_leg + 30 * math.sin(math.radians(self.gait_timer1))
        
        # Forward/backward movement
        stride_length = 40
        fr_x = self.filtered_data.RFB + stride_length * math.sin(math.radians(self.gait_timer1))
        fl_x = self.filtered_data.RFB + stride_length * math.sin(math.radians(self.gait_timer1 + 180))
        br_x = self.filtered_data.RFB + stride_length * math.sin(math.radians(self.gait_timer1 + 180))
        bl_x = self.filtered_data.RFB + stride_length * math.sin(math.radians(self.gait_timer1))
        
        # Left/right movement  
        fr_y = self.filtered_data.RLR
        fl_y = self.filtered_data.RLR
        br_y = self.filtered_data.RLR
        bl_y = self.filtered_data.RLR
        
        # Apply IMU stabilization
        self.update_imu_stabilization()
        
        # Send commands to legs with stabilization
        self.kinematics(
            1, 
            fr_x - self.leg_trans_x, 
            fr_y - self.leg_trans_y, 
            fr_height,
            self.leg_roll, 
            self.leg_pitch, 
            0
        )
        
        self.kinematics(
            2,
            fl_x - self.leg_trans_x,
            fl_y - self.leg_trans_y, 
            fl_height,
            self.leg_roll,
            self.leg_pitch,
            0
        )
        
        self.kinematics(
            3,
            bl_x - self.leg_trans_x,
            bl_y - self.leg_trans_y,
            bl_height, 
            self.leg_roll,
            self.leg_pitch,
            0
        )
        
        self.kinematics(
            4,
            br_x - self.leg_trans_x,
            br_y - self.leg_trans_y,
            br_height,
            self.leg_roll, 
            self.leg_pitch,
            0
        )
        
    def control_loop(self):
        """Main control loop running at 100Hz"""
        logger.info("Starting control loop at 100Hz")
        
        while self.running:
            start_time = time.time()
            
            # TODO: Read remote control data
            # self.read_remote_data()
            
            # Update IMU if enabled
            if self.remote_data.toggleBottom:
                self.imu.read_data()
                
            # Mode selection and execution
            if self.run_mode == 0:
                # Standby mode
                pass
                
            elif self.run_mode == 1:
                # Inverse kinematics demo
                self.inverse_kinematics_demo()
                
            elif self.run_mode == 2:
                # Walking mode
                self.simple_walking_mode()
                
            elif self.run_mode == 10:
                # Home position
                self.home_position()
                
            # Calculate sleep time to maintain 100Hz
            elapsed = time.time() - start_time
            sleep_time = max(0, self.control_period - elapsed)
            time.sleep(sleep_time)
            
    def start(self):
        """Start the robot controller"""
        logger.info("Starting OpenDogV3 Controller")
        
        self.initialize_system()
        
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        logger.info("Controller started successfully")
        
    def stop(self):
        """Stop the robot controller"""
        logger.info("Stopping OpenDogV3 Controller")
        
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
            
        logger.info("Controller stopped")
        
    def set_mode(self, mode: int):
        """Set control mode"""
        self.run_mode = mode
        logger.info(f"Mode set to {mode}")
        
    def set_remote_data(self, data: RemoteControlData):
        """Update remote control data"""
        self.remote_data = data


# Demo usage
if __name__ == "__main__":
    """
    Demo usage of the OpenDogV3 Controller
    """
    
    # Create controller instance
    robot = OpenDogV3Controller()
    
    try:
        # Start the controller
        robot.start()
        
        # Demo sequence
        logger.info("Starting demo sequence...")
        
        # Home position
        robot.set_mode(10)
        time.sleep(3)
        
        # Inverse kinematics demo
        robot.set_mode(1)
        
        # Simulate remote control inputs
        demo_data = RemoteControlData()
        demo_data.toggleTop = 1  # Enable motors
        demo_data.toggleBottom = 1  # Enable IMU
        
        for i in range(1000):  # Run for 10 seconds at 100Hz
            # Simulate stick movements
            demo_data.RFB = 50 * math.sin(i * 0.01)  # Forward/back oscillation
            demo_data.RLR = 30 * math.cos(i * 0.01)  # Left/right oscillation
            demo_data.RT = 300 + 50 * math.sin(i * 0.005)  # Height variation
            
            robot.set_remote_data(demo_data)
            time.sleep(0.01)  # 100Hz
            
        # Walking demo
        logger.info("Starting walking demo...")
        robot.set_mode(2)
        
        for i in range(500):  # Run for 5 seconds
            demo_data.RFB = 20  # Forward motion
            demo_data.RLR = 0   # Straight
            demo_data.LT = 0    # No yaw
            
            robot.set_remote_data(demo_data)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        logger.info("Demo interrupted by user")
        
    finally:
        # Stop the controller
        robot.stop()
        logger.info("Demo completed")
