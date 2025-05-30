#!/usr/bin/env python3
"""
ODrive Motor Control Interface for OpenDog V3
Converted from Arduino C++ ODriveInit.ino

This module handles communication with 6 ODrive motor controllers,
each managing 2 joints (12 total joints: 4 legs × 3 joints per leg).

ODrive Configuration:
- ODrive 1: Right front hip, Right rear hip
- ODrive 2: Right front knee, Right front shoulder  
- ODrive 3: Right rear knee, Right rear shoulder
- ODrive 4: Left front hip, Left rear hip
- ODrive 5: Left front knee, Left front shoulder
- ODrive 6: Left rear knee, Left rear shoulder

Author: Converted from Arduino C++ to Python
Date: 2024
"""

import logging
import time
import serial
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AxisState(Enum):
    """ODrive axis states"""
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11

class JointType(Enum):
    """Joint types with their ID mappings"""
    # Right leg joints
    RIGHT_FRONT_HIP = 10
    RIGHT_REAR_HIP = 11
    RIGHT_FRONT_KNEE = 20
    RIGHT_FRONT_SHOULDER = 21
    RIGHT_REAR_KNEE = 30
    RIGHT_REAR_SHOULDER = 31
    
    # Left leg joints
    LEFT_FRONT_HIP = 40
    LEFT_REAR_HIP = 41
    LEFT_FRONT_KNEE = 50
    LEFT_FRONT_SHOULDER = 51
    LEFT_REAR_KNEE = 60
    LEFT_REAR_SHOULDER = 61

@dataclass
class ODriveGains:
    """ODrive PID controller gains"""
    pos_gain_knee: float = 15.0
    pos_gain_hip: float = 70.0
    pos_gain_shoulder: float = 15.0
    vel_gain: float = 0.1
    vel_integrator_gain: float = 0.1

@dataclass
class MotorLimits:
    """Motor position and velocity limits"""
    position_limit: float = 2.5  # radians
    velocity_limit: float = 6000.0  # rad/s
    current_limit: float = 20.0  # A

class ODriveController:
    """Individual ODrive controller interface"""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize ODrive controller
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0', 'COM3')
            baudrate: Serial communication speed
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection: Optional[serial.Serial] = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """
        Connect to ODrive controller
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.is_connected = True
            logger.info(f"Connected to ODrive on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ODrive on {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from ODrive controller"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.is_connected = False
        logger.info(f"Disconnected from ODrive on {self.port}")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to ODrive
        
        Args:
            command: Command string to send
            
        Returns:
            True if command sent successfully
        """
        if not self.is_connected or not self.serial_connection:
            logger.error("ODrive not connected")
            return False
        
        try:
            command_with_newline = command + '\n'
            self.serial_connection.write(command_with_newline.encode())
            logger.debug(f"Sent command: {command}")
            return True
        except Exception as e:
            logger.error(f"Failed to send command '{command}': {e}")
            return False
    
    def set_position(self, axis: int, position: float) -> bool:
        """
        Set motor position
        
        Args:
            axis: Motor axis (0 or 1)
            position: Target position in radians
            
        Returns:
            True if command sent successfully
        """
        command = f"c {axis} {position}"
        return self.send_command(command)
    
    def run_state(self, axis: int, state: AxisState, wait: bool = False) -> bool:
        """
        Set axis state
        
        Args:
            axis: Motor axis (0 or 1)
            state: Target axis state
            wait: Whether to wait for state change completion
            
        Returns:
            True if command sent successfully
        """
        command = f"w axis{axis}.requested_state {state.value}"
        success = self.send_command(command)
        
        if wait and success:
            # In real implementation, would check axis state
            time.sleep(0.1)
        
        return success
    
    def set_current_limit(self, axis: int, current_limit: float) -> bool:
        """Set motor current limit"""
        command = f"w axis{axis}.motor.config.current_lim {current_limit}"
        return self.send_command(command)
    
    def set_velocity_limit(self, axis: int, vel_limit: float) -> bool:
        """Set motor velocity limit"""
        command = f"w axis{axis}.controller.config.vel_limit {vel_limit}"
        return self.send_command(command)
    
    def set_pos_gain(self, axis: int, pos_gain: float) -> bool:
        """Set position controller gain"""
        command = f"w axis{axis}.controller.config.pos_gain {pos_gain}"
        return self.send_command(command)
    
    def set_vel_gain(self, axis: int, vel_gain: float) -> bool:
        """Set velocity controller gain"""
        command = f"w axis{axis}.controller.config.vel_gain {vel_gain}"
        return self.send_command(command)
    
    def set_vel_integrator_gain(self, axis: int, integrator_gain: float) -> bool:
        """Set velocity integrator gain"""
        command = f"w axis{axis}.controller.config.vel_integrator_gain {integrator_gain}"
        return self.send_command(command)

class ODriveInterface:
    """
    Main ODrive interface managing all 6 ODrive controllers
    Converted from Arduino ODriveInit.ino functions
    """
    
    def __init__(self, serial_ports: List[str], joint_offsets: Dict[int, float]):
        """
        Initialize ODrive interface
        
        Args:
            serial_ports: List of 6 serial ports for ODrives 1-6
            joint_offsets: Dictionary mapping joint IDs to their offset values
        """
        self.joint_offsets = joint_offsets
        self.gains = ODriveGains()
        self.limits = MotorLimits()
        self.motor_enable = False  # Motor enable state from remote
        
        # Initialize ODrive controllers
        self.odrives = {}
        for i, port in enumerate(serial_ports, 1):
            self.odrives[i] = ODriveController(port)
        
        # Joint to ODrive mapping
        self.joint_mapping = {
            # Right leg
            10: (1, 0),  # Right front hip -> ODrive1, Axis0
            11: (1, 1),  # Right rear hip -> ODrive1, Axis1
            20: (2, 0),  # Right front knee -> ODrive2, Axis0
            21: (2, 1),  # Right front shoulder -> ODrive2, Axis1
            30: (3, 0),  # Right rear knee -> ODrive3, Axis0
            31: (3, 1),  # Right rear shoulder -> ODrive3, Axis1
            
            # Left leg
            40: (4, 0),  # Left front hip -> ODrive4, Axis0
            41: (4, 1),  # Left rear hip -> ODrive4, Axis1
            50: (5, 0),  # Left front knee -> ODrive5, Axis0
            51: (5, 1),  # Left front shoulder -> ODrive5, Axis1
            60: (6, 0),  # Left rear knee -> ODrive6, Axis0
            61: (6, 1),  # Left rear shoulder -> ODrive6, Axis1
        }
        
        # Direction multipliers for consistent joint directions
        self.direction_multipliers = {
            10: 1,   # Right front hip
            11: -1,  # Right rear hip (reversed)
            20: 1,   # Right front knee
            21: -1,  # Right front shoulder (reversed)
            30: -1,  # Right rear knee (reversed)
            31: 1,   # Right rear shoulder
            40: 1,   # Left front hip
            41: -1,  # Left rear hip (reversed)
            50: -1,  # Left front knee (reversed)
            51: 1,   # Left front shoulder
            60: 1,   # Left rear knee
            61: -1,  # Left rear shoulder (reversed)
        }
        
        logger.info("ODrive interface initialized")
    
    def connect_all(self) -> bool:
        """
        Connect to all ODrive controllers
        
        Returns:
            True if all connections successful
        """
        logger.info("Connecting to all ODrive controllers...")
        success = True
        
        for odrive_id, odrive in self.odrives.items():
            if not odrive.connect():
                success = False
                logger.error(f"Failed to connect to ODrive {odrive_id}")
            else:
                logger.info(f"ODrive {odrive_id} connected successfully")
        
        return success
    
    def disconnect_all(self):
        """Disconnect from all ODrive controllers"""
        logger.info("Disconnecting from all ODrive controllers...")
        for odrive in self.odrives.values():
            odrive.disconnect()
    
    def odrive_init(self) -> bool:
        """
        Initialize all ODrives - equivalent to OdriveInit1() function
        
        Returns:
            True if initialization successful
        """
        logger.info("Initializing ODrives...")
        success = True
        
        for odrive_id in range(1, 7):
            logger.info(f"Initializing ODrive {odrive_id}")
            odrive = self.odrives[odrive_id]
            
            if not odrive.is_connected:
                logger.error(f"ODrive {odrive_id} not connected")
                success = False
                continue
            
            # Configure both axes on each ODrive
            for axis in range(2):
                # Set current limit
                if not odrive.set_current_limit(axis, self.limits.current_limit):
                    success = False
                
                # Request closed loop control state
                if not odrive.run_state(axis, AxisState.CLOSED_LOOP_CONTROL, wait=False):
                    success = False
                
                logger.info(f"ODrive {odrive_id} Axis {axis} initialized")
        
        if success:
            logger.info("All ODrives initialized successfully")
        else:
            logger.error("ODrive initialization completed with errors")
        
        return success
    
    def modify_gains(self) -> bool:
        """
        Modify controller gains - equivalent to modifyGains() function
        
        Returns:
            True if all gains set successfully
        """
        logger.info("Modifying ODrive gains...")
        success = True
        
        # Gain configuration for each ODrive
        gain_config = {
            1: [(0, self.gains.pos_gain_hip), (1, self.gains.pos_gain_hip)],          # Hips
            2: [(0, self.gains.pos_gain_knee), (1, self.gains.pos_gain_shoulder)],   # Knee + Shoulder
            3: [(0, self.gains.pos_gain_knee), (1, self.gains.pos_gain_shoulder)],   # Knee + Shoulder
            4: [(0, self.gains.pos_gain_hip), (1, self.gains.pos_gain_hip)],         # Hips
            5: [(0, self.gains.pos_gain_knee), (1, self.gains.pos_gain_shoulder)],   # Knee + Shoulder
            6: [(0, self.gains.pos_gain_knee), (1, self.gains.pos_gain_shoulder)]    # Knee + Shoulder
        }
        
        for odrive_id, axis_gains in gain_config.items():
            odrive = self.odrives[odrive_id]
            
            for axis, pos_gain in axis_gains:
                # Set position gain
                if not odrive.set_pos_gain(axis, pos_gain):
                    success = False
                
                # Set velocity gain
                if not odrive.set_vel_gain(axis, self.gains.vel_gain):
                    success = False
                
                # Set velocity integrator gain
                if not odrive.set_vel_integrator_gain(axis, self.gains.vel_integrator_gain):
                    success = False
        
        if success:
            logger.info("ODrive gains modified successfully")
        else:
            logger.error("Failed to modify some ODrive gains")
        
        return success
    
    def apply_offsets_hips(self) -> bool:
        """
        Apply initial offsets to hip joints - equivalent to applyOffsets1()
        
        Returns:
            True if all offsets applied successfully
        """
        logger.info("Applying hip joint offsets...")
        success = True
        
        hip_joints = [10, 11, 40, 41]  # All hip joints
        
        for joint_id in hip_joints:
            odrive_id, axis = self.joint_mapping[joint_id]
            offset = self.joint_offsets.get(joint_id, 0.0)
            
            if not self.odrives[odrive_id].set_position(axis, offset):
                success = False
                logger.error(f"Failed to apply offset to joint {joint_id}")
            else:
                logger.debug(f"Applied offset {offset} to joint {joint_id}")
        
        if success:
            logger.info("Hip joint offsets applied successfully")
        
        return success
    
    def apply_offsets_knees_shoulders(self) -> bool:
        """
        Apply initial offsets to knee and shoulder joints - equivalent to applyOffsets2()
        
        Returns:
            True if all offsets applied successfully
        """
        logger.info("Applying knee and shoulder joint offsets...")
        success = True
        
        knee_shoulder_joints = [20, 21, 30, 31, 50, 51, 60, 61]  # All knee and shoulder joints
        
        for joint_id in knee_shoulder_joints:
            odrive_id, axis = self.joint_mapping[joint_id]
            offset = self.joint_offsets.get(joint_id, 0.0)
            
            if not self.odrives[odrive_id].set_position(axis, offset):
                success = False
                logger.error(f"Failed to apply offset to joint {joint_id}")
            else:
                logger.debug(f"Applied offset {offset} to joint {joint_id}")
        
        if success:
            logger.info("Knee and shoulder joint offsets applied successfully")
        
        return success
    
    def drive_joint(self, joint_id: int, position: float) -> bool:
        """
        Drive individual joint to target position - equivalent to driveJoints() function
        
        Args:
            joint_id: Joint ID (10, 11, 20, 21, etc.)
            position: Target position in radians
            
        Returns:
            True if position command sent successfully
        """
        # Only drive joints if motor enable is active
        if not self.motor_enable:
            logger.debug("Motor enable off, skipping joint drive command")
            return False
        
        # Check if joint ID is valid
        if joint_id not in self.joint_mapping:
            logger.error(f"Invalid joint ID: {joint_id}")
            return False
        
        # Apply position limits
        constrained_pos = max(-self.limits.position_limit, 
                            min(self.limits.position_limit, position))
        
        if abs(constrained_pos - position) > 0.001:
            logger.warning(f"Joint {joint_id} position {position} constrained to {constrained_pos}")
        
        # Get ODrive and axis for this joint
        odrive_id, axis = self.joint_mapping[joint_id]
        
        # Apply direction multiplier and offset
        direction_multiplier = self.direction_multipliers[joint_id]
        offset = self.joint_offsets.get(joint_id, 0.0)
        final_position = (constrained_pos * direction_multiplier) + offset
        
        # Send position command
        success = self.odrives[odrive_id].set_position(axis, final_position)
        
        if success:
            logger.debug(f"Joint {joint_id} set to position {final_position} "
                        f"(input: {position}, multiplier: {direction_multiplier}, offset: {offset})")
        else:
            logger.error(f"Failed to set joint {joint_id} position")
        
        return success
    
    def drive_all_joints(self, joint_positions: Dict[int, float]) -> bool:
        """
        Drive all joints to target positions
        
        Args:
            joint_positions: Dictionary mapping joint IDs to target positions
            
        Returns:
            True if all position commands sent successfully
        """
        if not self.motor_enable:
            logger.debug("Motor enable off, skipping all joint drive commands")
            return False
        
        success = True
        for joint_id, position in joint_positions.items():
            if not self.drive_joint(joint_id, position):
                success = False
        
        return success
    
    def set_motor_enable(self, enable: bool):
        """
        Set motor enable state (from remote control)
        
        Args:
            enable: True to enable motors, False to disable
        """
        self.motor_enable = enable
        logger.info(f"Motor enable set to: {enable}")
    
    def emergency_stop(self) -> bool:
        """
        Emergency stop all motors
        
        Returns:
            True if all motors stopped successfully
        """
        logger.warning("Emergency stop activated!")
        success = True
        
        for odrive_id in range(1, 7):
            odrive = self.odrives[odrive_id]
            for axis in range(2):
                if not odrive.run_state(axis, AxisState.IDLE):
                    success = False
        
        self.motor_enable = False
        
        if success:
            logger.info("Emergency stop completed successfully")
        else:
            logger.error("Emergency stop completed with errors")
        
        return success
    
    def get_joint_info(self, joint_id: int) -> Dict:
        """
        Get information about a specific joint
        
        Args:
            joint_id: Joint ID
            
        Returns:
            Dictionary with joint information
        """
        if joint_id not in self.joint_mapping:
            return {}
        
        odrive_id, axis = self.joint_mapping[joint_id]
        
        return {
            'joint_id': joint_id,
            'odrive_id': odrive_id,
            'axis': axis,
            'offset': self.joint_offsets.get(joint_id, 0.0),
            'direction_multiplier': self.direction_multipliers[joint_id],
            'position_limit': self.limits.position_limit,
            'joint_type': self._get_joint_type_name(joint_id)
        }
    
    def _get_joint_type_name(self, joint_id: int) -> str:
        """Get human-readable joint type name"""
        joint_names = {
            10: "Right Front Hip",
            11: "Right Rear Hip", 
            20: "Right Front Knee",
            21: "Right Front Shoulder",
            30: "Right Rear Knee",
            31: "Right Rear Shoulder",
            40: "Left Front Hip",
            41: "Left Rear Hip",
            50: "Left Front Knee", 
            51: "Left Front Shoulder",
            60: "Left Rear Knee",
            61: "Left Rear Shoulder"
        }
        return joint_names.get(joint_id, f"Unknown Joint {joint_id}")
    
    def get_status(self) -> Dict:
        """
        Get overall ODrive interface status
        
        Returns:
            Dictionary with status information
        """
        connected_count = sum(1 for odrive in self.odrives.values() if odrive.is_connected)
        
        return {
            'total_odrives': len(self.odrives),
            'connected_odrives': connected_count,
            'motor_enable': self.motor_enable,
            'all_connected': connected_count == len(self.odrives),
            'gains': {
                'pos_gain_knee': self.gains.pos_gain_knee,
                'pos_gain_hip': self.gains.pos_gain_hip,
                'pos_gain_shoulder': self.gains.pos_gain_shoulder,
                'vel_gain': self.gains.vel_gain,
                'vel_integrator_gain': self.gains.vel_integrator_gain
            },
            'limits': {
                'position_limit': self.limits.position_limit,
                'velocity_limit': self.limits.velocity_limit,
                'current_limit': self.limits.current_limit
            }
        }

# Example usage and testing
if __name__ == "__main__":
    # Example joint offsets (these would be calibrated for each robot)
    joint_offsets = {
        10: 0.1,   # Right front hip
        11: -0.05, # Right rear hip
        20: 0.2,   # Right front knee
        21: 0.0,   # Right front shoulder
        30: -0.1,  # Right rear knee
        31: 0.15,  # Right rear shoulder
        40: 0.05,  # Left front hip
        41: 0.0,   # Left rear hip
        50: -0.2,  # Left front knee
        51: 0.1,   # Left front shoulder
        60: 0.0,   # Left rear knee
        61: -0.15  # Left rear shoulder
    }
    
    # Example serial ports (would be actual device paths)
    serial_ports = [
        '/dev/ttyUSB0',  # ODrive 1
        '/dev/ttyUSB1',  # ODrive 2
        '/dev/ttyUSB2',  # ODrive 3
        '/dev/ttyUSB3',  # ODrive 4
        '/dev/ttyUSB4',  # ODrive 5
        '/dev/ttyUSB5'   # ODrive 6
    ]
    
    # Initialize ODrive interface
    odrive_interface = ODriveInterface(serial_ports, joint_offsets)
    
    # Print status
    print("ODrive Interface Status:")
    status = odrive_interface.get_status()
    for key, value in status.items():
        print(f"  {key}: {value}")
    
    # Print joint information
    print("\nJoint Information:")
    for joint_id in [10, 20, 21, 30, 31]:  # Sample joints
        info = odrive_interface.get_joint_info(joint_id)
        if info:
            print(f"  Joint {joint_id} ({info['joint_type']}): "
                  f"ODrive {info['odrive_id']}, Axis {info['axis']}, "
                  f"Offset {info['offset']}, Direction {info['direction_multiplier']}")
