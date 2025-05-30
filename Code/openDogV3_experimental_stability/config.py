#!/usr/bin/env python3
"""
Configuration Management Module for OpenDog V3
Centralized configuration for robot parameters, joint offsets, and system settings

This module provides:
- Robot physical dimensions and limits
- Joint offset calibration values
- Motor control parameters
- Communication settings
- System timing parameters

Author: Python configuration module for OpenDog V3
Date: 2024
"""

import json
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class RobotPhysicalConfig:
    """Robot physical dimensions and constraints"""
    # Body dimensions (mm)
    hip_offset: float = 108.0       # Distance from hip pivot to leg center
    body_width: float = 59.0        # Half distance between hip pivots
    body_length: float = 272.0      # Half distance between shoulder pivots
    shin_length: float = 200.0      # Lower leg segment length
    thigh_length: float = 200.0     # Upper leg segment length
    
    # Robot limits
    max_leg_reach: float = 390.0    # Maximum leg extension (mm)
    min_leg_reach: float = 200.0    # Minimum leg extension (mm)
    safe_leg_height: float = 300.0  # Safe default leg height (mm)
    
    # Motion limits (degrees)
    max_roll: float = 30.0          # Maximum roll angle
    max_pitch: float = 30.0         # Maximum pitch angle
    max_yaw: float = 45.0           # Maximum yaw angle
    
    # Speed limits (mm/s and deg/s)
    max_translation_speed: float = 100.0  # mm/s
    max_rotation_speed: float = 30.0      # deg/s

@dataclass
class MotorConfig:
    """Motor control parameters"""
    # Current limits (A)
    current_limit: float = 20.0
    
    # Velocity limits (rad/s)
    velocity_limit: float = 6000.0
    
    # Position limits (rad)
    position_limit: float = 2.5
    
    # PID gains
    pos_gain_hip: float = 70.0
    pos_gain_knee: float = 15.0
    pos_gain_shoulder: float = 15.0
    vel_gain: float = 0.1
    vel_integrator_gain: float = 0.1
    
    # Motor direction multipliers for consistent joint directions
    direction_multipliers: Dict[int, int] = None
    
    def __post_init__(self):
        if self.direction_multipliers is None:
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

@dataclass
class JointOffsetsConfig:
    """Joint offset calibration values in radians"""
    # Right leg joints
    right_front_hip: float = 0.0        # Joint ID 10
    right_rear_hip: float = 0.0         # Joint ID 11
    right_front_knee: float = 0.0       # Joint ID 20
    right_front_shoulder: float = 0.0   # Joint ID 21
    right_rear_knee: float = 0.0        # Joint ID 30
    right_rear_shoulder: float = 0.0    # Joint ID 31
    
    # Left leg joints
    left_front_hip: float = 0.0         # Joint ID 40
    left_rear_hip: float = 0.0          # Joint ID 41
    left_front_knee: float = 0.0        # Joint ID 50
    left_front_shoulder: float = 0.0    # Joint ID 51
    left_rear_knee: float = 0.0         # Joint ID 60
    left_rear_shoulder: float = 0.0     # Joint ID 61
    
    def to_dict(self) -> Dict[int, float]:
        """Convert to dictionary mapping joint IDs to offset values"""
        return {
            10: self.right_front_hip,
            11: self.right_rear_hip,
            20: self.right_front_knee,
            21: self.right_front_shoulder,
            30: self.right_rear_knee,
            31: self.right_rear_shoulder,
            40: self.left_front_hip,
            41: self.left_rear_hip,
            50: self.left_front_knee,
            51: self.left_front_shoulder,
            60: self.left_rear_knee,
            61: self.left_rear_shoulder,
        }
    
    def from_dict(self, offsets: Dict[int, float]):
        """Load from dictionary mapping joint IDs to offset values"""
        self.right_front_hip = offsets.get(10, 0.0)
        self.right_rear_hip = offsets.get(11, 0.0)
        self.right_front_knee = offsets.get(20, 0.0)
        self.right_front_shoulder = offsets.get(21, 0.0)
        self.right_rear_knee = offsets.get(30, 0.0)
        self.right_rear_shoulder = offsets.get(31, 0.0)
        self.left_front_hip = offsets.get(40, 0.0)
        self.left_rear_hip = offsets.get(41, 0.0)
        self.left_front_knee = offsets.get(50, 0.0)
        self.left_front_shoulder = offsets.get(51, 0.0)
        self.left_rear_knee = offsets.get(60, 0.0)
        self.left_rear_shoulder = offsets.get(61, 0.0)

@dataclass
class CommunicationConfig:
    """Communication interface settings"""
    # ODrive serial ports
    odrive_ports: List[str] = None
    odrive_baudrate: int = 115200
    odrive_timeout: float = 1.0
    
    # Remote control settings
    remote_port: str = "/dev/ttyUSB6"
    remote_baudrate: int = 115200
    remote_timeout: float = 0.1
    
    # IMU settings
    imu_port: str = "/dev/ttyUSB7"
    imu_baudrate: int = 115200
    imu_timeout: float = 0.1
    
    def __post_init__(self):
        if self.odrive_ports is None:
            # Default ODrive ports
            self.odrive_ports = [
                '/dev/ttyUSB0',  # ODrive 1
                '/dev/ttyUSB1',  # ODrive 2
                '/dev/ttyUSB2',  # ODrive 3
                '/dev/ttyUSB3',  # ODrive 4
                '/dev/ttyUSB4',  # ODrive 5
                '/dev/ttyUSB5'   # ODrive 6
            ]

@dataclass
class ControlSystemConfig:
    """Control system timing and behavior settings"""
    # Main control loop
    main_loop_frequency: float = 100.0  # Hz
    main_loop_period: float = 0.01      # seconds
    
    # Input processing
    input_deadzone: float = 100.0       # Stick deadzone value
    input_filter_alpha: float = 0.1     # Low-pass filter coefficient
    input_max_value: float = 1000.0     # Maximum stick input value
    
    # Interpolation
    interpolation_settle_time: float = 0.3  # seconds
    default_interpolation_duration: int = 100  # milliseconds
    
    # IMU filtering
    imu_filter_alpha: float = 0.98      # Complementary filter coefficient
    
    # Safety limits
    emergency_stop_enabled: bool = True
    position_limit_enabled: bool = True
    velocity_limit_enabled: bool = True
    
    # Gait parameters
    trot_cycle_time: float = 1.0        # seconds
    trot_duty_cycle: float = 0.6        # fraction of cycle in stance
    step_height: float = 50.0           # mm
    default_step_distance: float = 80.0 # mm

@dataclass
class OpenDogV3Config:
    """Complete OpenDog V3 configuration"""
    physical: RobotPhysicalConfig
    motor: MotorConfig
    joint_offsets: JointOffsetsConfig
    communication: CommunicationConfig
    control_system: ControlSystemConfig
    
    # System metadata
    config_version: str = "1.0"
    robot_name: str = "OpenDog V3"
    description: str = "OpenDog V3 Quadruped Robot Configuration"

class ConfigManager:
    """Configuration manager for loading, saving, and validating robot config"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration manager
        
        Args:
            config_path: Path to configuration file (optional)
        """
        self.config_path = Path(config_path) if config_path else Path("opendog_v3_config.json")
        self.config: Optional[OpenDogV3Config] = None
        
        logger.info(f"Configuration manager initialized with path: {self.config_path}")
    
    def load_config(self) -> OpenDogV3Config:
        """
        Load configuration from file or create default
        
        Returns:
            Loaded or default configuration
        """
        if self.config_path.exists():
            try:
                with open(self.config_path, 'r') as f:
                    config_dict = json.load(f)
                
                self.config = self._dict_to_config(config_dict)
                logger.info(f"Configuration loaded from {self.config_path}")
                
            except Exception as e:
                logger.error(f"Failed to load config from {self.config_path}: {e}")
                logger.info("Creating default configuration")
                self.config = self._create_default_config()
        else:
            logger.info("Config file not found, creating default configuration")
            self.config = self._create_default_config()
        
        return self.config
    
    def save_config(self, config: Optional[OpenDogV3Config] = None) -> bool:
        """
        Save configuration to file
        
        Args:
            config: Configuration to save (uses current if None)
            
        Returns:
            True if saved successfully
        """
        if config is not None:
            self.config = config
        
        if self.config is None:
            logger.error("No configuration to save")
            return False
        
        try:
            config_dict = self._config_to_dict(self.config)
            
            # Create directory if it doesn't exist
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(self.config_path, 'w') as f:
                json.dump(config_dict, f, indent=4)
            
            logger.info(f"Configuration saved to {self.config_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save config to {self.config_path}: {e}")
            return False
    
    def get_config(self) -> OpenDogV3Config:
        """Get current configuration (load if not already loaded)"""
        if self.config is None:
            self.load_config()
        return self.config
    
    def update_joint_offsets(self, offsets: Dict[int, float]) -> bool:
        """
        Update joint offset values
        
        Args:
            offsets: Dictionary mapping joint IDs to offset values
            
        Returns:
            True if updated successfully
        """
        if self.config is None:
            self.load_config()
        
        try:
            self.config.joint_offsets.from_dict(offsets)
            logger.info("Joint offsets updated")
            return True
        except Exception as e:
            logger.error(f"Failed to update joint offsets: {e}")
            return False
    
    def validate_config(self, config: Optional[OpenDogV3Config] = None) -> bool:
        """
        Validate configuration values
        
        Args:
            config: Configuration to validate (uses current if None)
            
        Returns:
            True if configuration is valid
        """
        if config is None:
            config = self.get_config()
        
        try:
            # Validate physical dimensions
            if config.physical.shin_length <= 0 or config.physical.thigh_length <= 0:
                logger.error("Invalid leg segment lengths")
                return False
            
            # Validate motor parameters
            if config.motor.current_limit <= 0 or config.motor.velocity_limit <= 0:
                logger.error("Invalid motor limits")
                return False
            
            # Validate communication settings
            if len(config.communication.odrive_ports) != 6:
                logger.error("Must have exactly 6 ODrive ports configured")
                return False
            
            # Validate control system parameters
            if config.control_system.main_loop_frequency <= 0:
                logger.error("Invalid main loop frequency")
                return False
            
            logger.info("Configuration validation passed")
            return True
            
        except Exception as e:
            logger.error(f"Configuration validation failed: {e}")
            return False
    
    def _create_default_config(self) -> OpenDogV3Config:
        """Create default configuration"""
        return OpenDogV3Config(
            physical=RobotPhysicalConfig(),
            motor=MotorConfig(),
            joint_offsets=JointOffsetsConfig(),
            communication=CommunicationConfig(),
            control_system=ControlSystemConfig()
        )
    
    def _config_to_dict(self, config: OpenDogV3Config) -> Dict[str, Any]:
        """Convert configuration to dictionary for JSON serialization"""
        return {
            'config_version': config.config_version,
            'robot_name': config.robot_name,
            'description': config.description,
            'physical': asdict(config.physical),
            'motor': asdict(config.motor),
            'joint_offsets': asdict(config.joint_offsets),
            'communication': asdict(config.communication),
            'control_system': asdict(config.control_system)
        }
    
    def _dict_to_config(self, config_dict: Dict[str, Any]) -> OpenDogV3Config:
        """Convert dictionary to configuration object"""
        return OpenDogV3Config(
            physical=RobotPhysicalConfig(**config_dict.get('physical', {})),
            motor=MotorConfig(**config_dict.get('motor', {})),
            joint_offsets=JointOffsetsConfig(**config_dict.get('joint_offsets', {})),
            communication=CommunicationConfig(**config_dict.get('communication', {})),
            control_system=ControlSystemConfig(**config_dict.get('control_system', {})),
            config_version=config_dict.get('config_version', '1.0'),
            robot_name=config_dict.get('robot_name', 'OpenDog V3'),
            description=config_dict.get('description', 'OpenDog V3 Quadruped Robot Configuration')
        )
    
    def export_joint_offsets(self) -> Dict[int, float]:
        """Export joint offsets as dictionary for use with other modules"""
        config = self.get_config()
        return config.joint_offsets.to_dict()
    
    def get_odrive_ports(self) -> List[str]:
        """Get ODrive serial port list"""
        config = self.get_config()
        return config.communication.odrive_ports.copy()
    
    def get_motor_gains(self) -> Dict[str, float]:
        """Get motor PID gains"""
        config = self.get_config()
        return {
            'pos_gain_hip': config.motor.pos_gain_hip,
            'pos_gain_knee': config.motor.pos_gain_knee,
            'pos_gain_shoulder': config.motor.pos_gain_shoulder,
            'vel_gain': config.motor.vel_gain,
            'vel_integrator_gain': config.motor.vel_integrator_gain
        }
    
    def get_safety_limits(self) -> Dict[str, float]:
        """Get safety limits"""
        config = self.get_config()
        return {
            'position_limit': config.motor.position_limit,
            'velocity_limit': config.motor.velocity_limit,
            'current_limit': config.motor.current_limit,
            'max_roll': config.physical.max_roll,
            'max_pitch': config.physical.max_pitch,
            'max_yaw': config.physical.max_yaw
        }

# Default configuration instance
DEFAULT_CONFIG = OpenDogV3Config(
    physical=RobotPhysicalConfig(),
    motor=MotorConfig(),
    joint_offsets=JointOffsetsConfig(),
    communication=CommunicationConfig(),
    control_system=ControlSystemConfig()
)

# Example usage and testing
if __name__ == "__main__":
    # Initialize configuration manager
    config_manager = ConfigManager("test_config.json")
    
    print("OpenDog V3 Configuration Manager Test")
    print("=" * 50)
    
    # Load or create configuration
    config = config_manager.load_config()
    print(f"Robot: {config.robot_name}")
    print(f"Version: {config.config_version}")
    print(f"Description: {config.description}")
    
    # Validate configuration
    is_valid = config_manager.validate_config()
    print(f"Configuration valid: {is_valid}")
    
    # Display key parameters
    print(f"\nPhysical Parameters:")
    print(f"  Body dimensions: {config.physical.body_width}x{config.physical.body_length}mm")
    print(f"  Leg segments: {config.physical.thigh_length}, {config.physical.shin_length}mm")
    print(f"  Hip offset: {config.physical.hip_offset}mm")
    
    print(f"\nMotor Parameters:")
    print(f"  Current limit: {config.motor.current_limit}A")
    print(f"  Position limit: {config.motor.position_limit} rad")
    print(f"  PID gains: Hip={config.motor.pos_gain_hip}, Knee={config.motor.pos_gain_knee}")
    
    print(f"\nControl System:")
    print(f"  Main loop: {config.control_system.main_loop_frequency}Hz")
    print(f"  Input deadzone: {config.control_system.input_deadzone}")
    print(f"  Trot cycle: {config.control_system.trot_cycle_time}s")
    
    print(f"\nCommunication:")
    print(f"  ODrive ports: {len(config.communication.odrive_ports)} configured")
    print(f"  Baudrate: {config.communication.odrive_baudrate}")
    
    # Test joint offsets
    print(f"\nJoint Offsets:")
    offsets = config_manager.export_joint_offsets()
    for joint_id, offset in sorted(offsets.items()):
        if offset != 0.0:
            print(f"  Joint {joint_id}: {offset:.4f} rad")
    
    # Save configuration
    if config_manager.save_config():
        print(f"\nConfiguration saved to {config_manager.config_path}")
    
    # Test configuration update
    test_offsets = {10: 0.1, 20: -0.05, 30: 0.15}
    if config_manager.update_joint_offsets(test_offsets):
        print(f"Joint offsets updated: {test_offsets}")
    
    print(f"\nConfiguration manager test completed")
