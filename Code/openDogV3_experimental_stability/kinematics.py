#!/usr/bin/env python3
"""
Inverse Kinematics Module for OpenDog V3
Converted from Arduino C++ kinematics.ino

This module provides 6-DOF inverse kinematics calculations for a quadruped robot:
- X, Y, Z translation
- Roll, Pitch, Yaw rotation
- Interpolation support for smooth motion
- Individual leg calculations with body orientation compensation

Physical Parameters:
- Hip offset: 108mm (distance from hip pivot to leg center)
- Body width: 59mm (half distance between hip pivots)
- Body length: 272mm (half distance between shoulder pivots)
- Shin length: 200mm
- Thigh length: 200mm

Leg numbering:
- Leg 1: Front Left
- Leg 2: Front Right  
- Leg 3: Back Left
- Leg 4: Back Right

Author: Converted from Arduino C++ to Python
Date: 2024
"""

import math
import logging
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LegID(Enum):
    """Leg identification constants"""
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4

@dataclass
class RobotDimensions:
    """Robot physical dimensions in millimeters"""
    hip_offset: float = 108.0      # Distance from hip pivot to leg center
    body_width: float = 59.0       # Half distance between hip pivots
    body_length: float = 272.0     # Half distance between shoulder pivots
    shin_length: float = 200.0     # Lower leg segment length
    thigh_length: float = 200.0    # Upper leg segment length

@dataclass
class LegPosition:
    """Target position for a leg"""
    x: float = 0.0  # Forward/backward position (mm)
    y: float = 0.0  # Left/right position (mm)
    z: float = 0.0  # Up/down position (mm)

@dataclass
class BodyOrientation:
    """Body orientation in degrees"""
    roll: float = 0.0   # Roll rotation (around X axis)
    pitch: float = 0.0  # Pitch rotation (around Y axis)
    yaw: float = 0.0    # Yaw rotation (around Z axis)

@dataclass
class JointAngles:
    """Joint angles for one leg in radians"""
    hip: float = 0.0        # Hip joint angle
    shoulder: float = 0.0   # Shoulder joint angle
    knee: float = 0.0       # Knee joint angle

@dataclass
class JointIDs:
    """Joint ID mappings for ODrive control"""
    hip: int
    knee: int 
    shoulder: int

class SimpleInterpolator:
    """
    Simple linear interpolator for smooth motion transitions
    Equivalent to Arduino interpolation library
    """
    
    def __init__(self):
        self.current_value = 0.0
        self.target_value = 0.0
        self.step_size = 0.0
        self.duration = 0
        self.steps_remaining = 0
        
    def go(self, target: float, duration_ms: int) -> float:
        """
        Interpolate to target value over specified duration
        
        Args:
            target: Target value to interpolate to
            duration_ms: Duration in milliseconds
            
        Returns:
            Current interpolated value
        """
        if self.target_value != target or duration_ms != self.duration:
            # New target or duration, restart interpolation
            self.target_value = target
            self.duration = duration_ms
            self.steps_remaining = max(1, duration_ms // 10)  # Assume 100Hz update rate
            
            if self.steps_remaining > 0:
                self.step_size = (target - self.current_value) / self.steps_remaining
            else:
                self.step_size = 0
                
        if self.steps_remaining > 0:
            self.current_value += self.step_size
            self.steps_remaining -= 1
        else:
            self.current_value = self.target_value
            
        return self.current_value
    
    def is_finished(self) -> bool:
        """Check if interpolation is complete"""
        return self.steps_remaining <= 0

class InverseKinematics:
    """
    Inverse kinematics calculator for OpenDog V3 quadruped robot
    Converted from Arduino kinematics() function
    """
    
    def __init__(self, dimensions: Optional[RobotDimensions] = None):
        """
        Initialize inverse kinematics calculator
        
        Args:
            dimensions: Robot physical dimensions
        """
        self.dims = dimensions or RobotDimensions()
        
        # Conversion factor from degrees to motor encoder counts
        self.degrees_to_motor_turns = 0.02777777777777777777777777777778
        
        # Interpolation setup
        self.interpolation_enabled = False
        self.interpolation_settled = False
        self.interpolation_start_time = 0
        
        # Initialize interpolators for each leg and axis
        self.interpolators = {
            LegID.FRONT_LEFT: {
                'x': SimpleInterpolator(),
                'y': SimpleInterpolator(), 
                'z': SimpleInterpolator(),
                'yaw': SimpleInterpolator()
            },
            LegID.FRONT_RIGHT: {
                'x': SimpleInterpolator(),
                'y': SimpleInterpolator(),
                'z': SimpleInterpolator(), 
                'yaw': SimpleInterpolator()
            },
            LegID.BACK_LEFT: {
                'x': SimpleInterpolator(),
                'y': SimpleInterpolator(),
                'z': SimpleInterpolator(),
                'yaw': SimpleInterpolator()
            },
            LegID.BACK_RIGHT: {
                'x': SimpleInterpolator(),
                'y': SimpleInterpolator(),
                'z': SimpleInterpolator(),
                'yaw': SimpleInterpolator()
            }
        }
        
        # Joint ID mappings for each leg
        self.joint_ids = {
            LegID.FRONT_LEFT: JointIDs(hip=40, knee=50, shoulder=51),    # Front left
            LegID.FRONT_RIGHT: JointIDs(hip=10, knee=20, shoulder=21),   # Front right
            LegID.BACK_LEFT: JointIDs(hip=41, knee=60, shoulder=61),     # Back left
            LegID.BACK_RIGHT: JointIDs(hip=11, knee=30, shoulder=31)     # Back right
        }
        
        logger.info("Inverse kinematics initialized")
    
    def calculate_leg_kinematics(self, 
                                leg: LegID,
                                target_pos: LegPosition, 
                                body_orientation: BodyOrientation,
                                use_interpolation: bool = False,
                                interpolation_duration: int = 100) -> Dict[int, float]:
        """
        Calculate inverse kinematics for one leg
        Equivalent to Arduino kinematics() function
        
        Args:
            leg: Which leg to calculate
            target_pos: Target foot position
            body_orientation: Body roll, pitch, yaw
            use_interpolation: Enable smooth interpolation
            interpolation_duration: Interpolation duration in ms
            
        Returns:
            Dictionary mapping joint IDs to target positions in radians
        """
        
        # Get interpolated or direct values
        if use_interpolation and self.interpolation_settled:
            x = self.interpolators[leg]['x'].go(target_pos.x, interpolation_duration)
            y = self.interpolators[leg]['y'].go(target_pos.y, interpolation_duration)
            z = self.interpolators[leg]['z'].go(target_pos.z, interpolation_duration)
            yaw = self.interpolators[leg]['yaw'].go(body_orientation.yaw, interpolation_duration)
        else:
            x = target_pos.x
            y = target_pos.y
            z = target_pos.z
            yaw = body_orientation.yaw
        
        # Convert input values to working coordinates
        roll = body_orientation.roll
        pitch = body_orientation.pitch
        
        # *** YAW AXIS TRANSFORMATION ***
        yaw_rad = math.radians(yaw)
        
        # Apply leg-specific offsets for yaw calculation
        if leg == LegID.FRONT_LEFT:
            y_offset = y - (self.dims.body_width + self.dims.hip_offset)
            x_offset = x - self.dims.body_length
        elif leg == LegID.FRONT_RIGHT:
            y_offset = y + (self.dims.body_width + self.dims.hip_offset)
            x_offset = x - self.dims.body_length
        elif leg == LegID.BACK_LEFT:
            y_offset = y - (self.dims.body_width + self.dims.hip_offset)
            x_offset = x + self.dims.body_length
        elif leg == LegID.BACK_RIGHT:
            y_offset = y + (self.dims.body_width + self.dims.hip_offset)
            x_offset = x + self.dims.body_length
        
        # Calculate existing angle and radius from center
        if abs(x_offset) < 1e-6:  # Avoid division by zero
            x_offset = 1e-6
        existing_angle = math.atan(y_offset / x_offset)
        
        if abs(math.sin(existing_angle)) < 1e-6:  # Avoid division by zero
            radius = abs(x_offset / math.cos(existing_angle))
        else:
            radius = abs(y_offset / math.sin(existing_angle))
        
        # Calculate new position with yaw rotation
        demand_yaw = existing_angle + yaw_rad
        xx3 = radius * math.cos(demand_yaw)
        yy3 = radius * math.sin(demand_yaw)
        
        # Remove offsets to pivot around 0,0
        if leg == LegID.FRONT_LEFT:
            yy3 = yy3 + (self.dims.body_width + self.dims.hip_offset)
            xx3 = xx3 + self.dims.body_length
        elif leg == LegID.FRONT_RIGHT:
            yy3 = yy3 - (self.dims.body_width + self.dims.hip_offset)
            xx3 = xx3 + self.dims.body_length
        elif leg == LegID.BACK_LEFT:
            yy3 = yy3 + (self.dims.body_width + self.dims.hip_offset)
            xx3 = xx3 - self.dims.body_length
        elif leg == LegID.BACK_RIGHT:
            yy3 = yy3 - (self.dims.body_width + self.dims.hip_offset)
            xx3 = xx3 - self.dims.body_length
        
        # *** PITCH AXIS TRANSFORMATION ***
        
        # Invert pitch for front legs
        if leg in [LegID.FRONT_LEFT, LegID.FRONT_RIGHT]:
            pitch = -pitch
            xx3 = -xx3
        
        pitch_rad = math.radians(pitch)
        
        # Calculate pitch transformation
        leg_diff_pitch = math.sin(pitch_rad) * self.dims.body_length
        body_diff_pitch = math.cos(pitch_rad) * self.dims.body_length
        
        # Calculate actual height for this leg
        leg_diff_pitch = z - leg_diff_pitch
        
        # Calculate foot displacement
        foot_displacement_pitch = ((body_diff_pitch - self.dims.body_length) * -1) + xx3
        
        # Avoid division by zero
        if abs(leg_diff_pitch) < 1e-6:
            foot_displacement_angle_pitch = 0
        else:
            foot_displacement_angle_pitch = math.atan(foot_displacement_pitch / leg_diff_pitch)
        
        # Calculate hypotenuse and new coordinates
        if abs(math.cos(foot_displacement_angle_pitch)) < 1e-6:
            zz2a = abs(leg_diff_pitch)
        else:
            zz2a = abs(leg_diff_pitch / math.cos(foot_displacement_angle_pitch))
        
        foot_whole_angle_pitch = foot_displacement_angle_pitch + pitch_rad
        zz2 = math.cos(foot_whole_angle_pitch) * zz2a
        xx1 = math.sin(foot_whole_angle_pitch) * zz2a
        
        # Restore coordinate system for front legs
        if leg in [LegID.FRONT_LEFT, LegID.FRONT_RIGHT]:
            xx1 = -xx1
        
        # *** ROLL AXIS TRANSFORMATION ***
        
        # Apply roll direction for each side
        if leg in [LegID.FRONT_RIGHT, LegID.BACK_LEFT]:
            roll = -roll
            yy3 = -yy3
        
        roll_rad = math.radians(roll)
        
        # Calculate roll transformation
        leg_diff_roll = math.sin(roll_rad) * self.dims.body_width
        body_diff_roll = math.cos(roll_rad) * self.dims.body_width
        
        # Calculate actual height for this leg
        leg_diff_roll = zz2 - leg_diff_roll
        
        # Calculate foot displacement
        foot_displacement_roll = (((body_diff_roll - self.dims.body_width) * -1) + self.dims.hip_offset) - yy3
        
        # Avoid division by zero
        if abs(leg_diff_roll) < 1e-6:
            foot_displacement_angle_roll = 0
        else:
            foot_displacement_angle_roll = math.atan(foot_displacement_roll / leg_diff_roll)
        
        # Calculate hypotenuse and new coordinates
        if abs(math.cos(foot_displacement_angle_roll)) < 1e-6:
            zz1a = abs(leg_diff_roll)
        else:
            zz1a = abs(leg_diff_roll / math.cos(foot_displacement_angle_roll))
        
        foot_whole_angle_roll = foot_displacement_angle_roll + roll_rad
        zz1 = math.cos(foot_whole_angle_roll) * zz1a
        yy1 = (math.sin(foot_whole_angle_roll) * zz1a) - self.dims.hip_offset
        
        # *** HIP JOINT CALCULATION (Y AXIS - SIDE TO SIDE) ***
        
        # Reverse calculations for left side legs
        hip_offset = self.dims.hip_offset
        if leg in [LegID.FRONT_LEFT, LegID.BACK_LEFT]:
            hip_offset = -hip_offset
            yy1 = -yy1
        
        yy1 = yy1 + hip_offset  # Add hip offset for default Y distance
        
        # Avoid division by zero
        if abs(zz1) < 1e-6:
            hip_angle_1a = 0
        else:
            hip_angle_1a = math.atan(yy1 / zz1)
        
        hip_hyp = abs(zz1 / math.cos(hip_angle_1a)) if abs(math.cos(hip_angle_1a)) > 1e-6 else abs(zz1)
        
        # Calculate hip angle components
        if hip_hyp < abs(hip_offset):
            # Prevent invalid asin input
            hip_angle_1b = 0
        else:
            hip_angle_1b = math.asin(abs(hip_offset) / hip_hyp)
        
        hip_angle_1 = (math.pi - (math.pi/2) - hip_angle_1b) + hip_angle_1a
        hip_angle_1 = hip_angle_1 - 1.5708  # Remove rest position offset
        
        # Calculate new leg length
        if abs(math.tan(hip_angle_1b)) < 1e-6:
            z2 = abs(hip_offset) * 1000  # Very large value if tan is zero
        else:
            z2 = abs(hip_offset / math.tan(hip_angle_1b))
        
        # *** SHOULDER JOINT CALCULATION (X AXIS - FRONT TO BACK) ***
        
        # Avoid division by zero
        if abs(z2) < 1e-6:
            shoulder_angle_2 = 0
        else:
            shoulder_angle_2 = math.atan(xx1 / z2)
        
        # Calculate new leg length
        if abs(math.cos(shoulder_angle_2)) < 1e-6:
            z3 = abs(z2)
        else:
            z3 = abs(z2 / math.cos(shoulder_angle_2))
        
        # *** KNEE AND SHOULDER ANGLE CALCULATION (Z AXIS - UP AND DOWN) ***
        
        # Constrain leg length to prevent impossible configurations
        z3 = max(200, min(390, z3))
        
        # Calculate shoulder and knee angles using law of cosines
        thigh_sq = self.dims.thigh_length ** 2
        shin_sq = self.dims.shin_length ** 2
        z3_sq = z3 ** 2
        
        # Calculate shoulder angle
        cos_shoulder = (thigh_sq + z3_sq - shin_sq) / (2 * self.dims.thigh_length * z3)
        cos_shoulder = max(-1, min(1, cos_shoulder))  # Clamp to valid range
        shoulder_angle_1 = math.acos(cos_shoulder)
        
        # Calculate knee angle
        knee_angle = math.pi - (shoulder_angle_1 * 2)
        
        # *** CONVERT TO JOINT COMMANDS ***
        
        # Convert angles to degrees for motor conversion
        shoulder_angle_1_deg = math.degrees(shoulder_angle_1)
        shoulder_angle_2_deg = math.degrees(shoulder_angle_2)
        knee_angle_deg = math.degrees(knee_angle)
        hip_angle_1_deg = math.degrees(hip_angle_1)
        
        # Calculate final joint positions in motor encoder units
        shoulder_angle_1_counts = (shoulder_angle_1_deg - 45) * self.degrees_to_motor_turns
        shoulder_angle_2_counts = shoulder_angle_2_deg * self.degrees_to_motor_turns
        knee_angle_counts = (knee_angle_deg - 90) * self.degrees_to_motor_turns
        hip_angle_counts = hip_angle_1_deg * self.degrees_to_motor_turns
        
        # Combine shoulder angles based on leg position
        if leg in [LegID.FRONT_LEFT, LegID.FRONT_RIGHT]:
            shoulder_angle_counts = shoulder_angle_1_counts + shoulder_angle_2_counts
        else:  # Back legs
            shoulder_angle_counts = shoulder_angle_1_counts - shoulder_angle_2_counts
        
        # Get joint IDs for this leg
        joint_mapping = self.joint_ids[leg]
        
        # Return joint positions
        return {
            joint_mapping.hip: hip_angle_counts,
            joint_mapping.knee: knee_angle_counts,
            joint_mapping.shoulder: shoulder_angle_counts
        }
    
    def calculate_all_legs(self,
                          leg_positions: Dict[LegID, LegPosition],
                          body_orientation: BodyOrientation,
                          use_interpolation: bool = False,
                          interpolation_duration: int = 100) -> Dict[int, float]:
        """
        Calculate inverse kinematics for all legs
        
        Args:
            leg_positions: Target positions for each leg
            body_orientation: Body roll, pitch, yaw
            use_interpolation: Enable smooth interpolation
            interpolation_duration: Interpolation duration in ms
            
        Returns:
            Dictionary mapping all joint IDs to target positions
        """
        all_joint_positions = {}
        
        for leg, position in leg_positions.items():
            joint_positions = self.calculate_leg_kinematics(
                leg, position, body_orientation, 
                use_interpolation, interpolation_duration
            )
            all_joint_positions.update(joint_positions)
        
        return all_joint_positions
    
    def set_interpolation_enable(self, enable: bool):
        """Enable or disable interpolation"""
        self.interpolation_enabled = enable
        logger.info(f"Interpolation {'enabled' if enable else 'disabled'}")
    
    def reset_interpolation(self):
        """Reset all interpolators to current values"""
        for leg_interpolators in self.interpolators.values():
            for interpolator in leg_interpolators.values():
                interpolator.steps_remaining = 0
        logger.info("Interpolation reset")
    
    def is_interpolation_finished(self) -> bool:
        """Check if all interpolators have finished"""
        for leg_interpolators in self.interpolators.values():
            for interpolator in leg_interpolators.values():
                if not interpolator.is_finished():
                    return False
        return True
    
    def get_leg_reach_limits(self, leg: LegID) -> Dict[str, float]:
        """
        Get reach limits for a specific leg
        
        Args:
            leg: Which leg to get limits for
            
        Returns:
            Dictionary with min/max reach values
        """
        max_reach = self.dims.shin_length + self.dims.thigh_length
        min_reach = abs(self.dims.shin_length - self.dims.thigh_length)
        
        return {
            'max_z': max_reach - 50,  # Leave some margin
            'min_z': min_reach + 50,
            'max_xy_radius': max_reach * 0.8,  # Conservative limit
            'safe_z': 300,  # Safe default height
        }
    
    def validate_leg_position(self, leg: LegID, position: LegPosition) -> bool:
        """
        Validate if a leg position is reachable
        
        Args:
            leg: Which leg to validate
            position: Target position to check
            
        Returns:
            True if position is reachable
        """
        limits = self.get_leg_reach_limits(leg)
        
        # Check Z limits
        if position.z < limits['min_z'] or position.z > limits['max_z']:
            return False
        
        # Check XY radius
        xy_radius = math.sqrt(position.x**2 + position.y**2)
        if xy_radius > limits['max_xy_radius']:
            return False
        
        return True
    
    def get_default_stance(self) -> Dict[LegID, LegPosition]:
        """
        Get default standing position for all legs
        
        Returns:
            Dictionary with default leg positions
        """
        return {
            LegID.FRONT_LEFT: LegPosition(x=0, y=0, z=300),
            LegID.FRONT_RIGHT: LegPosition(x=0, y=0, z=300),
            LegID.BACK_LEFT: LegPosition(x=0, y=0, z=300),
            LegID.BACK_RIGHT: LegPosition(x=0, y=0, z=300)
        }

# Example usage and testing
if __name__ == "__main__":
    # Initialize kinematics
    ik = InverseKinematics()
    
    # Test basic leg position calculation
    test_position = LegPosition(x=50, y=25, z=300)
    test_orientation = BodyOrientation(roll=5, pitch=10, yaw=15)
    
    print("Inverse Kinematics Test")
    print("=" * 50)
    
    # Test each leg
    for leg in LegID:
        print(f"\nLeg: {leg.name}")
        
        # Validate position
        is_valid = ik.validate_leg_position(leg, test_position)
        print(f"Position valid: {is_valid}")
        
        if is_valid:
            # Calculate joint angles
            joint_positions = ik.calculate_leg_kinematics(
                leg, test_position, test_orientation
            )
            
            print(f"Joint positions:")
            for joint_id, position in joint_positions.items():
                print(f"  Joint {joint_id}: {position:.4f} rad ({math.degrees(position):.2f}°)")
        
        # Show limits
        limits = ik.get_leg_reach_limits(leg)
        print(f"Reach limits: Z={limits['min_z']:.0f}-{limits['max_z']:.0f}mm, "
              f"XY_radius={limits['max_xy_radius']:.0f}mm")
    
    # Test all legs calculation
    print(f"\nAll Legs Calculation")
    print("=" * 30)
    
    leg_positions = ik.get_default_stance()
    all_joints = ik.calculate_all_legs(leg_positions, test_orientation)
    
    print(f"Total joints calculated: {len(all_joints)}")
    for joint_id, position in sorted(all_joints.items()):
        print(f"Joint {joint_id}: {position:.4f} rad")
    
    print(f"\nRobot Dimensions:")
    print(f"Hip offset: {ik.dims.hip_offset}mm")
    print(f"Body width: {ik.dims.body_width}mm") 
    print(f"Body length: {ik.dims.body_length}mm")
    print(f"Shin length: {ik.dims.shin_length}mm")
    print(f"Thigh length: {ik.dims.thigh_length}mm")
