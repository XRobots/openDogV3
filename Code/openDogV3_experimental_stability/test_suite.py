#!/usr/bin/env python3
"""
OpenDog V3 Test Suite - Python Implementation

This module provides comprehensive testing for all OpenDog V3 Python modules.
Tests kinematics, motor control, IMU, input processing, and full system integration.

Author: OpenDog V3 Project
Date: 2025
"""

import sys
import os
import time
import math
import logging
import unittest
from typing import Dict, List, Tuple, Any
import threading
import json

# Add the current directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import OpenDog modules
from config import RobotConfig
from kinematics import InverseKinematics, LegPosition
from odrive_interface import ODriveInterface
from imu_interface import IMUInterface
from input_processing import InputProcessor
from opendog_v3_controller import OpenDogController
from remote_controller import RemoteController


class TestKinematics(unittest.TestCase):
    """Test the inverse kinematics module"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.kinematics = InverseKinematics(self.config)
    
    def test_basic_kinematics(self):
        """Test basic kinematics calculations"""
        # Test default standing position
        result = self.kinematics.calculate_leg_position(
            leg_id=1,  # Front right
            x=0, y=0, z=300,  # Standing height
            roll=0, pitch=0, yaw=0
        )
        
        self.assertIsInstance(result, LegPosition)
        self.assertTrue(result.valid)
        self.assertGreater(result.hip_angle, -math.pi)
        self.assertLess(result.hip_angle, math.pi)
        
    def test_leg_reach_limits(self):
        """Test leg reach validation"""
        # Test extreme positions that should be invalid
        result = self.kinematics.calculate_leg_position(
            leg_id=1,
            x=1000, y=1000, z=100,  # Unreachable position
            roll=0, pitch=0, yaw=0
        )
        
        self.assertFalse(result.valid)
        
    def test_all_legs(self):
        """Test calculations for all four legs"""
        for leg_id in range(1, 5):
            result = self.kinematics.calculate_leg_position(
                leg_id=leg_id,
                x=0, y=0, z=300,
                roll=0, pitch=0, yaw=0
            )
            self.assertTrue(result.valid, f"Leg {leg_id} failed")
    
    def test_body_rotations(self):
        """Test body roll, pitch, yaw transformations"""
        # Test small rotations
        for angle in [-0.2, 0, 0.2]:  # ±11.5 degrees
            result = self.kinematics.calculate_leg_position(
                leg_id=1,
                x=0, y=0, z=300,
                roll=angle, pitch=angle, yaw=angle
            )
            self.assertTrue(result.valid, f"Failed at angle {angle}")


class TestInputProcessing(unittest.TestCase):
    """Test the input processing module"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.processor = InputProcessor(self.config)
    
    def test_deadzone_filtering(self):
        """Test deadzone filtering"""
        # Test values within deadzone
        result = self.processor.apply_deadzone(50, 100)
        self.assertEqual(result, 0)
        
        # Test values outside deadzone
        result = self.processor.apply_deadzone(150, 100)
        self.assertNotEqual(result, 0)
    
    def test_low_pass_filter(self):
        """Test low-pass filtering"""
        # Process a step input
        filtered_value = 0
        for _ in range(50):
            filtered_value = self.processor.low_pass_filter(100, filtered_value, 10)
        
        # Should approach the target value
        self.assertGreater(filtered_value, 80)
        self.assertLess(filtered_value, 100)
    
    def test_input_scaling(self):
        """Test input scaling and mapping"""
        # Test normal range
        result = self.processor.map_stick_input(512, -100, 100)
        self.assertAlmostEqual(result, 0, delta=5)
        
        # Test extremes
        result = self.processor.map_stick_input(0, -100, 100)
        self.assertAlmostEqual(result, -100, delta=5)
        
        result = self.processor.map_stick_input(1023, -100, 100)
        self.assertAlmostEqual(result, 100, delta=5)


class TestODriveInterface(unittest.TestCase):
    """Test the ODrive interface module"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.odrive = ODriveInterface(self.config, simulation_mode=True)
    
    def test_initialization(self):
        """Test ODrive initialization"""
        self.assertTrue(self.odrive.simulation_mode)
        self.assertEqual(len(self.odrive.controllers), 6)
    
    def test_joint_mapping(self):
        """Test joint to controller mapping"""
        # Test valid joints
        for joint_id in range(12):
            controller_id, axis = self.odrive.get_controller_for_joint(joint_id)
            self.assertIsNotNone(controller_id)
            self.assertIsNotNone(axis)
    
    def test_position_commands(self):
        """Test position command processing"""
        # Test setting all joint positions
        positions = [0.5] * 12  # All joints to 0.5 radians
        success = self.odrive.set_joint_positions(positions)
        self.assertTrue(success)
        
        # Verify positions were set
        current_positions = self.odrive.get_joint_positions()
        self.assertEqual(len(current_positions), 12)


class TestIMUInterface(unittest.TestCase):
    """Test the IMU interface module"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.imu = IMUInterface(self.config, simulation_mode=True)
    
    def test_initialization(self):
        """Test IMU initialization"""
        self.assertTrue(self.imu.simulation_mode)
        self.assertIsNotNone(self.imu.attitude)
    
    def test_complementary_filter(self):
        """Test complementary filter"""
        # Simulate accelerometer and gyroscope data
        accel_data = (0, 0, 1)  # 1g downward
        gyro_data = (0.1, 0, 0)  # Small rotation
        
        # Process data multiple times
        for _ in range(100):
            self.imu._update_attitude(accel_data, gyro_data, 0.01)
        
        # Check that attitude is reasonable
        self.assertLess(abs(self.imu.attitude.roll), math.pi/2)
        self.assertLess(abs(self.imu.attitude.pitch), math.pi/2)
    
    def test_data_acquisition(self):
        """Test continuous data acquisition"""
        self.imu.start()
        time.sleep(0.1)  # Let it run briefly
        self.imu.stop()
        
        # Should have collected some data
        self.assertIsNotNone(self.imu.attitude)


class TestFullSystem(unittest.TestCase):
    """Test full system integration"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.robot = OpenDogController(simulation_mode=True)
        self.remote = RemoteController(simulation_mode=True)
    
    def test_system_initialization(self):
        """Test full system initialization"""
        self.assertTrue(self.robot.simulation_mode)
        self.assertTrue(self.remote.simulation_mode)
        self.assertIsNotNone(self.robot.kinematics)
        self.assertIsNotNone(self.robot.odrive)
        self.assertIsNotNone(self.robot.imu)
        self.assertIsNotNone(self.robot.input_processor)
    
    def test_communication_loop(self):
        """Test robot-remote communication"""
        # Start both systems
        self.robot.start()
        self.remote.start()
        
        try:
            # Let them communicate briefly
            time.sleep(0.5)
            
            # Check that they're running
            self.assertTrue(self.robot.running)
            self.assertTrue(self.remote.running)
            
            # Test remote input
            self.remote.set_stick_values(rfb=700, rlr=300)
            time.sleep(0.1)
            
            # Check robot received input
            status = self.robot.get_status()
            self.assertIsNotNone(status)
            
        finally:
            self.robot.stop()
            self.remote.stop()
    
    def test_walking_gait(self):
        """Test walking gait execution"""
        self.robot.start()
        
        try:
            # Set walking mode
            self.robot.set_run_mode(2)  # Walking mode
            
            # Simulate forward walking command
            remote_data = {
                'rfb': 700,  # Forward
                'rlr': 512,  # Centered
                'rt': 340,   # Standing height
                'toggle_top': 1  # Motors enabled
            }
            
            # Let it walk for a few cycles
            for _ in range(20):
                self.robot.process_remote_input(remote_data)
                time.sleep(0.01)  # 100Hz
            
            # Check that gait is executing
            status = self.robot.get_status()
            self.assertIsNotNone(status)
            
        finally:
            self.robot.stop()
    
    def test_stability_control(self):
        """Test IMU-based stability control"""
        self.robot.start()
        
        try:
            # Enable stability mode
            self.robot.set_run_mode(2)  # Walking with stability
            
            # Simulate IMU disturbance
            self.robot.imu.simulate_attitude(roll=0.1, pitch=0.05)
            
            # Let stability control respond
            for _ in range(10):
                time.sleep(0.01)
            
            # Check that stability compensation was applied
            status = self.robot.get_status()
            self.assertIsNotNone(status)
            
        finally:
            self.robot.stop()


class TestPerformance(unittest.TestCase):
    """Test system performance and timing"""
    
    def setUp(self):
        self.config = RobotConfig()
        self.robot = OpenDogController(simulation_mode=True)
    
    def test_control_loop_timing(self):
        """Test that control loop maintains 100Hz"""
        self.robot.start()
        
        try:
            # Measure loop timing
            start_time = time.time()
            initial_count = self.robot.loop_count
            
            time.sleep(1.0)  # Run for 1 second
            
            end_time = time.time()
            final_count = self.robot.loop_count
            
            # Calculate actual frequency
            elapsed = end_time - start_time
            loops = final_count - initial_count
            frequency = loops / elapsed
            
            # Should be close to 100Hz
            self.assertGreater(frequency, 90)  # At least 90Hz
            self.assertLess(frequency, 110)    # Not more than 110Hz
            
        finally:
            self.robot.stop()
    
    def test_kinematics_performance(self):
        """Test kinematics calculation performance"""
        kinematics = InverseKinematics(self.config)
        
        # Time 1000 kinematics calculations
        start_time = time.time()
        
        for i in range(1000):
            kinematics.calculate_leg_position(
                leg_id=(i % 4) + 1,
                x=i % 100, y=i % 50, z=300,
                roll=0, pitch=0, yaw=0
            )
        
        end_time = time.time()
        elapsed = end_time - start_time
        
        # Should complete in reasonable time
        self.assertLess(elapsed, 1.0)  # Less than 1 second for 1000 calculations
        
        avg_time = elapsed / 1000
        print(f"Average kinematics calculation time: {avg_time*1000:.3f}ms")


def run_comprehensive_test():
    """Run a comprehensive test of the entire system"""
    print("=" * 60)
    print("OpenDog V3 Comprehensive Test Suite")
    print("=" * 60)
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create test suite
    suite = unittest.TestSuite()
    
    # Add all test classes
    test_classes = [
        TestKinematics,
        TestInputProcessing,
        TestODriveInterface,
        TestIMUInterface,
        TestFullSystem,
        TestPerformance
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        suite.addTests(tests)
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "=" * 60)
    print("Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print("\nFailures:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback}")
    
    if result.errors:
        print("\nErrors:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    print(f"\nOverall result: {'PASS' if success else 'FAIL'}")
    print("=" * 60)
    
    return success


def demo_robot_operation():
    """Demonstrate robot operation"""
    print("\n" + "=" * 60)
    print("OpenDog V3 Operation Demo")
    print("=" * 60)
    
    # Create robot and remote
    robot = OpenDogController(simulation_mode=True)
    remote = RemoteController(simulation_mode=True)
    
    try:
        # Start systems
        print("Starting robot and remote controllers...")
        robot.start()
        remote.start()
        
        # Demo sequence
        demo_sequences = [
            ("Standing Mode", {
                'run_mode': 0,
                'rfb': 512, 'rlr': 512, 'rt': 340,
                'toggle_top': 1
            }),
            ("Kinematics Demo", {
                'run_mode': 1,
                'rfb': 600, 'rlr': 400, 'rt': 320,
                'lfb': 5, 'llr': -5, 'lt': 0,
                'toggle_top': 1
            }),
            ("Walking Forward", {
                'run_mode': 2,
                'rfb': 700, 'rlr': 512, 'rt': 340,
                'toggle_top': 1, 'toggle_bottom': 1
            }),
            ("Walking with Turn", {
                'run_mode': 2,
                'rfb': 650, 'rlr': 400, 'rt': 340,
                'lt': 10,
                'toggle_top': 1, 'toggle_bottom': 1
            })
        ]
        
        for demo_name, commands in demo_sequences:
            print(f"\n{demo_name}:")
            print("-" * 30)
            
            # Set robot mode
            if 'run_mode' in commands:
                robot.set_run_mode(commands['run_mode'])
            
            # Set remote inputs
            if 'rfb' in commands:
                remote.set_stick_values(
                    rfb=commands.get('rfb', 512),
                    rlr=commands.get('rlr', 512),
                    rt=commands.get('rt', 340),
                    lfb=commands.get('lfb', 512),
                    llr=commands.get('llr', 512),
                    lt=commands.get('lt', 512)
                )
            
            # Set switches
            if 'toggle_top' in commands:
                remote.set_switch_state('sw1', commands['toggle_top'] == 1)
            if 'toggle_bottom' in commands:
                remote.set_switch_state('sw2', commands['toggle_bottom'] == 1)
            
            # Run for a few seconds
            for i in range(30):  # 3 seconds at 10Hz display
                time.sleep(0.1)
                
                # Show status every 10 iterations
                if i % 10 == 0:
                    robot_status = robot.get_status()
                    remote_status = remote.get_status()
                    
                    print(f"  Robot mode: {robot_status.get('run_mode', 'Unknown')}")
                    print(f"  Input: RFB={remote_status['send_data']['rfb']:4d}, "
                          f"RLR={remote_status['send_data']['rlr']:4d}")
                    print(f"  Motors enabled: {remote_status['send_data']['toggle_top']}")
        
        print(f"\nDemo completed successfully!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nDemo failed with error: {e}")
    finally:
        print("Stopping systems...")
        robot.stop()
        remote.stop()
        print("Demo finished")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='OpenDog V3 Test Suite')
    parser.add_argument('--test', action='store_true', help='Run unit tests')
    parser.add_argument('--demo', action='store_true', help='Run operation demo')
    parser.add_argument('--all', action='store_true', help='Run tests and demo')
    
    args = parser.parse_args()
    
    if args.all or (not args.test and not args.demo):
        # Run everything if no specific option or --all
        success = run_comprehensive_test()
        if success:
            demo_robot_operation()
    elif args.test:
        run_comprehensive_test()
    elif args.demo:
        demo_robot_operation()


if __name__ == "__main__":
    main()
