#!/usr/bin/env python3
"""
OpenDogV3 Input Processing and Filtering System
Converted from Arduino C++ thresholdSticks.ino

This module handles remote control input processing, including:
- Deadzone application
- Input filtering and smoothing
- Stick calibration and scaling
- Safety threshold management
"""

import time
import math
from typing import Tuple, Dict, Optional
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)

@dataclass
class StickData:
    """Structure for individual stick data"""
    raw_value: float = 0.0
    processed_value: float = 0.0
    filtered_value: float = 0.0
    deadzone: float = 50.0
    scale_min: float = -462.0
    scale_max: float = 462.0
    
class DeadzoneFilter:
    """Deadzone filter for stick inputs"""
    
    def __init__(self, deadzone: float = 50.0):
        self.deadzone = deadzone
        
    def apply(self, value: float) -> float:
        """Apply deadzone to input value"""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale the remaining range
        if value > 0:
            return (value - self.deadzone) / (462.0 - self.deadzone) * 462.0
        else:
            return (value + self.deadzone) / (462.0 - self.deadzone) * 462.0

class LowPassFilter:
    """Low-pass filter for smooth input processing"""
    
    def __init__(self, alpha: float = 0.8):
        """
        Initialize low-pass filter
        
        Args:
            alpha: Filter coefficient (0-1). Higher = more filtering
        """
        self.alpha = alpha
        self.previous_value: Optional[float] = None
        
    def update(self, new_value: float) -> float:
        """Apply low-pass filter to new value"""
        if self.previous_value is None:
            self.previous_value = new_value
            return new_value
            
        filtered = self.alpha * self.previous_value + (1 - self.alpha) * new_value
        self.previous_value = filtered
        return filtered
        
    def reset(self):
        """Reset filter state"""
        self.previous_value = None

class ThresholdSticks:
    """
    Remote control input processing system
    Equivalent to Arduino thresholdSticks.ino functionality
    """
    
    def __init__(self):
        """Initialize stick processing system"""
        
        # Stick data structures
        self.sticks = {
            'RLR': StickData(deadzone=50.0),  # Right stick left/right
            'RFB': StickData(deadzone=50.0),  # Right stick forward/backward
            'RT': StickData(deadzone=20.0),   # Right trigger
            'LLR': StickData(deadzone=30.0),  # Left stick left/right
            'LFB': StickData(deadzone=30.0),  # Left stick forward/backward
            'LT': StickData(deadzone=20.0),   # Left trigger
        }
        
        # Deadzone filters
        self.deadzone_filters = {
            name: DeadzoneFilter(stick.deadzone) 
            for name, stick in self.sticks.items()
        }
        
        # Low-pass filters
        self.lowpass_filters = {
            name: LowPassFilter(alpha=0.7) 
            for name in self.sticks.keys()
        }
        
        # Additional filtering stages
        self.smoothing_filters = {
            name: LowPassFilter(alpha=0.85) 
            for name in self.sticks.keys()
        }
        
        # Filter enable flags
        self.deadzone_enabled = True
        self.lowpass_enabled = True
        self.smoothing_enabled = True
        
        # Calibration values (can be adjusted per controller)
        self.calibration = {
            'RLR': {'min': -462, 'max': 462, 'center': 0},
            'RFB': {'min': -462, 'max': 462, 'center': 0},
            'RT': {'min': 0, 'max': 462, 'center': 0},
            'LLR': {'min': -462, 'max': 462, 'center': 0},
            'LFB': {'min': -462, 'max': 462, 'center': 0},
            'LT': {'min': 0, 'max': 462, 'center': 0},
        }
        
        # Safety limits
        self.safety_limits = {
            'RLR': {'min': -400, 'max': 400},
            'RFB': {'min': -400, 'max': 400},
            'RT': {'min': 0, 'max': 400},
            'LLR': {'min': -300, 'max': 300},
            'LFB': {'min': -300, 'max': 300},
            'LT': {'min': -300, 'max': 300},
        }
        
        # Processing statistics
        self.stats = {
            'total_samples': 0,
            'filtered_samples': 0,
            'last_update_time': 0.0
        }
        
        logger.info("ThresholdSticks input processor initialized")
        
    def set_deadzone(self, stick_name: str, deadzone: float):
        """Set deadzone for specific stick"""
        if stick_name in self.sticks:
            self.sticks[stick_name].deadzone = deadzone
            self.deadzone_filters[stick_name] = DeadzoneFilter(deadzone)
            logger.debug(f"Set deadzone for {stick_name}: {deadzone}")
            
    def set_filter_alpha(self, stick_name: str, alpha: float):
        """Set filter coefficient for specific stick"""
        if stick_name in self.lowpass_filters:
            self.lowpass_filters[stick_name].alpha = alpha
            logger.debug(f"Set filter alpha for {stick_name}: {alpha}")
            
    def calibrate_stick(self, stick_name: str, min_val: float, max_val: float, center: float = 0.0):
        """Calibrate stick range"""
        if stick_name in self.calibration:
            self.calibration[stick_name] = {
                'min': min_val,
                'max': max_val,
                'center': center
            }
            logger.info(f"Calibrated {stick_name}: min={min_val}, max={max_val}, center={center}")
            
    def apply_calibration(self, stick_name: str, raw_value: float) -> float:
        """Apply calibration to raw stick value"""
        if stick_name not in self.calibration:
            return raw_value
            
        cal = self.calibration[stick_name]
        
        # Remove center offset
        centered = raw_value - cal['center']
        
        # Scale to -462 to 462 range (matching Arduino implementation)
        if centered >= 0:
            scaled = (centered / (cal['max'] - cal['center'])) * 462.0
        else:
            scaled = (centered / (cal['center'] - cal['min'])) * 462.0
            
        return scaled
        
    def apply_safety_limits(self, stick_name: str, value: float) -> float:
        """Apply safety limits to processed value"""
        if stick_name not in self.safety_limits:
            return value
            
        limits = self.safety_limits[stick_name]
        return max(limits['min'], min(limits['max'], value))
        
    def process_single_stick(self, stick_name: str, raw_value: float) -> float:
        """Process a single stick input through complete filter chain"""
        
        if stick_name not in self.sticks:
            logger.warning(f"Unknown stick: {stick_name}")
            return 0.0
            
        stick = self.sticks[stick_name]
        
        # Step 1: Apply calibration
        calibrated = self.apply_calibration(stick_name, raw_value)
        stick.raw_value = calibrated
        
        # Step 2: Apply deadzone
        if self.deadzone_enabled:
            processed = self.deadzone_filters[stick_name].apply(calibrated)
        else:
            processed = calibrated
        stick.processed_value = processed
        
        # Step 3: Apply primary low-pass filter
        if self.lowpass_enabled:
            filtered = self.lowpass_filters[stick_name].update(processed)
        else:
            filtered = processed
            
        # Step 4: Apply secondary smoothing filter
        if self.smoothing_enabled:
            smoothed = self.smoothing_filters[stick_name].update(filtered)
        else:
            smoothed = filtered
            
        # Step 5: Apply safety limits
        final = self.apply_safety_limits(stick_name, smoothed)
        stick.filtered_value = final
        
        return final
        
    def process_all_sticks(self, raw_inputs: Dict[str, float]) -> Dict[str, float]:
        """Process all stick inputs"""
        
        self.stats['total_samples'] += 1
        self.stats['last_update_time'] = time.time()
        
        processed = {}
        
        for stick_name, raw_value in raw_inputs.items():
            processed[stick_name] = self.process_single_stick(stick_name, raw_value)
            
        return processed
        
    def get_stick_status(self, stick_name: str) -> Dict:
        """Get detailed status of specific stick"""
        if stick_name not in self.sticks:
            return {}
            
        stick = self.sticks[stick_name]
        return {
            'raw_value': stick.raw_value,
            'processed_value': stick.processed_value,
            'filtered_value': stick.filtered_value,
            'deadzone': stick.deadzone,
            'filter_alpha': self.lowpass_filters[stick_name].alpha,
            'calibration': self.calibration.get(stick_name, {}),
            'safety_limits': self.safety_limits.get(stick_name, {})
        }
        
    def get_processing_stats(self) -> Dict:
        """Get processing statistics"""
        return {
            'total_samples': self.stats['total_samples'],
            'filtered_samples': self.stats['filtered_samples'],
            'last_update_time': self.stats['last_update_time'],
            'sample_rate': self.calculate_sample_rate()
        }
        
    def calculate_sample_rate(self) -> float:
        """Calculate approximate sample rate"""
        if self.stats['total_samples'] < 2:
            return 0.0
            
        current_time = time.time()
        if self.stats.get('start_time'):
            elapsed = current_time - self.stats['start_time']
            return self.stats['total_samples'] / elapsed if elapsed > 0 else 0.0
        else:
            self.stats['start_time'] = current_time
            return 0.0
            
    def reset_filters(self):
        """Reset all filter states"""
        for filter_obj in self.lowpass_filters.values():
            filter_obj.reset()
        for filter_obj in self.smoothing_filters.values():
            filter_obj.reset()
            
        logger.info("All filters reset")
        
    def enable_processing(self, deadzone: bool = True, lowpass: bool = True, smoothing: bool = True):
        """Enable/disable processing stages"""
        self.deadzone_enabled = deadzone
        self.lowpass_enabled = lowpass
        self.smoothing_enabled = smoothing
        
        logger.info(f"Processing enabled - Deadzone: {deadzone}, LowPass: {lowpass}, Smoothing: {smoothing}")
        
    def set_aggressive_filtering(self):
        """Set aggressive filtering for noisy environments"""
        for name in self.lowpass_filters:
            self.lowpass_filters[name].alpha = 0.9
            self.smoothing_filters[name].alpha = 0.95
            
        logger.info("Aggressive filtering enabled")
        
    def set_responsive_filtering(self):
        """Set responsive filtering for quick response"""
        for name in self.lowpass_filters:
            self.lowpass_filters[name].alpha = 0.5
            self.smoothing_filters[name].alpha = 0.6
            
        logger.info("Responsive filtering enabled")
        
    def threshold_test(self, raw_inputs: Dict[str, float], threshold: float = 50.0) -> Dict[str, bool]:
        """Test if inputs exceed threshold (for debugging)"""
        results = {}
        for name, value in raw_inputs.items():
            results[name] = abs(value) > threshold
        return results


# Utility functions (equivalent to Arduino helper functions)
def map_range(value: float, from_min: float, from_max: float, to_min: float, to_max: float) -> float:
    """Map value from one range to another (Arduino map() equivalent)"""
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

def constrain_value(value: float, min_val: float, max_val: float) -> float:
    """Constrain value within range (Arduino constrain() equivalent)"""
    return max(min_val, min(max_val, value))

def apply_expo_curve(value: float, expo: float = 0.3) -> float:
    """Apply exponential curve to stick input for better feel"""
    if value == 0:
        return 0
        
    sign = 1 if value > 0 else -1
    normalized = abs(value) / 462.0  # Normalize to 0-1
    
    # Apply exponential curve
    curved = expo * normalized**3 + (1 - expo) * normalized
    
    return sign * curved * 462.0

def apply_dual_rate(value: float, rate_low: float = 0.5, rate_high: float = 1.0, 
                   switch_point: float = 0.7) -> float:
    """Apply dual rate to stick input (lower sensitivity for small movements)"""
    if value == 0:
        return 0
        
    sign = 1 if value > 0 else -1
    normalized = abs(value) / 462.0
    
    if normalized <= switch_point:
        # Low rate for small movements
        scaled = normalized * rate_low
    else:
        # High rate for large movements
        excess = normalized - switch_point
        scaled = switch_point * rate_low + excess * rate_high
        
    return sign * scaled * 462.0


# Demo and testing
if __name__ == "__main__":
    """
    Demo usage of the ThresholdSticks input processor
    """
    
    import random
    
    # Create processor
    processor = ThresholdSticks()
    
    # Configure for demo
    processor.set_responsive_filtering()
    
    logger.info("Starting ThresholdSticks demo...")
    
    # Simulate 1000 input samples
    for i in range(1000):
        # Simulate noisy stick inputs
        raw_inputs = {
            'RLR': 200 * math.sin(i * 0.01) + random.uniform(-10, 10),
            'RFB': 150 * math.cos(i * 0.02) + random.uniform(-15, 15),
            'RT': 300 + 100 * math.sin(i * 0.005) + random.uniform(-5, 5),
            'LLR': 50 * math.sin(i * 0.03) + random.uniform(-20, 20),
            'LFB': 30 * math.cos(i * 0.04) + random.uniform(-8, 8),
            'LT': 20 * math.sin(i * 0.01) + random.uniform(-3, 3),
        }
        
        # Process inputs
        processed = processor.process_all_sticks(raw_inputs)
        
        # Print every 100 samples
        if i % 100 == 0:
            print(f"\nSample {i}:")
            for name in ['RLR', 'RFB', 'RT']:
                raw = raw_inputs[name]
                filt = processed[name]
                print(f"  {name}: {raw:6.1f} -> {filt:6.1f}")
                
        time.sleep(0.001)  # 1ms delay
        
    # Print final statistics
    stats = processor.get_processing_stats()
    print(f"\nProcessing complete:")
    print(f"  Total samples: {stats['total_samples']}")
    print(f"  Sample rate: {stats['sample_rate']:.1f} Hz")
    
    # Print stick status
    print(f"\nStick status:")
    for stick_name in ['RLR', 'RFB', 'RT']:
        status = processor.get_stick_status(stick_name)
        print(f"  {stick_name}: final={status['filtered_value']:.1f}, deadzone={status['deadzone']}")
        
    logger.info("Demo completed")
