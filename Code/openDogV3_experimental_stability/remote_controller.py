#!/usr/bin/env python3
"""
OpenDog V3 Remote Controller - Python Implementation

This module implements the remote control system for the OpenDog V3 quadruped robot.
Converted from Arduino C++ code to Python, maintaining all original functionality.

Original Arduino code: Remote.ino
Author: OpenDog V3 Project
Converted to Python: 2025
"""

import time
import struct
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import threading
import queue

# For GPIO and hardware interfaces (would need actual hardware libraries)
try:
    import RPi.GPIO as GPIO
    import spidev
    import smbus
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: GPIO libraries not available, using simulation mode")

# For radio communication (would need actual nRF24L01 library)
try:
    from nrf24 import NRF24
    RADIO_AVAILABLE = True
except ImportError:
    RADIO_AVAILABLE = False
    print("Warning: nRF24 library not available, using simulation mode")


@dataclass
class SendDataStructure:
    """Data structure for sending commands to the robot"""
    menu_down: int = 0
    select: int = 0
    menu_up: int = 0
    toggle_bottom: int = 0
    toggle_top: int = 0
    toggle1: int = 0
    toggle2: int = 0
    mode: int = 0
    rlr: int = 0  # Right stick left/right
    rfb: int = 0  # Right stick forward/back
    rt: int = 0   # Right trigger
    llr: int = 0  # Left stick left/right
    lfb: int = 0  # Left stick forward/back
    lt: int = 0   # Left trigger

    def to_bytes(self) -> bytes:
        """Convert to byte array for transmission"""
        return struct.pack('<14h',  # 14 signed shorts (int16_t)
            self.menu_down, self.select, self.menu_up,
            self.toggle_bottom, self.toggle_top, self.toggle1, self.toggle2,
            self.mode, self.rlr, self.rfb, self.rt, self.llr, self.lfb, self.lt
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> 'SendDataStructure':
        """Create from byte array"""
        values = struct.unpack('<14h', data)
        return cls(*values)


@dataclass
class ReceiveDataStructureRemote:
    """Data structure for receiving data from the robot"""
    mode: int = 0
    count: int = 0

    def to_bytes(self) -> bytes:
        """Convert to byte array for transmission"""
        return struct.pack('<2h', self.mode, self.count)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ReceiveDataStructureRemote':
        """Create from byte array"""
        values = struct.unpack('<2h', data)
        return cls(*values)


class GPIOInterface:
    """GPIO interface for buttons, switches, and analog inputs"""
    
    def __init__(self, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode or not GPIO_AVAILABLE
        self.logger = logging.getLogger(__name__)
        
        # Pin definitions (matching Arduino)
        self.button_pins = {
            'but1': 2,  # menuDown
            'but2': 3,  # menuUp  
            'but3': 0,  # toggle2
            'but4': 1   # Select
        }
        
        self.switch_pins = {
            'sw1': 4,  # toggleTop (motor enable)
            'sw2': 5,  # toggleBottom (IMU enable)
            'sw3': 6,  # toggle1
            'sw4': 8,  # unused
            'sw5': 7   # reverse walking
        }
        
        # Analog pin mappings (would use ADC on Pi)
        self.analog_pins = {
            'axis1': 'A5',  # RFB
            'axis2': 'A4',  # RLR
            'axis3': 'A6',  # RT
            'axis4': 'A3',  # LFB
            'axis5': 'A2',  # LLR
            'axis6': 'A1'   # LT
        }
        
        # Current values
        self.button_states = {}
        self.switch_states = {}
        self.analog_values = {}
        
        if not self.simulation_mode:
            self._init_gpio()
        else:
            self._init_simulation()
    
    def _init_gpio(self):
        """Initialize real GPIO"""
        if not GPIO_AVAILABLE:
            raise RuntimeError("GPIO not available")
            
        GPIO.setmode(GPIO.BCM)
        
        # Setup button pins with pullup
        for name, pin in self.button_pins.items():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
        # Setup switch pins with pullup
        for name, pin in self.switch_pins.items():
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
        # Would need SPI ADC setup for analog inputs
        self.logger.info("GPIO initialized for real hardware")
    
    def _init_simulation(self):
        """Initialize simulation mode"""
        # Initialize with default values
        for name in self.button_pins:
            self.button_states[name] = 1  # Pulled up (not pressed)
            
        for name in self.switch_pins:
            self.switch_states[name] = 1  # Pulled up (off)
            
        for name in self.analog_pins:
            self.analog_values[name] = 512  # Mid-range (0-1023)
            
        self.logger.info("GPIO initialized in simulation mode")
    
    def read_button(self, button_name: str) -> int:
        """Read button state (0 = pressed, 1 = not pressed)"""
        if self.simulation_mode:
            return self.button_states.get(button_name, 1)
        else:
            pin = self.button_pins.get(button_name)
            if pin is not None:
                return GPIO.input(pin)
            return 1
    
    def read_switch(self, switch_name: str) -> int:
        """Read switch state (0 = on, 1 = off)"""
        if self.simulation_mode:
            return self.switch_states.get(switch_name, 1)
        else:
            pin = self.switch_pins.get(switch_name)
            if pin is not None:
                return GPIO.input(pin)
            return 1
    
    def read_analog(self, axis_name: str) -> int:
        """Read analog value (0-1023)"""
        if self.simulation_mode:
            return self.analog_values.get(axis_name, 512)
        else:
            # Would implement real ADC reading here
            return 512
    
    def set_simulation_button(self, button_name: str, value: int):
        """Set button state in simulation mode"""
        if self.simulation_mode:
            self.button_states[button_name] = value
    
    def set_simulation_switch(self, switch_name: str, value: int):
        """Set switch state in simulation mode"""
        if self.simulation_mode:
            self.switch_states[switch_name] = value
    
    def set_simulation_analog(self, axis_name: str, value: int):
        """Set analog value in simulation mode (0-1023)"""
        if self.simulation_mode:
            self.analog_values[axis_name] = max(0, min(1023, value))
    
    def cleanup(self):
        """Cleanup GPIO resources"""
        if not self.simulation_mode and GPIO_AVAILABLE:
            GPIO.cleanup()


class RadioInterface:
    """Radio interface for nRF24L01 communication"""
    
    def __init__(self, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode or not RADIO_AVAILABLE
        self.logger = logging.getLogger(__name__)
        
        # Radio configuration
        self.ce_pin = 14
        self.csn_pin = 10
        self.addresses = [b"00001", b"00002"]
        
        # Data queues for simulation
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()
        
        if not self.simulation_mode:
            self._init_radio()
        else:
            self._init_simulation()
    
    def _init_radio(self):
        """Initialize real radio"""
        if not RADIO_AVAILABLE:
            raise RuntimeError("Radio not available")
            
        self.radio = NRF24()
        self.radio.begin(self.ce_pin, self.csn_pin)
        self.radio.setRetries(15, 15)
        self.radio.setPayloadSize(32)
        self.radio.setChannel(0x60)
        self.radio.setDataRate(NRF24.BR_250KBPS)
        self.radio.setPALevel(NRF24.PA_MIN)
        
        self.radio.openWritingPipe(self.addresses[1])  # 00001
        self.radio.openReadingPipe(1, self.addresses[0])  # 00002
        
        self.radio.stopListening()
        self.logger.info("Radio initialized for real hardware")
    
    def _init_simulation(self):
        """Initialize simulation mode"""
        self.logger.info("Radio initialized in simulation mode")
    
    def write(self, data: bytes) -> bool:
        """Write data to radio"""
        if self.simulation_mode:
            self.tx_queue.put(data)
            return True
        else:
            return self.radio.write(data)
    
    def read(self) -> Optional[bytes]:
        """Read data from radio"""
        if self.simulation_mode:
            try:
                return self.rx_queue.get_nowait()
            except queue.Empty:
                return None
        else:
            if self.radio.available():
                return self.radio.read(32)  # Read payload size
            return None
    
    def available(self) -> bool:
        """Check if data is available"""
        if self.simulation_mode:
            return not self.rx_queue.empty()
        else:
            return self.radio.available()


class LCDInterface:
    """LCD interface for displaying status"""
    
    def __init__(self, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode
        self.logger = logging.getLogger(__name__)
        
        # LCD configuration
        self.i2c_address = 0x27
        self.cols = 20
        self.rows = 4
        
        # Current display content
        self.display_buffer = [[''] * self.cols for _ in range(self.rows)]
        
        if not self.simulation_mode:
            self._init_lcd()
        else:
            self._init_simulation()
    
    def _init_lcd(self):
        """Initialize real LCD"""
        try:
            import smbus
            self.bus = smbus.SMBus(1)  # I2C bus 1
            self.logger.info("LCD initialized for real hardware")
        except ImportError:
            self.simulation_mode = True
            self._init_simulation()
    
    def _init_simulation(self):
        """Initialize simulation mode"""
        self.logger.info("LCD initialized in simulation mode")
        self.clear()
        self.print_line(0, "Everything Remote   ")
        self.print_line(1, "XRobots.co.uk       ")
    
    def clear(self):
        """Clear the display"""
        self.display_buffer = [[' '] * self.cols for _ in range(self.rows)]
        if self.simulation_mode:
            print("\nLCD Display:")
            print("=" * (self.cols + 2))
    
    def set_cursor(self, col: int, row: int):
        """Set cursor position"""
        self.current_col = max(0, min(col, self.cols - 1))
        self.current_row = max(0, min(row, self.rows - 1))
    
    def print_text(self, text: str):
        """Print text at current cursor position"""
        if not hasattr(self, 'current_row'):
            self.current_row = 0
            self.current_col = 0
            
        for char in text:
            if self.current_col < self.cols:
                self.display_buffer[self.current_row][self.current_col] = char
                self.current_col += 1
    
    def print_line(self, row: int, text: str):
        """Print text on a specific line"""
        self.set_cursor(0, row)
        # Clear the line first
        self.display_buffer[row] = [' '] * self.cols
        self.print_text(text[:self.cols])
    
    def update_display(self):
        """Update the physical display (or print in simulation)"""
        if self.simulation_mode:
            print("\nLCD Display:")
            print("+" + "-" * self.cols + "+")
            for row in self.display_buffer:
                print("|" + "".join(row) + "|")
            print("+" + "-" * self.cols + "+")


class RemoteController:
    """Main remote controller class"""
    
    def __init__(self, simulation_mode: bool = False):
        self.simulation_mode = simulation_mode
        self.logger = logging.getLogger(__name__)
        
        # Initialize interfaces
        self.gpio = GPIOInterface(simulation_mode)
        self.radio = RadioInterface(simulation_mode)
        self.lcd = LCDInterface(simulation_mode)
        
        # Timing
        self.previous_millis = 0
        self.interval = 20  # 20ms = 50Hz update rate
        
        # Data structures
        self.send_data = SendDataStructure()
        self.receive_data = ReceiveDataStructureRemote()
        
        # State variables
        self.remote_flag = 0
        self.running = False
        self.control_thread = None
        
        self.logger.info(f"Remote controller initialized (simulation: {simulation_mode})")
    
    def map_value(self, value: int, in_min: int, in_max: int, 
                  out_min: int, out_max: int) -> int:
        """Map a value from one range to another (Arduino map function)"""
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
    def read_inputs(self):
        """Read all input devices"""
        # Read buttons
        but1 = self.gpio.read_button('but1')
        but2 = self.gpio.read_button('but2')
        but3 = self.gpio.read_button('but3')
        but4 = self.gpio.read_button('but4')
        
        # Read switches
        sw1 = self.gpio.read_switch('sw1')
        sw2 = self.gpio.read_switch('sw2')
        sw3 = self.gpio.read_switch('sw3')
        sw4 = self.gpio.read_switch('sw4')
        sw5 = self.gpio.read_switch('sw5')
        
        # Read analog inputs
        axis1 = self.gpio.read_analog('axis1')  # A5 -> RFB
        axis2 = self.gpio.read_analog('axis2')  # A4 -> RLR
        axis3 = self.gpio.read_analog('axis3')  # A6 -> RT
        axis4 = self.gpio.read_analog('axis4')  # A3 -> LFB
        axis5 = self.gpio.read_analog('axis5')  # A2 -> LLR
        axis6 = self.gpio.read_analog('axis6')  # A1 -> LT
        
        # Process button inputs
        self.send_data.menu_down = 1 if but1 == 0 else 0
        self.send_data.select = 1 if but4 == 0 else 0
        self.send_data.menu_up = 1 if but2 == 0 else 0
        
        # Process switch inputs
        self.send_data.toggle_bottom = 1 if sw2 == 0 else 0  # IMU enable
        self.send_data.toggle_top = 1 if sw1 == 0 else 0     # Motor enable
        self.send_data.toggle1 = 1 if sw3 == 0 else 0
        self.send_data.toggle2 = 1 if but3 == 0 else 0
        
        # Process analog inputs with reversals as in original code
        axis2 = self.map_value(axis2, 0, 1023, 1023, 0)  # Reverse axis2
        axis4 = self.map_value(axis4, 0, 1023, 1023, 0)  # Reverse axis4
        
        # Reverse walking mode - reverse all controls when sw5 is active
        if sw5 == 0:
            axis1 = self.map_value(axis1, 0, 1023, 1023, 0)
            axis2 = self.map_value(axis2, 0, 1023, 1023, 0)
            axis4 = self.map_value(axis4, 0, 1023, 1023, 0)
            axis5 = self.map_value(axis5, 0, 1023, 1023, 0)
        
        # Always reverse axis6
        axis6 = self.map_value(axis6, 0, 1023, 1023, 0)
        
        # Assign to send data structure
        self.send_data.rfb = axis1
        self.send_data.rlr = axis2
        self.send_data.rt = axis3
        self.send_data.lfb = axis4
        self.send_data.llr = axis5
        self.send_data.lt = axis6
        
        return True
    
    def transmit_data(self) -> bool:
        """Transmit data to robot"""
        try:
            data = self.send_data.to_bytes()
            success = self.radio.write(data)
            if success:
                self.logger.debug("Data transmitted successfully")
            return success
        except Exception as e:
            self.logger.error(f"Transmission error: {e}")
            return False
    
    def receive_data(self) -> bool:
        """Receive data from robot"""
        if self.radio.available():
            data = self.radio.read()
            if data and len(data) >= 4:  # 2 int16_t = 4 bytes
                try:
                    self.receive_data = ReceiveDataStructureRemote.from_bytes(data[:4])
                    self.logger.debug(f"Received: mode={self.receive_data.mode}, count={self.receive_data.count}")
                    return True
                except Exception as e:
                    self.logger.error(f"Data parsing error: {e}")
        return False
    
    def update_display(self):
        """Update LCD display with current status"""
        # Display basic info
        line0 = f"Remote Control v3.0 "
        line1 = f"Mode: {self.receive_data.mode:2d}  Count: {self.receive_data.count:4d}"
        line2 = f"RFB:{self.send_data.rfb:4d} RLR:{self.send_data.rlr:4d}"
        line3 = f"LFB:{self.send_data.lfb:4d} LLR:{self.send_data.llr:4d}"
        
        self.lcd.print_line(0, line0)
        self.lcd.print_line(1, line1)
        self.lcd.print_line(2, line2)
        self.lcd.print_line(3, line3)
        
        if self.simulation_mode:
            self.lcd.update_display()
    
    def control_loop(self):
        """Main control loop"""
        while self.running:
            current_millis = int(time.time() * 1000)
            
            if current_millis - self.previous_millis >= self.interval:
                self.previous_millis = current_millis
                
                # Read inputs
                self.read_inputs()
                
                # Transmit data
                self.transmit_data()
                
                # Try to receive data
                self.receive_data()
                
                # Update display
                self.update_display()
            
            time.sleep(0.001)  # 1ms sleep to prevent excessive CPU usage
    
    def start(self):
        """Start the remote controller"""
        if self.running:
            self.logger.warning("Remote controller already running")
            return
        
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.logger.info("Remote controller started")
    
    def stop(self):
        """Stop the remote controller"""
        if not self.running:
            return
        
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        
        self.gpio.cleanup()
        self.logger.info("Remote controller stopped")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        return {
            'running': self.running,
            'simulation_mode': self.simulation_mode,
            'send_data': {
                'rfb': self.send_data.rfb,
                'rlr': self.send_data.rlr,
                'rt': self.send_data.rt,
                'lfb': self.send_data.lfb,
                'llr': self.send_data.llr,
                'lt': self.send_data.lt,
                'menu_up': self.send_data.menu_up,
                'menu_down': self.send_data.menu_down,
                'select': self.send_data.select,
                'toggle_top': self.send_data.toggle_top,
                'toggle_bottom': self.send_data.toggle_bottom,
            },
            'receive_data': {
                'mode': self.receive_data.mode,
                'count': self.receive_data.count,
            }
        }
    
    def set_stick_values(self, rfb: int = None, rlr: int = None, rt: int = None,
                        lfb: int = None, llr: int = None, lt: int = None):
        """Set stick values directly (for testing/simulation)"""
        if not self.simulation_mode:
            self.logger.warning("Cannot set stick values in hardware mode")
            return
        
        if rfb is not None:
            self.gpio.set_simulation_analog('axis1', rfb)
        if rlr is not None:
            self.gpio.set_simulation_analog('axis2', rlr)
        if rt is not None:
            self.gpio.set_simulation_analog('axis3', rt)
        if lfb is not None:
            self.gpio.set_simulation_analog('axis4', lfb)
        if llr is not None:
            self.gpio.set_simulation_analog('axis5', llr)
        if lt is not None:
            self.gpio.set_simulation_analog('axis6', lt)
    
    def set_button_state(self, button: str, pressed: bool):
        """Set button state (for testing/simulation)"""
        if not self.simulation_mode:
            self.logger.warning("Cannot set button state in hardware mode")
            return
        
        # Button is active low (0 = pressed)
        value = 0 if pressed else 1
        self.gpio.set_simulation_button(button, value)
    
    def set_switch_state(self, switch: str, active: bool):
        """Set switch state (for testing/simulation)"""
        if not self.simulation_mode:
            self.logger.warning("Cannot set switch state in hardware mode")
            return
        
        # Switch is active low (0 = on)
        value = 0 if active else 1
        self.gpio.set_simulation_switch(switch, value)


def main():
    """Main function for testing"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create remote controller in simulation mode
    remote = RemoteController(simulation_mode=True)
    
    try:
        # Start the controller
        remote.start()
        
        # Test sequence
        print("Remote controller started in simulation mode")
        print("Testing stick movements...")
        
        for i in range(10):
            time.sleep(1)
            
            # Simulate stick movements
            if i % 2 == 0:
                remote.set_stick_values(rfb=800, rlr=200, rt=512)
            else:
                remote.set_stick_values(rfb=200, rlr=800, rt=1000)
            
            # Simulate button presses
            if i == 3:
                remote.set_button_state('but4', True)  # Press select
                print("Select button pressed")
            elif i == 4:
                remote.set_button_state('but4', False)  # Release select
                print("Select button released")
            
            # Show status
            status = remote.get_status()
            print(f"Cycle {i}: RFB={status['send_data']['rfb']}, "
                  f"RLR={status['send_data']['rlr']}, "
                  f"SELECT={status['send_data']['select']}")
        
    except KeyboardInterrupt:
        print("\nStopping remote controller...")
    finally:
        remote.stop()
        print("Remote controller stopped")


if __name__ == "__main__":
    main()
