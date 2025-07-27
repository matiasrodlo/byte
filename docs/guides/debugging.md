# Debugging Guide

Learn how to debug and diagnose issues with Byte robot.

## 🔍 Debugging Overview

Effective debugging involves:
- **Systematic approach** - Methodical problem-solving
- **Logging and monitoring** - Track system behavior
- **Hardware diagnostics** - Verify physical connections
- **Software debugging** - Code-level troubleshooting

## 📊 Logging and Monitoring

### Basic Logging

```python
import logging

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('byte_debug.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger('byte')

# Use in your code
logger.debug("Debug message")
logger.info("Info message")
logger.warning("Warning message")
logger.error("Error message")
```

### Robot-Specific Logging

```python
from byte import RobotDog
import logging

class DebugRobotDog(RobotDog):
    """RobotDog with enhanced logging."""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.logger = logging.getLogger('byte.robot')
        
    def do_action(self, action, **kwargs):
        self.logger.info(f"Executing action: {action} with params: {kwargs}")
        try:
            result = super().do_action(action, **kwargs)
            self.logger.info(f"Action {action} completed successfully")
            return result
        except Exception as e:
            self.logger.error(f"Action {action} failed: {e}")
            raise
    
    def ultrasonic_read(self):
        distance = super().ultrasonic.read()
        self.logger.debug(f"Ultrasonic reading: {distance} cm")
        return distance
```

### Performance Monitoring

```python
import time
import psutil

def monitor_performance():
    """Monitor system performance."""
    logger = logging.getLogger('byte.performance')
    
    while True:
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        
        # Memory usage
        memory = psutil.virtual_memory()
        
        # Temperature (Raspberry Pi)
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read()) / 1000.0
        except:
            temp = None
        
        logger.info(f"CPU: {cpu_percent}%, Memory: {memory.percent}%, Temp: {temp}°C")
        
        time.sleep(5)
```

## 🔧 Hardware Diagnostics

### Connection Testing

```python
def test_hardware_connections():
    """Test all hardware connections."""
    logger = logging.getLogger('byte.hardware')
    
    # Test I2C bus
    try:
        import smbus2
        bus = smbus2.SMBus(1)
        logger.info("✅ I2C bus accessible")
    except Exception as e:
        logger.error(f"❌ I2C bus error: {e}")
    
    # Test GPIO
    try:
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        logger.info("✅ GPIO accessible")
    except Exception as e:
        logger.error(f"❌ GPIO error: {e}")
    
    # Test PWM
    try:
        from robot_hat import PWM
        pwm = PWM()
        logger.info("✅ PWM accessible")
    except Exception as e:
        logger.error(f"❌ PWM error: {e}")
```

### Sensor Diagnostics

```python
def test_sensors(dog):
    """Test all sensors."""
    logger = logging.getLogger('byte.sensors')
    
    # Test ultrasonic
    try:
        distance = dog.ultrasonic.read()
        logger.info(f"✅ Ultrasonic: {distance} cm")
    except Exception as e:
        logger.error(f"❌ Ultrasonic error: {e}")
    
    # Test touch sensors
    try:
        left, right = dog.dual_touch.read()
        logger.info(f"✅ Touch sensors: Left={left}, Right={right}")
    except Exception as e:
        logger.error(f"❌ Touch sensor error: {e}")
    
    # Test IMU
    try:
        pitch, roll, yaw = dog.imu.read()
        logger.info(f"✅ IMU: Pitch={pitch}, Roll={roll}, Yaw={yaw}")
    except Exception as e:
        logger.error(f"❌ IMU error: {e}")
```

### Servo Diagnostics

```python
def test_servos(dog):
    """Test all servos."""
    logger = logging.getLogger('byte.servos')
    
    # Test each servo
    for i in range(16):
        try:
            # Move to center position
            dog.set_servo_angle(i, 90)
            logger.info(f"✅ Servo {i}: OK")
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"❌ Servo {i} error: {e}")
```

## 🐛 Software Debugging

### Exception Handling

```python
def robust_robot_control():
    """Robust robot control with exception handling."""
    logger = logging.getLogger('byte.control')
    
    try:
        with RobotDog() as dog:
            logger.info("Robot initialized successfully")
            
            # Test basic functionality
            try:
                dog.do_action('sit')
                logger.info("Sit action successful")
            except Exception as e:
                logger.error(f"Sit action failed: {e}")
            
            try:
                dog.do_action('stand')
                logger.info("Stand action successful")
            except Exception as e:
                logger.error(f"Stand action failed: {e}")
                
    except Exception as e:
        logger.error(f"Robot initialization failed: {e}")
        raise
```

### Debug Mode

```python
class DebugRobotDog(RobotDog):
    """RobotDog with debug mode."""
    
    def __init__(self, debug=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.debug = debug
        self.logger = logging.getLogger('byte.debug')
        
    def do_action(self, action, **kwargs):
        if self.debug:
            self.logger.debug(f"DEBUG: Executing {action} with {kwargs}")
            
        start_time = time.time()
        try:
            result = super().do_action(action, **kwargs)
            if self.debug:
                duration = time.time() - start_time
                self.logger.debug(f"DEBUG: {action} completed in {duration:.3f}s")
            return result
        except Exception as e:
            if self.debug:
                duration = time.time() - start_time
                self.logger.error(f"DEBUG: {action} failed after {duration:.3f}s: {e}")
            raise
```

## 🔍 Interactive Debugging

### Debug Console

```python
import cmd

class ByteDebugConsole(cmd.Cmd):
    """Interactive debug console for Byte."""
    
    intro = 'Byte Debug Console. Type help or ? to list commands.\n'
    prompt = '(byte) '
    
    def __init__(self):
        super().__init__()
        self.dog = None
        self.logger = logging.getLogger('byte.console')
    
    def do_connect(self, arg):
        """Connect to robot."""
        try:
            self.dog = RobotDog()
            print("✅ Connected to robot")
        except Exception as e:
            print(f"❌ Connection failed: {e}")
    
    def do_disconnect(self, arg):
        """Disconnect from robot."""
        if self.dog:
            self.dog.close()
            self.dog = None
            print("✅ Disconnected from robot")
    
    def do_action(self, arg):
        """Execute robot action."""
        if not self.dog:
            print("❌ Not connected to robot")
            return
        
        try:
            self.dog.do_action(arg)
            print(f"✅ Executed: {arg}")
        except Exception as e:
            print(f"❌ Action failed: {e}")
    
    def do_sensors(self, arg):
        """Read all sensors."""
        if not self.dog:
            print("❌ Not connected to robot")
            return
        
        try:
            distance = self.dog.ultrasonic.read()
            left, right = self.dog.dual_touch.read()
            pitch, roll, yaw = self.dog.imu.read()
            
            print(f"Distance: {distance} cm")
            print(f"Touch: Left={left}, Right={right}")
            print(f"IMU: Pitch={pitch}, Roll={roll}, Yaw={yaw}")
        except Exception as e:
            print(f"❌ Sensor read failed: {e}")
    
    def do_quit(self, arg):
        """Exit debug console."""
        if self.dog:
            self.dog.close()
        print("Goodbye!")
        return True

if __name__ == "__main__":
    ByteDebugConsole().cmdloop()
```

### Real-time Monitoring

```python
import threading
import time

def real_time_monitor(dog):
    """Real-time system monitoring."""
    logger = logging.getLogger('byte.monitor')
    
    def monitor_sensors():
        while True:
            try:
                distance = dog.ultrasonic.read()
                left, right = dog.dual_touch.read()
                pitch, roll, yaw = dog.imu.read()
                
                logger.info(f"Sensors - Distance: {distance}cm, Touch: {left}/{right}, IMU: {pitch:.1f}/{roll:.1f}/{yaw:.1f}")
                time.sleep(1)
            except Exception as e:
                logger.error(f"Sensor monitoring error: {e}")
    
    def monitor_system():
        while True:
            try:
                cpu = psutil.cpu_percent()
                memory = psutil.virtual_memory().percent
                logger.info(f"System - CPU: {cpu}%, Memory: {memory}%")
                time.sleep(5)
            except Exception as e:
                logger.error(f"System monitoring error: {e}")
    
    # Start monitoring threads
    sensor_thread = threading.Thread(target=monitor_sensors, daemon=True)
    system_thread = threading.Thread(target=monitor_system, daemon=True)
    
    sensor_thread.start()
    system_thread.start()
    
    return sensor_thread, system_thread
```

## 🧪 Testing Framework

### Unit Tests

```python
import unittest
from unittest.mock import Mock, patch

class TestRobotDog(unittest.TestCase):
    """Unit tests for RobotDog."""
    
    def setUp(self):
        """Set up test environment."""
        self.dog = RobotDog()
    
    def tearDown(self):
        """Clean up after tests."""
        self.dog.close()
    
    def test_initialization(self):
        """Test robot initialization."""
        self.assertIsNotNone(self.dog)
    
    def test_sensor_readings(self):
        """Test sensor readings."""
        distance = self.dog.ultrasonic.read()
        self.assertIsInstance(distance, (int, float))
        self.assertGreaterEqual(distance, 0)
    
    def test_basic_actions(self):
        """Test basic robot actions."""
        # Test sit action
        self.dog.do_action('sit')
        # Add assertions based on expected behavior
    
    @patch('robot_hat.PWM')
    def test_servo_control(self, mock_pwm):
        """Test servo control with mocked hardware."""
        # Test servo movement
        self.dog.set_servo_angle(0, 90)
        mock_pwm.assert_called()

if __name__ == '__main__':
    unittest.main()
```

### Integration Tests

```python
def integration_test():
    """Integration test for complete robot functionality."""
    logger = logging.getLogger('byte.integration')
    
    try:
        with RobotDog() as dog:
            logger.info("Starting integration test")
            
            # Test sensor integration
            distance = dog.ultrasonic.read()
            if distance < 20:
                logger.info("Obstacle detected - testing avoidance")
                dog.do_action('turn_left', angle=45)
            
            # Test movement integration
            dog.do_action('walk', steps=3)
            dog.do_action('turn_right', angle=90)
            dog.do_action('walk', steps=3)
            
            # Test behavior integration
            dog.do_action('bark')
            dog.do_action('wag_tail')
            
            logger.info("Integration test completed successfully")
            
    except Exception as e:
        logger.error(f"Integration test failed: {e}")
        raise
```

## 🔧 Debugging Tools

### System Information

```python
def get_system_info():
    """Get comprehensive system information."""
    import platform
    import subprocess
    
    info = {
        'platform': platform.platform(),
        'python_version': platform.python_version(),
        'processor': platform.processor(),
        'memory': psutil.virtual_memory().total // (1024**3),  # GB
    }
    
    # Raspberry Pi specific info
    try:
        with open('/proc/cpuinfo', 'r') as f:
            for line in f:
                if line.startswith('Model'):
                    info['pi_model'] = line.split(':')[1].strip()
                    break
    except:
        pass
    
    return info
```

### Performance Profiling

```python
import cProfile
import pstats

def profile_robot_operation():
    """Profile robot operations for performance analysis."""
    profiler = cProfile.Profile()
    
    def robot_operation():
        with RobotDog() as dog:
            for _ in range(10):
                dog.do_action('sit')
                dog.do_action('stand')
                dog.ultrasonic.read()
    
    profiler.enable()
    robot_operation()
    profiler.disable()
    
    # Save stats
    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    stats.print_stats(20)
    stats.dump_stats('robot_profile.stats')
```

## 🐛 Common Debugging Scenarios

### Robot Not Responding

```python
def debug_unresponsive_robot():
    """Debug unresponsive robot."""
    logger = logging.getLogger('byte.debug')
    
    # Check power
    logger.info("Checking power supply...")
    # Add power checking logic
    
    # Check connections
    logger.info("Checking hardware connections...")
    test_hardware_connections()
    
    # Check software
    logger.info("Checking software...")
    try:
        with RobotDog() as dog:
            logger.info("Software connection successful")
    except Exception as e:
        logger.error(f"Software connection failed: {e}")
```

### Erratic Movement

```python
def debug_erratic_movement():
    """Debug erratic robot movement."""
    logger = logging.getLogger('byte.debug')
    
    with RobotDog() as dog:
        # Test individual servos
        logger.info("Testing individual servos...")
        test_servos(dog)
        
        # Check IMU calibration
        logger.info("Checking IMU calibration...")
        pitch, roll, yaw = dog.imu.read()
        if abs(pitch) > 5 or abs(roll) > 5:
            logger.warning("IMU may need recalibration")
            dog.calibrate_imu()
        
        # Test with reduced speed
        logger.info("Testing with reduced speed...")
        dog.servo_speed = 30
        dog.do_action('walk', steps=2)
```

## 📚 Next Steps

- **[Troubleshooting Guide](troubleshooting.md)** - Common issues and solutions
- **[Performance Guide](performance.md)** - Optimization and tuning
- **[Examples](../examples/debugging.md)** - More debugging examples
- **[API Reference](../api/debugging.md)** - Debugging API documentation 