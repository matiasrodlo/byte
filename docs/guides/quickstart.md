# Quick Start Guide

Get Byte up and running in minutes with this quick start guide.

## 🚀 Prerequisites

- Byte robot assembled and powered on
- Raspberry Pi connected to network
- Python 3.7+ installed

## ⚡ Quick Setup

### 1. Install Byte

```bash
# Clone repository
git clone https://github.com/matiasrodlo/byte.git
cd byte

# Install package
pip install -e .
```

### 2. Run Demo

```bash
# Run built-in demo
byte --demo
```

### 3. Basic Python Usage

```python
from byte import RobotDog

# Initialize robot
dog = RobotDog()

# Basic movements
dog.do_action('sit')
dog.do_action('stand')
dog.do_action('bark')

# Read sensors
distance = dog.ultrasonic.read()
print(f"Distance: {distance} cm")

# Cleanup
dog.close()
```

## 🎯 Essential Commands

### Movement Actions

```python
# Basic poses
dog.do_action('sit')      # Sit down
dog.do_action('stand')    # Stand up
dog.do_action('lie')      # Lie down

# Movements
dog.do_action('walk')     # Walk forward
dog.do_action('trot')     # Trot (faster walk)
dog.do_action('turn_left') # Turn left
dog.do_action('turn_right') # Turn right

# Behaviors
dog.do_action('bark')     # Bark
dog.do_action('wag_tail') # Wag tail
dog.do_action('stretch')  # Stretch
```

### Sensor Reading

```python
# Distance sensor
distance = dog.ultrasonic.read()

# Touch sensors
touch_left, touch_right = dog.dual_touch.read()

# IMU (orientation)
pitch, roll, yaw = dog.imu.read()

# Sound direction
angle = dog.sound_direction.read()
```

### LED Control

```python
# Set RGB color
dog.rgb_strip.set_color(255, 0, 0)  # Red
dog.rgb_strip.set_color(0, 255, 0)  # Green
dog.rgb_strip.set_color(0, 0, 255)  # Blue

# Turn off
dog.rgb_strip.off()
```

## 🎮 Interactive Example

```python
from byte import RobotDog
import time

dog = RobotDog()

try:
    print("Byte is ready! Press Ctrl+C to stop.")
    
    while True:
        # Read distance
        distance = dog.ultrasonic.read()
        print(f"Distance: {distance} cm")
        
        # React to obstacles
        if distance < 20:
            print("Obstacle detected! Turning...")
            dog.do_action('turn_left')
        else:
            print("Path clear! Walking...")
            dog.do_action('walk')
        
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping Byte...")
    dog.do_action('sit')
    dog.close()
    print("Goodbye!")
```

## 🔧 Configuration

### Basic Settings

```python
from byte import RobotDog

# Initialize with custom settings
dog = RobotDog(
    servo_speed=50,      # Servo movement speed (0-100)
    servo_accel=20,      # Servo acceleration (0-100)
    imu_calibration=True # Auto-calibrate IMU
)
```

### Calibration

```python
# Calibrate servos (run once after assembly)
dog.calibrate_servos()

# Calibrate IMU
dog.calibrate_imu()
```

## 📚 Next Steps

- **[Basic Usage](basic-usage.md)** - Learn fundamental operations
- **[Movement Control](movement.md)** - Advanced movement programming
- **[Sensor Integration](sensors.md)** - Working with sensors
- **[Examples](../examples/basic.md)** - More code examples

## 🆘 Need Help?

- **Issues**: Check [troubleshooting guide](troubleshooting.md)
- **Examples**: Browse `samples/` directory
- **API**: See [API reference](../api/robotdog.md)
- **Community**: Ask questions in discussions 