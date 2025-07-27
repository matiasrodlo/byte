# RobotDog Class Reference

The main `RobotDog` class provides the primary interface for controlling the Byte robot.

## 📋 Class Overview

```python
from byte import RobotDog

class RobotDog:
    """
    Main robot control class for Byte quadruped robot.
    
    Provides interfaces for movement, sensor reading, and behavior control.
    """
```

## 🚀 Initialization

### Constructor

```python
RobotDog(
    servo_speed=50,
    servo_accel=20,
    imu_calibration=True,
    auto_calibrate=True
)
```

**Parameters:**
- `servo_speed` (int): Servo movement speed (0-100, default: 50)
- `servo_accel` (int): Servo acceleration (0-100, default: 20)
- `imu_calibration` (bool): Auto-calibrate IMU on startup (default: True)
- `auto_calibrate` (bool): Auto-calibrate servos on startup (default: True)

### Example

```python
from byte import RobotDog

# Basic initialization
dog = RobotDog()

# Custom settings
dog = RobotDog(
    servo_speed=75,      # Faster movements
    servo_accel=30,      # Higher acceleration
    imu_calibration=False # Manual IMU calibration
)
```

## 🎯 Core Methods

### Movement Control

#### `do_action(action_name, **kwargs)`

Execute predefined robot actions.

```python
# Basic actions
dog.do_action('sit')
dog.do_action('stand')
dog.do_action('walk')
dog.do_action('trot')

# Actions with parameters
dog.do_action('walk', steps=10, speed=60)
dog.do_action('turn_left', angle=90)
```

**Available Actions:**
- `sit`, `stand`, `lie` - Basic poses
- `walk`, `trot`, `run` - Forward movement
- `turn_left`, `turn_right` - Turning
- `bark`, `wag_tail`, `stretch` - Behaviors
- `pushup`, `howling` - Advanced behaviors

#### `move_leg(leg_id, x, y, z)`

Move individual leg to specific coordinates.

```python
# Move front left leg (leg 0)
dog.move_leg(0, x=50, y=30, z=-20)

# Move back right leg (leg 3)
dog.move_leg(3, x=-50, y=30, z=-20)
```

**Parameters:**
- `leg_id` (int): Leg number (0-3: front left, front right, back left, back right)
- `x`, `y`, `z` (float): Target coordinates in mm

#### `set_pose(pose_name)`

Set robot to predefined poses.

```python
dog.set_pose('neutral')    # Standing pose
dog.set_pose('sit')        # Sitting pose
dog.set_pose('lie')        # Lying pose
dog.set_pose('stretch')    # Stretching pose
```

### Sensor Reading

#### `ultrasonic.read()`

Read distance from ultrasonic sensor.

```python
distance = dog.ultrasonic.read()
print(f"Distance: {distance} cm")
```

**Returns:** Distance in centimeters (float)

#### `dual_touch.read()`

Read touch sensor states.

```python
left_touch, right_touch = dog.dual_touch.read()
print(f"Left: {left_touch}, Right: {right_touch}")
```

**Returns:** Tuple of boolean values (left_touch, right_touch)

#### `imu.read()`

Read IMU sensor data.

```python
pitch, roll, yaw = dog.imu.read()
print(f"Pitch: {pitch}°, Roll: {roll}°, Yaw: {yaw}°")
```

**Returns:** Tuple of angles in degrees (pitch, roll, yaw)

#### `sound_direction.read()`

Read sound direction angle.

```python
angle = dog.sound_direction.read()
print(f"Sound detected at {angle}°")
```

**Returns:** Angle in degrees (float, 0-360)

### LED Control

#### `rgb_strip.set_color(r, g, b)`

Set RGB LED strip color.

```python
# Set colors
dog.rgb_strip.set_color(255, 0, 0)    # Red
dog.rgb_strip.set_color(0, 255, 0)    # Green
dog.rgb_strip.set_color(0, 0, 255)    # Blue
dog.rgb_strip.set_color(255, 255, 0)  # Yellow
```

**Parameters:**
- `r`, `g`, `b` (int): RGB values (0-255)

#### `rgb_strip.off()`

Turn off RGB LED strip.

```python
dog.rgb_strip.off()
```

### Calibration

#### `calibrate_servos()`

Calibrate all servo motors.

```python
print("Calibrating servos...")
dog.calibrate_servos()
print("Calibration complete!")
```

#### `calibrate_imu()`

Calibrate IMU sensor.

```python
print("Calibrating IMU...")
dog.calibrate_imu()
print("IMU calibration complete!")
```

## 🔧 Configuration

### Properties

#### `servo_speed`
Get or set servo movement speed.

```python
# Get current speed
current_speed = dog.servo_speed

# Set new speed
dog.servo_speed = 75
```

#### `servo_accel`
Get or set servo acceleration.

```python
# Get current acceleration
current_accel = dog.servo_accel

# Set new acceleration
dog.servo_accel = 30
```

### Status Methods

#### `is_moving()`

Check if robot is currently moving.

```python
if dog.is_moving():
    print("Robot is in motion")
else:
    print("Robot is stationary")
```

**Returns:** Boolean indicating movement status

#### `get_battery_level()`

Get battery level (if supported).

```python
level = dog.get_battery_level()
print(f"Battery: {level}%")
```

**Returns:** Battery percentage (0-100) or None if not supported

## 🧹 Resource Management

### `close()`

Properly close robot connections and cleanup resources.

```python
try:
    dog = RobotDog()
    # ... robot operations ...
finally:
    dog.close()  # Always close when done
```

### Context Manager

Use as context manager for automatic cleanup.

```python
with RobotDog() as dog:
    dog.do_action('sit')
    dog.do_action('stand')
    # Automatic cleanup when exiting context
```

## 📊 Error Handling

### Common Exceptions

```python
from byte import RobotDog

try:
    dog = RobotDog()
    dog.do_action('walk')
except ConnectionError:
    print("Failed to connect to robot hardware")
except ValueError as e:
    print(f"Invalid parameter: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    dog.close()
```

## 📝 Complete Example

```python
from byte import RobotDog
import time

def robot_demo():
    """Complete robot demonstration."""
    
    with RobotDog() as dog:
        print("Starting Byte demo...")
        
        # Basic movements
        dog.do_action('sit')
        time.sleep(1)
        
        dog.do_action('stand')
        time.sleep(1)
        
        # Sensor reading
        distance = dog.ultrasonic.read()
        print(f"Distance: {distance} cm")
        
        # LED feedback
        dog.rgb_strip.set_color(0, 255, 0)  # Green for ready
        
        # Walking
        dog.do_action('walk', steps=5)
        
        # Behavior
        dog.do_action('bark')
        dog.do_action('wag_tail')
        
        # Final pose
        dog.do_action('sit')
        dog.rgb_strip.off()
        
        print("Demo complete!")

if __name__ == "__main__":
    robot_demo()
```

## 🔗 Related Documentation

- **[Movement API](movement.md)** - Advanced movement control
- **[Sensor API](sensors.md)** - Detailed sensor documentation
- **[Action API](actions.md)** - Predefined actions reference
- **[Examples](../examples/basic.md)** - Usage examples 