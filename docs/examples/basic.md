# Examples

## Basic Initialization

```python
from byte import RobotDog

# RobotDog initializes servos, sensors, and loads action dictionary
dog = RobotDog()

# Actions are executed via do_action() which moves servos
dog.do_action('sit')
dog.do_action('stand')

# Always close to release resources
dog.close()
```

## Reading Sensors

```python
from byte import RobotDog

dog = RobotDog()

# Ultrasonic sensor reading
distance = dog.ultrasonic.read()  # Returns float in cm
print(f"Distance: {distance}")

# Touch sensors
left, right = dog.dual_touch.read()  # Returns (bool, bool)
print(f"Touch: left={left}, right={right}")

# IMU orientation
pitch, roll, yaw = dog.imu.read()  # Returns (float, float, float) in degrees
print(f"Orientation: pitch={pitch}, roll={roll}, yaw={yaw}")

dog.close()
```

## Simple Movement

```python
from byte import RobotDog
import time

dog = RobotDog()

# do_action() loads servo angles and executes movement
dog.do_action('stand')
time.sleep(2)  # Wait for movement to complete
dog.do_action('sit')

dog.close()
```

## Obstacle Avoidance

```python
from byte import RobotDog
import time

dog = RobotDog()

try:
    while True:
        # Read ultrasonic sensor
        distance = dog.ultrasonic.read()
        
        # If obstacle detected, turn
        if distance < 20:
            dog.do_action('turn_left')
        else:
            dog.do_action('walk', steps=1)
        
        time.sleep(0.5)
except KeyboardInterrupt:
    dog.do_action('sit')
    dog.close()
```

## Direct Servo Control

```python
from byte import RobotDog

dog = RobotDog()

# Move specific leg to coordinates (inverse kinematics)
dog.move_leg(0, x=50, y=30, z=-20)  # Leg 0 = front left

# Control head servos directly
dog.head_move(yaw=0, roll=0, pitch=-30, speed=50)

# Control tail servo
dog.tail_move(angle=45, speed=50)

dog.close()
```

## RGB LED Control

```python
from byte import RobotDog
import time

dog = RobotDog()

# Set RGB color (0-255 per channel)
dog.rgb_strip.set_color(255, 0, 0)  # Red
time.sleep(1)
dog.rgb_strip.set_color(0, 255, 0)  # Green
time.sleep(1)
dog.rgb_strip.off()

dog.close()
```

## More Examples

See `samples/` directory:
- `samples/demos/` - Basic sensor and servo demos
- `samples/advanced/` - Complex behaviors (patrol, face tracking, balance)
- `samples/tests/` - Hardware testing scripts
