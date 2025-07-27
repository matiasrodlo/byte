# Movement Control

## How Actions Work

Byte stores predefined servo angle sequences in `ActionDict`. `do_action()` loads angles and moves servos.

```python
from byte import RobotDog

dog = RobotDog()

# do_action() loads angles from ActionDict and executes
dog.do_action('sit')      # Loads sit angles, moves leg servos
dog.do_action('walk', steps=5)  # Executes walk gait 5 cycles
dog.do_action('bark')     # Head movement sequence + sound

dog.close()
```

## Available Actions

```python
# Poses
dog.do_action('sit')
dog.do_action('stand')
dog.do_action('lie')

# Movement
dog.do_action('walk', steps=10, speed=60)
dog.do_action('trot')
dog.do_action('turn_left', angle=90)

# Behaviors
dog.do_action('bark')
dog.do_action('wag_tail')
dog.do_action('stretch')
```

## Inverse Kinematics

Byte uses IK to convert foot position [x,y,z] to joint angles. Body dimensions: 117mm × 98mm. Leg segments: 42mm leg, 76mm foot.

```python
from byte import RobotDog

dog = RobotDog()

# Move leg to coordinates (IK calculates joint angles)
dog.move_leg(0, x=50, y=30, z=-20)  # Front left leg

# Coordinate system (mm):
# X: forward (+), backward (-)
# Y: left (+), right (-)
# Z: up (+), down (-)

dog.close()
```

## Leg Mapping

```
Leg IDs:
0: Front Left   (servos on PWM 2,3)
1: Front Right (servos on PWM 7,8)
2: Back Left    (servos on PWM 0,1)
3: Back Right   (servos on PWM 10,11)

Each leg has 2 servos:
- Hip joint (rotation)
- Thigh joint (pitch)
```

## Gait Generation

Byte uses `Walk` and `Trot` classes for gaits. Gaits cycle leg positions over time.

```python
from byte.walk import Walk
from byte.trot import Trot

dog = RobotDog()

# Walk gait
walk = Walk(dog)
walk.forward(steps=10, speed=50)

# Trot gait (faster, diagonal pairs)
trot = Trot(dog)
trot.forward(steps=15, speed=70)

dog.close()
```

## Direct Servo Control

```python
from byte import RobotDog

dog = RobotDog()

# Control all legs with angle array
# Array: [FL_hip, FL_thigh, FR_hip, FR_thigh, BL_hip, BL_thigh, BR_hip, BR_thigh]
angles = [0, 45, 0, 45, 0, 45, 0, 45]
dog.legs_move(angles, speed=50)

# Head control (yaw, roll, pitch in degrees)
dog.head_move(yaw=0, roll=0, pitch=-30, speed=50)

# Tail control (angle in degrees)
dog.tail_move(angle=45, speed=50)

dog.close()
```

## Body Pose Control

Byte's body pose defined by position [x,y,z] and orientation [roll,pitch,yaw].

```python
from byte import RobotDog

dog = RobotDog()

# Set body height (20-95mm)
dog.body_height = 80

# Adjust body position
dog.pose = [0, 0, 80]  # [x, y, z] in mm

# Adjust orientation
dog.rpy = [0, 0, 0]  # [roll, pitch, yaw] in radians

dog.close()
```

## Speed and Acceleration

```python
from byte import RobotDog

# Set servo speed limits
dog = RobotDog(servo_speed=50, servo_accel=20)

# Servo speed limits (degrees/second):
# Head: 300°/s
# Legs: 428°/s
# Tail: 500°/s

# Adjust dynamically
dog.servo_speed = 75
dog.servo_accel = 30
```

## Balance Control

Byte uses IMU feedback for balance. PID controller adjusts leg positions.

```python
from byte import RobotDog
import time

dog = RobotDog()

while True:
    pitch, roll, yaw = dog.imu.read()
    
    # Adjust legs based on orientation
    if abs(pitch) > 5:  # Forward/backward tilt
        if pitch > 0:
            dog.move_leg(0, z=-5)  # Lower front legs
            dog.move_leg(1, z=-5)
        else:
            dog.move_leg(2, z=-5)  # Lower back legs
            dog.move_leg(3, z=-5)
    
    time.sleep(0.1)
```

## Custom Movements

```python
from byte import RobotDog
import time

dog = RobotDog()

# Create custom movement sequence
def custom_walk():
    # Lift front left
    dog.move_leg(0, z=20)
    time.sleep(0.2)
    # Step forward
    dog.move_leg(0, x=40, z=0)
    time.sleep(0.2)
    # Lower leg
    dog.move_leg(0, z=-20)
    time.sleep(0.2)

custom_walk()
dog.close()
```

## Troubleshooting

**Servos not moving**: Check 5V power, PWM connections, run `dog.calibrate_servos()`

**Unstable walking**: Calibrate IMU, reduce speed, check surface level

**Slow response**: Increase `servo_speed`, check for mechanical obstructions

See [Troubleshooting Guide](troubleshooting.md) for details.
