# Quick Start

## Installation

```bash
git clone https://github.com/matiasrodlo/byte.git
cd byte
pip install -e .
```

## Basic Usage

```python
from byte import RobotDog

dog = RobotDog()
dog.do_action('sit')
dog.do_action('stand')
dog.close()
```

## How Actions Work

Byte uses predefined servo angle sequences stored in `ActionDict`. Each action returns servo angles for legs, head, or tail.

```python
from byte import RobotDog

dog = RobotDog()

# Actions are executed via do_action()
dog.do_action('sit')      # Loads sit angles, moves leg servos
dog.do_action('walk', steps=5)  # Executes walk gait 5 times
dog.do_action('bark')     # Head movement + sound

dog.close()
```

## How Sensors Work

Byte reads sensors through dedicated objects:

```python
from byte import RobotDog

dog = RobotDog()

# Ultrasonic: HC-SR04 on GPIO23/24, returns distance in cm
distance = dog.ultrasonic.read()

# Dual touch: Capacitive sensors on GPIO17/27, returns (left, right) booleans
left, right = dog.dual_touch.read()

# IMU: SH3001 on I2C, returns (pitch, roll, yaw) in degrees
pitch, roll, yaw = dog.imu.read()

# Sound direction: SPI interface, returns angle 0-360° with 20° resolution
angle = dog.sound_direction.read()

dog.close()
```

## How Movement Works

Byte uses inverse kinematics to calculate leg positions. Body pose is defined by:
- Position: [x, y, z] in mm relative to body center
- Orientation: roll, pitch, yaw in radians

```python
from byte import RobotDog

dog = RobotDog()

# Direct leg control: move leg to coordinates
dog.move_leg(0, x=50, y=30, z=-20)  # Front left leg

# Head control: yaw, roll, pitch angles
dog.head_move(yaw=0, roll=0, pitch=-30, speed=50)

# Tail control: angle in degrees
dog.tail_move(angle=45, speed=50)

dog.close()
```

## Next Steps

- [Movement Guide](movement.md) - Gait generation, inverse kinematics
- [Sensors Guide](sensors.md) - Sensor protocols and data formats
- [API Reference](../api/robotdog.md) - Complete method reference
