# RobotDog API

## Initialization

```python
from byte import RobotDog

dog = RobotDog(
    servo_speed=50,      # Movement speed (0-100)
    servo_accel=20,      # Acceleration (0-100)
    imu_calibration=True # Auto-calibrate IMU
)
```

## Actions

```python
# Basic poses
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
```

## Sensors

```python
# Distance
distance = dog.ultrasonic.read()

# Touch
left, right = dog.dual_touch.read()

# IMU
pitch, roll, yaw = dog.imu.read()

# Sound direction
angle = dog.sound_direction.read()
```

## Movement Control

```python
# Direct leg control
dog.legs_move(angles, speed=50)

# Head control
dog.head_move(yaw, roll, pitch, speed=50)

# Tail control
dog.tail_move(angle, speed=50)
```

## RGB LED

```python
# Set color
dog.rgb_strip.set_color(r, g, b)

# Set mode
dog.rgb_strip.set_mode('breath', color='red', bps=1)

# Turn off
dog.rgb_strip.off()
```

## Audio

```python
# Play sound
dog.speak('bark', volume=100)

# Play sound file
dog.sound_play('angry.wav', volume=80)
```

## Cleanup

```python
# Always close when done
dog.close()
```
