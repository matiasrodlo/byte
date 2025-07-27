# Movement Control Guide

Learn how to control Byte's movement and create custom gaits.

## 🚶 Basic Movement

### Predefined Actions

```python
from byte import RobotDog

with RobotDog() as dog:
    # Basic poses
    dog.do_action('sit')      # Sit down
    dog.do_action('stand')    # Stand up
    dog.do_action('lie')      # Lie down
    
    # Basic movements
    dog.do_action('walk')     # Walk forward
    dog.do_action('trot')     # Trot (faster walk)
    dog.do_action('turn_left') # Turn left
    dog.do_action('turn_right') # Turn right
    
    # Behaviors
    dog.do_action('bark')     # Bark
    dog.do_action('wag_tail') # Wag tail
    dog.do_action('stretch')  # Stretch
```

### Action Parameters

```python
# Walk with custom parameters
dog.do_action('walk', steps=10, speed=60)

# Turn with specific angle
dog.do_action('turn_left', angle=90)

# Custom movement duration
dog.do_action('trot', duration=5.0)
```

## 🦵 Leg Control

### Individual Leg Movement

```python
# Move specific leg to coordinates
dog.move_leg(leg_id, x, y, z)

# Examples
dog.move_leg(0, x=50, y=30, z=-20)   # Front left leg
dog.move_leg(1, x=-50, y=30, z=-20)  # Front right leg
dog.move_leg(2, x=50, y=-30, z=-20)  # Back left leg
dog.move_leg(3, x=-50, y=-30, z=-20) # Back right leg
```

### Leg Mapping

```
Leg IDs:
0: Front Left   (FL)
1: Front Right  (FR)
2: Back Left    (BL)
3: Back Right   (BR)

Servo Mapping per Leg:
- Servo 0: Hip joint (rotation)
- Servo 1: Thigh joint (pitch)
- Servo 2: Calf joint (pitch)
```

### Coordinate System

```python
# Coordinate system (in mm)
# X: Forward/Backward (+ forward, - backward)
# Y: Left/Right (+ left, - right)
# Z: Up/Down (+ up, - down)

# Example: Lift front left leg
dog.move_leg(0, x=0, y=0, z=30)  # Lift up

# Example: Step forward with front left leg
dog.move_leg(0, x=40, y=0, z=0)  # Step forward
```

## 🎭 Pose Control

### Predefined Poses

```python
# Set robot to specific poses
dog.set_pose('neutral')    # Standing pose
dog.set_pose('sit')        # Sitting pose
dog.set_pose('lie')        # Lying pose
dog.set_pose('stretch')    # Stretching pose
dog.set_pose('ready')      # Ready to walk pose
```

### Custom Poses

```python
# Define custom pose
custom_pose = {
    'legs': {
        0: {'x': 0, 'y': 0, 'z': 0},    # Front left
        1: {'x': 0, 'y': 0, 'z': 0},    # Front right
        2: {'x': 0, 'y': 0, 'z': 0},    # Back left
        3: {'x': 0, 'y': 0, 'z': 0}     # Back right
    },
    'head': {'pan': 0, 'tilt': 0},
    'tail': {'wag': 0, 'lift': 0}
}

# Apply custom pose
dog.set_custom_pose(custom_pose)
```

## 🚶‍♂️ Gait Generation

### Walking Gait

```python
from byte.walk import Walk

# Initialize walking gait
walk_gait = Walk(dog)

# Walk forward
walk_gait.forward(steps=10, speed=50)

# Walk backward
walk_gait.backward(steps=5, speed=40)

# Turn left
walk_gait.turn_left(angle=90, speed=30)

# Turn right
walk_gait.turn_right(angle=90, speed=30)
```

### Trotting Gait

```python
from byte.trot import Trot

# Initialize trotting gait
trot_gait = Trot(dog)

# Trot forward
trot_gait.forward(steps=15, speed=70)

# Trot with custom parameters
trot_gait.forward(
    steps=20,
    speed=60,
    step_height=15,
    step_duration=0.3
)
```

### Custom Gait Parameters

```python
# Walking parameters
walk_params = {
    'step_length': 40,      # mm
    'step_height': 20,      # mm
    'step_duration': 0.5,   # seconds
    'body_height': 80,      # mm
    'gait_cycle': 0.8       # duty factor
}

# Trotting parameters
trot_params = {
    'step_length': 50,      # mm
    'step_height': 25,      # mm
    'step_duration': 0.3,   # seconds
    'body_height': 70,      # mm
    'gait_cycle': 0.5       # duty factor
}
```

## 🎯 Advanced Movement

### Trajectory Planning

```python
import numpy as np

def plan_trajectory(start_pos, end_pos, steps=10):
    """Plan smooth trajectory between two positions."""
    trajectory = []
    
    for i in range(steps + 1):
        t = i / steps
        # Linear interpolation
        pos = start_pos * (1 - t) + end_pos * t
        trajectory.append(pos)
    
    return trajectory

# Use trajectory planning
start_pose = dog.get_current_pose()
end_pose = {'x': 100, 'y': 0, 'z': 0}

trajectory = plan_trajectory(start_pose, end_pose)

for pos in trajectory:
    dog.move_leg(0, **pos)
    time.sleep(0.1)
```

### Dynamic Balance

```python
def maintain_balance(dog):
    """Maintain balance using IMU feedback."""
    while True:
        pitch, roll, yaw = dog.imu.read()
        
        # Adjust legs based on orientation
        if abs(pitch) > 5:  # Forward/backward tilt
            # Adjust front/back legs
            if pitch > 0:
                dog.move_leg(0, z=-5)  # Lower front legs
                dog.move_leg(1, z=-5)
            else:
                dog.move_leg(2, z=-5)  # Lower back legs
                dog.move_leg(3, z=-5)
        
        if abs(roll) > 5:  # Left/right tilt
            # Adjust left/right legs
            if roll > 0:
                dog.move_leg(0, z=-5)  # Lower left legs
                dog.move_leg(2, z=-5)
            else:
                dog.move_leg(1, z=-5)  # Lower right legs
                dog.move_leg(3, z=-5)
        
        time.sleep(0.1)
```

### Obstacle Avoidance

```python
def obstacle_avoidance(dog):
    """Navigate while avoiding obstacles."""
    while True:
        distance = dog.ultrasonic.read()
        
        if distance < 20:  # Obstacle detected
            # Turn to avoid
            dog.do_action('turn_left', angle=45)
            time.sleep(1)
        else:
            # Continue walking
            dog.do_action('walk', steps=1)
        
        time.sleep(0.5)
```

## 🎨 Movement Sequences

### Dance Routine

```python
def dance_routine(dog):
    """Perform a dance routine."""
    sequence = [
        ('stand', 2),
        ('wag_tail', 1),
        ('turn_left', {'angle': 90}),
        ('turn_right', {'angle': 90}),
        ('bark', 1),
        ('stretch', 2),
        ('sit', 1)
    ]
    
    for action, params in sequence:
        if isinstance(params, dict):
            dog.do_action(action, **params)
        else:
            dog.do_action(action)
            time.sleep(params)
```

### Patrol Pattern

```python
def patrol_pattern(dog):
    """Patrol in a square pattern."""
    pattern = [
        ('walk', {'steps': 10}),
        ('turn_left', {'angle': 90}),
        ('walk', {'steps': 10}),
        ('turn_left', {'angle': 90}),
        ('walk', {'steps': 10}),
        ('turn_left', {'angle': 90}),
        ('walk', {'steps': 10}),
        ('turn_left', {'angle': 90})
    ]
    
    for action, params in pattern:
        dog.do_action(action, **params)
        time.sleep(1)
```

## 🔧 Movement Configuration

### Speed and Acceleration

```python
# Configure movement parameters
dog = RobotDog(
    servo_speed=50,    # Movement speed (0-100)
    servo_accel=20     # Acceleration (0-100)
)

# Dynamic speed adjustment
dog.servo_speed = 75   # Increase speed
dog.servo_accel = 30   # Increase acceleration
```

### Movement Constraints

```python
# Set movement limits
movement_limits = {
    'max_step_length': 60,    # Maximum step length (mm)
    'max_step_height': 30,    # Maximum step height (mm)
    'max_turn_angle': 45,     # Maximum turn angle (degrees)
    'min_distance': 15        # Minimum obstacle distance (cm)
}

# Apply limits
dog.set_movement_limits(movement_limits)
```

## 📊 Movement Monitoring

### Status Tracking

```python
# Check if robot is moving
if dog.is_moving():
    print("Robot is currently moving")
else:
    print("Robot is stationary")

# Get current pose
current_pose = dog.get_current_pose()
print(f"Current pose: {current_pose}")

# Get movement history
history = dog.get_movement_history()
print(f"Movement history: {history}")
```

### Performance Metrics

```python
def measure_performance(dog):
    """Measure movement performance."""
    import time
    
    # Measure walking speed
    start_time = time.time()
    dog.do_action('walk', steps=10)
    end_time = time.time()
    
    duration = end_time - start_time
    speed = 10 / duration  # steps per second
    
    print(f"Walking speed: {speed:.2f} steps/second")
    
    # Measure turning accuracy
    start_angle = dog.imu.read()[2]  # Yaw
    dog.do_action('turn_left', angle=90)
    end_angle = dog.imu.read()[2]
    
    actual_turn = end_angle - start_angle
    accuracy = abs(90 - actual_turn)
    
    print(f"Turn accuracy: {accuracy:.1f} degrees")
```

## 🐛 Troubleshooting

### Common Movement Issues

**Robot falls over**
```python
# Reduce speed and check surface
dog = RobotDog(servo_speed=30, servo_accel=10)
dog.do_action('walk', speed=30)
```

**Uneven movement**
```python
# Calibrate servos
dog.calibrate_servos()

# Check individual leg positions
for i in range(4):
    pos = dog.get_leg_position(i)
    print(f"Leg {i}: {pos}")
```

**Slow response**
```python
# Increase servo speed
dog.servo_speed = 75
dog.servo_accel = 30

# Check for mechanical obstructions
```

## 📚 Next Steps

- **[Sensor Integration](sensors.md)** - Using sensors for movement
- **[AI Integration](ai-integration.md)** - AI-controlled movement
- **[Examples](../examples/advanced.md)** - Advanced movement examples
- **[API Reference](../api/movement.md)** - Complete movement API 