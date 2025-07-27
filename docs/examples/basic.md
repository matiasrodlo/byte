# Basic Examples

Learn Byte robot programming with these practical examples.

## 🚀 Getting Started Examples

### 1. Hello World

```python
from byte import RobotDog

# Initialize robot
dog = RobotDog()

# Simple greeting
print("Hello from Byte!")
dog.do_action('bark')
dog.do_action('wag_tail')

# Cleanup
dog.close()
```

### 2. Basic Movement Sequence

```python
from byte import RobotDog
import time

with RobotDog() as dog:
    print("Starting movement sequence...")
    
    # Stand up
    dog.do_action('stand')
    time.sleep(2)
    
    # Walk forward
    dog.do_action('walk', steps=5)
    time.sleep(1)
    
    # Turn around
    dog.do_action('turn_left', angle=180)
    time.sleep(1)
    
    # Walk back
    dog.do_action('walk', steps=5)
    time.sleep(1)
    
    # Sit down
    dog.do_action('sit')
    
    print("Sequence complete!")
```

## 📡 Sensor Examples

### 3. Distance Monitoring

```python
from byte import RobotDog
import time

with RobotDog() as dog:
    print("Monitoring distance... Press Ctrl+C to stop")
    
    try:
        while True:
            distance = dog.ultrasonic.read()
            print(f"Distance: {distance:.1f} cm")
            
            # Visual feedback
            if distance < 20:
                dog.rgb_strip.set_color(255, 0, 0)  # Red for close
            elif distance < 50:
                dog.rgb_strip.set_color(255, 255, 0)  # Yellow for medium
            else:
                dog.rgb_strip.set_color(0, 255, 0)  # Green for far
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nMonitoring stopped")
```

### 4. Touch Response

```python
from byte import RobotDog
import time

with RobotDog() as dog:
    print("Touch sensors active... Press Ctrl+C to stop")
    
    try:
        while True:
            left_touch, right_touch = dog.dual_touch.read()
            
            if left_touch:
                print("Left side touched!")
                dog.do_action('turn_left')
                dog.rgb_strip.set_color(255, 0, 0)  # Red
                
            elif right_touch:
                print("Right side touched!")
                dog.do_action('turn_right')
                dog.rgb_strip.set_color(0, 0, 255)  # Blue
                
            else:
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nTouch monitoring stopped")
```

### 5. IMU Orientation

```python
from byte import RobotDog
import time

with RobotDog() as dog:
    print("Reading IMU data... Press Ctrl+C to stop")
    
    try:
        while True:
            pitch, roll, yaw = dog.imu.read()
            print(f"Pitch: {pitch:6.1f}° | Roll: {roll:6.1f}° | Yaw: {yaw:6.1f}°")
            
            # React to orientation
            if abs(pitch) > 15:
                print("Robot is tilted!")
                dog.rgb_strip.set_color(255, 165, 0)  # Orange
            else:
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nIMU monitoring stopped")
```

## 🎮 Interactive Examples

### 6. Obstacle Avoidance

```python
from byte import RobotDog
import time

with RobotDog() as dog:
    print("Obstacle avoidance mode... Press Ctrl+C to stop")
    
    try:
        while True:
            distance = dog.ultrasonic.read()
            
            if distance < 15:
                print(f"Obstacle detected at {distance} cm!")
                dog.do_action('sit')
                dog.rgb_strip.set_color(255, 0, 0)  # Red
                time.sleep(1)
                
                # Turn to avoid
                dog.do_action('turn_left', angle=90)
                time.sleep(1)
                
            elif distance < 30:
                print(f"Caution: {distance} cm ahead")
                dog.rgb_strip.set_color(255, 255, 0)  # Yellow
                dog.do_action('walk', steps=1)
                
            else:
                print(f"Clear path: {distance} cm")
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                dog.do_action('walk', steps=2)
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.do_action('sit')
        dog.rgb_strip.off()
        print("\nObstacle avoidance stopped")
```

### 7. Voice-Controlled Robot

```python
from byte import RobotDog
import time

def voice_control():
    with RobotDog() as dog:
        print("Voice control mode...")
        print("Commands: 'sit', 'stand', 'walk', 'bark', 'quit'")
        
        while True:
            try:
                command = input("Enter command: ").lower().strip()
                
                if command == 'quit':
                    break
                elif command == 'sit':
                    dog.do_action('sit')
                elif command == 'stand':
                    dog.do_action('stand')
                elif command == 'walk':
                    dog.do_action('walk', steps=3)
                elif command == 'bark':
                    dog.do_action('bark')
                elif command == 'turn_left':
                    dog.do_action('turn_left')
                elif command == 'turn_right':
                    dog.do_action('turn_right')
                else:
                    print("Unknown command. Try: sit, stand, walk, bark, turn_left, turn_right, quit")
                    
            except KeyboardInterrupt:
                break
                
        print("Voice control ended")

if __name__ == "__main__":
    voice_control()
```

## 🎨 LED Animation Examples

### 8. Rainbow Effect

```python
from byte import RobotDog
import time
import math

def rainbow_effect():
    with RobotDog() as dog:
        print("Rainbow LED effect... Press Ctrl+C to stop")
        
        try:
            while True:
                for i in range(360):
                    # Convert angle to RGB
                    r = int(255 * (1 + math.sin(math.radians(i))) / 2)
                    g = int(255 * (1 + math.sin(math.radians(i + 120))) / 2)
                    b = int(255 * (1 + math.sin(math.radians(i + 240))) / 2)
                    
                    dog.rgb_strip.set_color(r, g, b)
                    time.sleep(0.05)
                    
        except KeyboardInterrupt:
            dog.rgb_strip.off()
            print("\nRainbow effect stopped")

if __name__ == "__main__":
    rainbow_effect()
```

### 9. Breathing Effect

```python
from byte import RobotDog
import time
import math

def breathing_effect():
    with RobotDog() as dog:
        print("Breathing LED effect... Press Ctrl+C to stop")
        
        try:
            while True:
                for i in range(100):
                    # Breathing pattern
                    intensity = int(255 * (0.3 + 0.7 * math.sin(math.radians(i * 3.6))))
                    dog.rgb_strip.set_color(intensity, 0, intensity)  # Purple breathing
                    time.sleep(0.05)
                    
        except KeyboardInterrupt:
            dog.rgb_strip.off()
            print("\nBreathing effect stopped")

if __name__ == "__main__":
    breathing_effect()
```

## 🔧 Utility Examples

### 10. Robot Status Monitor

```python
from byte import RobotDog
import time

def status_monitor():
    with RobotDog() as dog:
        print("Robot status monitor... Press Ctrl+C to stop")
        
        try:
            while True:
                # Read all sensors
                distance = dog.ultrasonic.read()
                left_touch, right_touch = dog.dual_touch.read()
                pitch, roll, yaw = dog.imu.read()
                
                # Display status
                print("\n" + "="*50)
                print("ROBOT STATUS")
                print("="*50)
                print(f"Distance:     {distance:6.1f} cm")
                print(f"Touch Left:   {left_touch}")
                print(f"Touch Right:  {right_touch}")
                print(f"Pitch:        {pitch:6.1f}°")
                print(f"Roll:         {roll:6.1f}°")
                print(f"Yaw:          {yaw:6.1f}°")
                print(f"Moving:       {dog.is_moving()}")
                print("="*50)
                
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\nStatus monitoring stopped")

if __name__ == "__main__":
    status_monitor()
```

### 11. Calibration Helper

```python
from byte import RobotDog
import time

def calibration_helper():
    with RobotDog() as dog:
        print("Calibration Helper")
        print("This will help you calibrate your robot")
        
        # Servo calibration
        print("\n1. Servo Calibration")
        input("Place robot on flat surface and press Enter...")
        dog.calibrate_servos()
        print("Servo calibration complete!")
        
        # IMU calibration
        print("\n2. IMU Calibration")
        input("Keep robot still and press Enter...")
        dog.calibrate_imu()
        print("IMU calibration complete!")
        
        # Test movements
        print("\n3. Testing movements...")
        dog.do_action('sit')
        time.sleep(1)
        dog.do_action('stand')
        time.sleep(1)
        dog.do_action('sit')
        
        print("\nCalibration complete! Your robot is ready to use.")

if __name__ == "__main__":
    calibration_helper()
```

## 📚 Next Steps

- **[Advanced Examples](advanced.md)** - Complex behaviors and AI integration
- **[Movement Guide](../guides/movement.md)** - Advanced movement programming
- **[Sensor Guide](../guides/sensors.md)** - Detailed sensor usage
- **[API Reference](../api/robotdog.md)** - Complete API documentation 