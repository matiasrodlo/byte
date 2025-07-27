# Sensor Integration Guide

Learn how to work with Byte's sensors for intelligent robot behavior.

## 📡 Sensor Overview

Byte comes equipped with multiple sensors:

- **Ultrasonic Sensor** - Distance measurement
- **Dual Touch Sensors** - Touch detection
- **IMU Sensor (SH3001)** - Orientation and motion
- **Sound Direction Sensor** - Audio localization
- **Camera Module** - Visual perception

## 🔍 Ultrasonic Sensor

### Basic Usage

```python
from byte import RobotDog

with RobotDog() as dog:
    # Read distance
    distance = dog.ultrasonic.read()
    print(f"Distance: {distance} cm")
```

### Continuous Monitoring

```python
import time

def monitor_distance(dog):
    """Continuously monitor distance."""
    print("Distance monitoring... Press Ctrl+C to stop")
    
    try:
        while True:
            distance = dog.ultrasonic.read()
            print(f"Distance: {distance:.1f} cm")
            
            # React to distance
            if distance < 10:
                print("⚠️  Very close!")
                dog.rgb_strip.set_color(255, 0, 0)  # Red
            elif distance < 30:
                print("⚠️  Close")
                dog.rgb_strip.set_color(255, 255, 0)  # Yellow
            else:
                print("✅ Safe distance")
                dog.rgb_strip.set_color(0, 255, 0)  # Green
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nMonitoring stopped")
```

### Obstacle Avoidance

```python
def obstacle_avoidance(dog):
    """Navigate while avoiding obstacles."""
    while True:
        distance = dog.ultrasonic.read()
        
        if distance < 20:
            print(f"Obstacle at {distance} cm - avoiding...")
            dog.do_action('sit')
            dog.do_action('turn_left', angle=45)
            time.sleep(1)
        else:
            dog.do_action('walk', steps=1)
        
        time.sleep(0.5)
```

## 👆 Touch Sensors

### Basic Usage

```python
# Read touch sensors
left_touch, right_touch = dog.dual_touch.read()
print(f"Left touch: {left_touch}, Right touch: {right_touch}")
```

### Touch Response

```python
def touch_response(dog):
    """Respond to touch inputs."""
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

### Pet Detection

```python
def pet_detection(dog):
    """Detect petting behavior."""
    pet_count = 0
    
    while True:
        left_touch, right_touch = dog.dual_touch.read()
        
        if left_touch or right_touch:
            pet_count += 1
            print(f"Pet detected! Count: {pet_count}")
            
            if pet_count >= 3:
                print("Happy! Wagging tail!")
                dog.do_action('wag_tail')
                dog.do_action('bark')
                pet_count = 0
        else:
            pet_count = max(0, pet_count - 1)  # Decay
        
        time.sleep(0.2)
```

## 🧭 IMU Sensor (SH3001)

### Basic Usage

```python
# Read orientation
pitch, roll, yaw = dog.imu.read()
print(f"Pitch: {pitch:.1f}°, Roll: {roll:.1f}°, Yaw: {yaw:.1f}°")
```

### Orientation Monitoring

```python
def monitor_orientation(dog):
    """Monitor robot orientation."""
    print("Orientation monitoring... Press Ctrl+C to stop")
    
    try:
        while True:
            pitch, roll, yaw = dog.imu.read()
            
            print(f"Pitch: {pitch:6.1f}° | Roll: {roll:6.1f}° | Yaw: {yaw:6.1f}°")
            
            # Check for tilting
            if abs(pitch) > 15:
                print("⚠️  Robot is tilted forward/backward!")
                dog.rgb_strip.set_color(255, 165, 0)  # Orange
            elif abs(roll) > 15:
                print("⚠️  Robot is tilted left/right!")
                dog.rgb_strip.set_color(255, 165, 0)  # Orange
            else:
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nOrientation monitoring stopped")
```

### Balance Control

```python
def balance_control(dog):
    """Maintain balance using IMU feedback."""
    print("Balance control active... Press Ctrl+C to stop")
    
    try:
        while True:
            pitch, roll, yaw = dog.imu.read()
            
            # Adjust based on orientation
            if abs(pitch) > 5:  # Forward/backward tilt
                if pitch > 0:  # Tilted forward
                    # Lower front legs
                    dog.move_leg(0, z=-5)  # Front left
                    dog.move_leg(1, z=-5)  # Front right
                else:  # Tilted backward
                    # Lower back legs
                    dog.move_leg(2, z=-5)  # Back left
                    dog.move_leg(3, z=-5)  # Back right
            
            if abs(roll) > 5:  # Left/right tilt
                if roll > 0:  # Tilted left
                    # Lower left legs
                    dog.move_leg(0, z=-5)  # Front left
                    dog.move_leg(2, z=-5)  # Back left
                else:  # Tilted right
                    # Lower right legs
                    dog.move_leg(1, z=-5)  # Front right
                    dog.move_leg(3, z=-5)  # Back right
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nBalance control stopped")
```

### Calibration

```python
def calibrate_imu(dog):
    """Calibrate IMU sensor."""
    print("IMU Calibration")
    print("Keep robot still on level surface...")
    
    # Calibrate
    dog.calibrate_imu()
    
    # Verify calibration
    pitch, roll, yaw = dog.imu.read()
    print(f"Calibrated values:")
    print(f"  Pitch: {pitch:.1f}°")
    print(f"  Roll: {roll:.1f}°")
    print(f"  Yaw: {yaw:.1f}°")
    
    if abs(pitch) < 2 and abs(roll) < 2:
        print("✅ Calibration successful!")
    else:
        print("⚠️  Calibration may need adjustment")
```

## 🔊 Sound Direction Sensor

### Basic Usage

```python
# Read sound direction
angle = dog.sound_direction.read()
print(f"Sound detected at {angle}°")
```

### Sound Tracking

```python
def sound_tracking(dog):
    """Track sound direction."""
    print("Sound tracking active... Press Ctrl+C to stop")
    
    try:
        while True:
            angle = dog.sound_direction.read()
            
            if angle is not None:
                print(f"Sound at {angle:.1f}°")
                
                # Turn towards sound
                if angle > 180:
                    dog.do_action('turn_left', angle=min(angle - 180, 45))
                else:
                    dog.do_action('turn_right', angle=min(180 - angle, 45))
                    
                dog.rgb_strip.set_color(0, 255, 255)  # Cyan
            else:
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nSound tracking stopped")
```

## 📷 Camera Integration

### Basic Camera Usage

```python
import cv2
import numpy as np

def camera_demo():
    """Basic camera demonstration."""
    # Initialize camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Camera demo - Press 'q' to quit")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame")
                break
            
            # Display frame
            cv2.imshow('Byte Camera', frame)
            
            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    camera_demo()
```

### Face Detection

```python
def face_detection():
    """Detect faces using camera."""
    import cv2
    
    # Load face cascade
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    )
    
    cap = cv2.VideoCapture(0)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )
            
            # Draw rectangles around faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            # Display count
            cv2.putText(frame, f'Faces: {len(faces)}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Face Detection', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
```

## 🔄 Multi-Sensor Integration

### Sensor Fusion

```python
def sensor_fusion(dog):
    """Combine multiple sensors for intelligent behavior."""
    print("Multi-sensor fusion active... Press Ctrl+C to stop")
    
    try:
        while True:
            # Read all sensors
            distance = dog.ultrasonic.read()
            left_touch, right_touch = dog.dual_touch.read()
            pitch, roll, yaw = dog.imu.read()
            sound_angle = dog.sound_direction.read()
            
            # Decision making based on multiple sensors
            if distance < 15:
                print("⚠️  Obstacle detected - stopping")
                dog.do_action('sit')
                dog.rgb_strip.set_color(255, 0, 0)  # Red
                
            elif left_touch or right_touch:
                print("👆 Touch detected - responding")
                if left_touch:
                    dog.do_action('turn_left')
                else:
                    dog.do_action('turn_right')
                dog.rgb_strip.set_color(0, 0, 255)  # Blue
                
            elif sound_angle is not None:
                print(f"🔊 Sound detected at {sound_angle}° - investigating")
                dog.rgb_strip.set_color(0, 255, 255)  # Cyan
                
            elif abs(pitch) > 10 or abs(roll) > 10:
                print("⚠️  Unstable orientation - balancing")
                dog.rgb_strip.set_color(255, 165, 0)  # Orange
                
            else:
                print("✅ All systems normal")
                dog.rgb_strip.set_color(0, 255, 0)  # Green
                
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.rgb_strip.off()
        print("\nSensor fusion stopped")
```

### Autonomous Navigation

```python
def autonomous_navigation(dog):
    """Navigate autonomously using sensor data."""
    print("Autonomous navigation active... Press Ctrl+C to stop")
    
    try:
        while True:
            distance = dog.ultrasonic.read()
            pitch, roll, yaw = dog.imu.read()
            
            # Navigation logic
            if distance < 20:
                # Obstacle avoidance
                print("Avoiding obstacle...")
                dog.do_action('turn_left', angle=45)
                time.sleep(1)
            elif abs(pitch) > 15 or abs(roll) > 15:
                # Unstable surface
                print("Unstable surface - stopping")
                dog.do_action('sit')
                time.sleep(2)
            else:
                # Normal navigation
                print("Navigating forward...")
                dog.do_action('walk', steps=2)
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        dog.do_action('sit')
        print("\nAutonomous navigation stopped")
```

## 🔧 Sensor Configuration

### Calibration

```python
def calibrate_all_sensors(dog):
    """Calibrate all sensors."""
    print("Sensor Calibration")
    
    # IMU calibration
    print("\n1. IMU Calibration")
    print("Keep robot still on level surface...")
    dog.calibrate_imu()
    
    # Ultrasonic calibration (if needed)
    print("\n2. Ultrasonic Calibration")
    print("Place robot 50cm from wall...")
    input("Press Enter when ready...")
    
    distance = dog.ultrasonic.read()
    print(f"Measured distance: {distance} cm")
    
    # Touch sensor test
    print("\n3. Touch Sensor Test")
    print("Touch left sensor...")
    while not dog.dual_touch.read()[0]:
        time.sleep(0.1)
    print("Left touch detected!")
    
    print("Touch right sensor...")
    while not dog.dual_touch.read()[1]:
        time.sleep(0.1)
    print("Right touch detected!")
    
    print("\n✅ All sensors calibrated!")
```

### Performance Monitoring

```python
def sensor_performance(dog):
    """Monitor sensor performance."""
    print("Sensor Performance Monitor")
    print("=" * 50)
    
    # Test ultrasonic response time
    start_time = time.time()
    distance = dog.ultrasonic.read()
    ultrasonic_time = time.time() - start_time
    
    # Test IMU response time
    start_time = time.time()
    pitch, roll, yaw = dog.imu.read()
    imu_time = time.time() - start_time
    
    # Test touch response time
    start_time = time.time()
    left, right = dog.dual_touch.read()
    touch_time = time.time() - start_time
    
    print(f"Ultrasonic response: {ultrasonic_time*1000:.1f} ms")
    print(f"IMU response: {imu_time*1000:.1f} ms")
    print(f"Touch response: {touch_time*1000:.1f} ms")
    print(f"Distance: {distance} cm")
    print(f"Orientation: Pitch={pitch:.1f}°, Roll={roll:.1f}°, Yaw={yaw:.1f}°")
    print(f"Touch: Left={left}, Right={right}")
```

## 🐛 Troubleshooting

### Common Sensor Issues

**Ultrasonic not working:**
```python
# Check sensor connections
# Verify I2C is enabled
# Test with known distance
```

**IMU giving wrong readings:**
```python
# Recalibrate IMU
dog.calibrate_imu()

# Check sensor mounting
# Verify connections
```

**Touch sensors not responding:**
```python
# Check GPIO connections
# Verify power supply
# Clean sensor contacts
```

## 📚 Next Steps

- **[Movement Guide](movement.md)** - Using sensors for movement control
- **[AI Integration](ai-integration.md)** - AI-powered sensor processing
- **[Examples](../examples/sensors.md)** - More sensor examples
- **[API Reference](../api/sensors.md)** - Complete sensor API documentation 