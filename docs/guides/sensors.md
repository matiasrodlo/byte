# Sensors

## Sensor Overview

Byte has 4 sensor types connected via specific protocols:

- **Ultrasonic**: HC-SR04 on GPIO23/24, returns distance in cm
- **Dual Touch**: Capacitive sensors on GPIO17/27, returns (left, right) booleans
- **IMU**: SH3001 on I2C (address 0x6F), returns (pitch, roll, yaw) in degrees
- **Sound Direction**: SPI interface with TR16F064B, returns angle 0-360° (20° resolution)

## Ultrasonic Sensor

HC-SR04 protocol: Send trigger pulse on GPIO23, measure echo on GPIO24.

```python
from byte import RobotDog

dog = RobotDog()

# Read distance (returns float in cm)
distance = dog.ultrasonic.read()
print(f"Distance: {distance} cm")

dog.close()
```

**Protocol details:**
- Trigger: 10μs pulse on GPIO23
- Echo: Measure pulse width on GPIO24
- Distance = (pulse_width × speed_of_sound) / 2
- Range: 2-400cm

## Touch Sensors

Capacitive sensors on GPIO17 (left) and GPIO27 (right). Read as digital inputs.

```python
from byte import RobotDog

dog = RobotDog()

# Read touch state (returns tuple of booleans)
left, right = dog.dual_touch.read()
print(f"Touch: left={left}, right={right}")

dog.close()
```

**How it works:**
- Sensors detect capacitance changes
- GPIO reads HIGH when touched
- Debouncing handled in software

## IMU Sensor (SH3001)

6-axis IMU on I2C bus. Accelerometer and gyroscope data fused for orientation.

```python
from byte import RobotDog

dog = RobotDog()

# Read orientation (returns tuple in degrees)
pitch, roll, yaw = dog.imu.read()
print(f"Pitch: {pitch}°, Roll: {roll}°, Yaw: {yaw}°")

# Calibrate (keep robot still on level surface)
dog.calibrate_imu()

dog.close()
```

**I2C details:**
- Address: 0x6F
- Bus: I2C1 (SDA=GPIO2, SCL=GPIO3)
- Update rate: ~100Hz
- Range: ±2000°/s (gyro), ±16g (accel)

## Sound Direction Sensor

SPI interface with TR16F064B chip. Detects sound direction in 360° range.

```python
from byte import RobotDog

dog = RobotDog()

# Read sound direction (returns angle 0-360° or None)
angle = dog.sound_direction.read()
if angle is not None:
    print(f"Sound at {angle}°")

dog.close()
```

**SPI protocol:**
- Master sends 16-bit command
- Slave responds with 16-bit angle data
- Resolution: 20° (18 positions)
- BUSY line controls detection cycle

## Sensor Integration Example

```python
from byte import RobotDog
import time

dog = RobotDog()

try:
    while True:
        # Read all sensors
        distance = dog.ultrasonic.read()
        left, right = dog.dual_touch.read()
        pitch, roll, yaw = dog.imu.read()
        sound_angle = dog.sound_direction.read()
        
        # Decision logic
        if distance < 20:
            dog.do_action('sit')
        elif left or right:
            if left:
                dog.do_action('turn_left')
            else:
                dog.do_action('turn_right')
        elif sound_angle is not None:
            # Turn towards sound
            if sound_angle > 180:
                dog.do_action('turn_left', angle=45)
            else:
                dog.do_action('turn_right', angle=45)
        else:
            dog.do_action('walk', steps=1)
        
        time.sleep(0.5)
except KeyboardInterrupt:
    dog.do_action('sit')
    dog.close()
```

## Troubleshooting

**Ultrasonic not working**: Check GPIO23/24 connections, verify trigger/echo wiring

**Touch sensors not responding**: Check GPIO17/27, verify 3.3V power, clean contacts

**IMU wrong readings**: Calibrate on level surface, check I2C bus: `sudo i2cdetect -y 1`

**Sound direction not working**: Check SPI interface, verify BUSY line connection

See [Troubleshooting Guide](troubleshooting.md) for details.
