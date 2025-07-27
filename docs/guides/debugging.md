# Debugging

## Logging

```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('byte')

from byte import RobotDog

dog = RobotDog()
logger.debug("Byte initialized")
logger.info(f"Body height: {dog.body_height}mm")
```

## Sensor Diagnostics

```python
from byte import RobotDog

dog = RobotDog()

# Test ultrasonic
distance = dog.ultrasonic.read()
print(f"Ultrasonic: {distance} cm")

# Test touch
left, right = dog.dual_touch.read()
print(f"Touch: left={left}, right={right}")

# Test IMU
pitch, roll, yaw = dog.imu.read()
print(f"IMU: pitch={pitch}°, roll={roll}°, yaw={yaw}°")

# Test sound direction
angle = dog.sound_direction.read()
print(f"Sound: {angle}°")

dog.close()
```

## Servo Diagnostics

```python
from byte import RobotDog

dog = RobotDog()

# Test each servo
for i in range(12):
    print(f"Testing servo {i}...")
    # Move to center position
    dog.set_servo_angle(i, 90)
    time.sleep(0.5)

dog.close()
```

## Performance Monitoring

```python
import time
from byte import RobotDog

dog = RobotDog()

# Measure action execution time
start = time.time()
dog.do_action('walk', steps=10)
duration = time.time() - start
print(f"Walk 10 steps took {duration:.2f}s")

# Monitor sensor update rate
start = time.time()
for _ in range(100):
    dog.ultrasonic.read()
duration = time.time() - start
print(f"Ultrasonic rate: {100/duration:.1f} Hz")

dog.close()
```

## Hardware Checks

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check GPIO state
gpio readall

# Check camera
vcgencmd get_camera

# Check power
vcgencmd measure_volts
```

## Common Issues

**Servos not responding**: Check PWM signals, verify power supply

**Sensors returning None**: Check I2C/SPI enabled, verify connections

**Slow performance**: Check CPU usage, reduce servo speed, optimize code

See [Troubleshooting Guide](troubleshooting.md) for more.
