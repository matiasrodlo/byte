# Hardware Setup

## Components

- Raspberry Pi 4
- RobotHat expansion board v2.0+
- 12× metal gear servos (MG996R, 5V, ~10kg-cm)
- 5MP camera module (Raspberry Pi Camera v2)
- Ultrasonic sensor (HC-SR04)
- Dual touch sensors (capacitive)
- IMU sensor (SH3001, I2C address 0x6F)
- RGB LED strip (WS2812B, 8-16 LEDs)
- 2× 18650 batteries (7.4V, 3000mAh each)

## Servo Connections

Byte uses 12 servos connected to RobotHat PWM ports:

```
Leg Servos (8 servos):
- Front Left:  PWM 2, 3
- Front Right: PWM 7, 8
- Back Left:    PWM 0, 1
- Back Right:   PWM 10, 11

Head Servos (3 servos):
- Yaw:   PWM 4
- Roll:  PWM 6
- Pitch: PWM 5

Tail Servo (1 servo):
- PWM 9
```

## Sensor Connections

```
Ultrasonic (HC-SR04):
- VCC → 5V
- GND → GND
- TRIG → GPIO23
- ECHO → GPIO24

Touch Sensors:
- Left:  VCC→3.3V, GND→GND, OUT→GPIO17
- Right: VCC→3.3V, GND→GND, OUT→GPIO27

IMU (SH3001):
- VCC → 3.3V
- GND → GND
- SDA → GPIO2 (I2C1)
- SCL → GPIO3 (I2C1)
- Address: 0x6F

RGB LED (WS2812B):
- VCC → 5V
- GND → GND
- DIN → GPIO18 (PWM)

Sound Direction:
- SPI interface (MOSI, MISO, SCLK, CS)
- BUSY line for detection control
```

## Power System

```
Battery Input:
- 2× 18650 batteries (7.4V nominal)
- Connected to RobotHat power input

RobotHat Power Distribution:
- Input: 7.4V from batteries
- Output: 5V for servos (via regulator)
- Output: 3.3V for sensors and logic

Current Requirements:
- Idle: ~500mA
- Walking: ~1.5A
- Peak (all servos): ~3A
```

## Physical Dimensions

Byte body dimensions:
- Length: 117mm
- Width: 98mm
- Default height: 80mm (adjustable 20-95mm)
- Leg segment: 42mm
- Foot segment: 76mm

## Assembly Notes

1. Mount RobotHat on Raspberry Pi GPIO header
2. Connect servos to PWM ports 0-11
3. Connect sensors to specified GPIO pins
4. Route cables to avoid interference
5. Secure battery holder to body
6. Test connections before final assembly

## Testing Connections

```python
from byte import RobotDog

dog = RobotDog()

# Test servos
dog.do_action('sit')
dog.do_action('stand')

# Test sensors
distance = dog.ultrasonic.read()
left, right = dog.dual_touch.read()
pitch, roll, yaw = dog.imu.read()

# Test LED
dog.rgb_strip.set_color(0, 255, 0)

dog.close()
```

## Calibration

After assembly, calibrate servos:

```python
from byte import RobotDog

dog = RobotDog()
dog.calibrate_servos()
dog.calibrate_imu()
dog.close()
```

See [Installation Guide](installation.md) for software setup.
