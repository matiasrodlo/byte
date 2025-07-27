# Compatibility

## Hardware Requirements

**Raspberry Pi:**
- Pi 4 (4GB/8GB) - Full support
- Pi 3B+ - Full support
- Pi Zero 2W - Limited (reduced servo speed)

**Components:**
- RobotHat v2.0+ expansion board
- 12× metal gear servos (MG996R, 5V, ~10kg-cm)
- 5MP camera (Raspberry Pi Camera v2)
- Ultrasonic sensor (HC-SR04 or I2C)
- Dual touch sensors (capacitive)
- IMU sensor (SH3001, I2C address 0x6F)
- RGB LED strip (WS2812B, 8-16 LEDs)
- 2× 18650 batteries (7.4V, 3000mAh each)

## Software Requirements

- Raspberry Pi OS (Debian-based)
- Python 3.7+
- Dependencies: robot_hat>=2.0.0, numpy>=1.19.0, readchar>=2.0.0, spidev>=3.5, gpiozero>=1.6.0, smbus2>=0.4.0

## Pin Configuration

Byte uses fixed GPIO pins:

**Servos (via RobotHat PWM):**
- Legs: PWM 0-7 (8 servos, 2 per leg)
- Head: PWM 4, 5, 6 (yaw, roll, pitch)
- Tail: PWM 9

**Sensors:**
- Ultrasonic: GPIO23 (TRIG), GPIO24 (ECHO)
- Touch: GPIO17 (left), GPIO27 (right)
- IMU: I2C (SDA=GPIO2, SCL=GPIO3)
- RGB LED: GPIO18 (PWM)
- Sound direction: SPI (MOSI, MISO, SCLK, CS)

## Physical Specifications

- Body: 117mm length × 98mm width
- Leg segment: 42mm
- Foot segment: 76mm
- Default height: 80mm (adjustable 20-95mm)

## Power Specifications

- Voltage: 7.4V (2× 18650 batteries)
- Current: ~500mA idle, ~1.5A walking, ~3A peak
- Runtime: ~1.5 hours typical usage

## Limitations

- Pi Zero 2W: Reduced performance with complex movements
- Battery runtime: ~1.5 hours
- Servo power: Requires stable 5V for all 12 servos
- Sound direction: 20° resolution (not continuous)
