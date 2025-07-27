# FAQ

## Installation

**Q: How do I install Byte?**
A: `git clone https://github.com/matiasrodlo/byte.git && cd byte && pip install -e .`

**Q: What Python version?**
A: Python 3.7+. Dependencies: robot_hat>=2.0.0, numpy>=1.19.0, readchar>=2.0.0, spidev>=3.5, gpiozero>=1.6.0, smbus2>=0.4.0

**Q: What Raspberry Pi models work?**
A: Pi 4 (recommended), Pi 3B+, Pi Zero 2W (limited). See [Compatibility](installation/compatibility.md).

## Hardware

**Q: How are servos connected?**
A: 12 servos via RobotHat PWM ports 0-11. Legs: 0-7 (2 per leg), Head: 4,5,6 (yaw,roll,pitch), Tail: 9.

**Q: How are sensors connected?**
A: Ultrasonic: GPIO23/24, Touch: GPIO17/27, IMU: I2C (SH3001 at 0x6F), RGB LED: GPIO18, Sound direction: SPI.

**Q: Power requirements?**
A: 2× 18650 batteries (7.4V). Servos need 5V, sensors need 3.3V. Current: ~500mA idle, ~1.5A walking, ~3A peak.

## Usage

**Q: How do actions work?**
A: `do_action()` loads predefined servo angles from `ActionDict` and moves servos. Actions like 'sit', 'walk', 'bark' are stored as angle sequences.

**Q: How do I read sensors?**
A: Use sensor objects: `dog.ultrasonic.read()` returns float (cm), `dog.dual_touch.read()` returns (bool, bool), `dog.imu.read()` returns (pitch, roll, yaw) degrees.

**Q: How does movement work?**
A: Byte uses inverse kinematics. Body pose [x,y,z] and orientation [roll,pitch,yaw] are converted to leg joint angles. See [Movement Guide](movement.md).

**Q: Can I control servos directly?**
A: Yes: `dog.legs_move(angles)`, `dog.head_move(yaw, roll, pitch)`, `dog.tail_move(angle)`. Or use `dog.move_leg(id, x, y, z)` for IK.

## Troubleshooting

**Q: Servos not moving?**
A: Check 5V power to RobotHat, verify PWM connections, run `dog.calibrate_servos()`. See [Troubleshooting](troubleshooting.md).

**Q: Sensors not working?**
A: Enable I2C/SPI: `sudo raspi-config nonint do_i2c 0 && sudo raspi-config nonint do_spi 0`. Check connections match pin layout.

**Q: Import errors?**
A: Ensure Byte installed: `pip install -e .` from byte directory. Check Python 3.7+.

**Q: Camera not working?**
A: Enable camera: `sudo raspi-config nonint do_camera 0`. Check CSI cable. Reboot.

**Q: Audio not working?**
A: Run `sudo ./bin/i2samp.sh` to configure I2S audio drivers.

## Technical

**Q: How does inverse kinematics work?**
A: Byte calculates leg joint angles from foot position [x,y,z] relative to body. Uses body dimensions (117×98mm) and leg segments (42mm leg, 76mm foot).

**Q: How does gait generation work?**
A: Gaits like 'walk' and 'trot' cycle through leg positions over time. Each step moves legs in sequence to create forward motion.

**Q: How does sound direction work?**
A: SPI interface with TR16F064B chip. Detects 360° with 20° resolution. Master sends 16-bit command, receives 16-bit response with angle.

**Q: What's the servo speed?**
A: Head: 300°/s, Legs: 428°/s, Tail: 500°/s. Configurable via `servo_speed` parameter.

## More Help

- [Troubleshooting Guide](troubleshooting.md)
- [API Reference](../api/robotdog.md)
- [GitHub Issues](https://github.com/matiasrodlo/byte/issues)
