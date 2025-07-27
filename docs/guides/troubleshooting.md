# Troubleshooting

## Emergency Stop

```python
dog.do_action('sit')  # Stops all movements
dog.close()            # Closes connections
```

## Servo Problems

**Servos not moving:**
- Check 5V power to RobotHat
- Verify PWM connections (ports 0-11)
- Run calibration: `dog.calibrate_servos()`
- Check servo wire connections

**Servos moving erratically:**
```python
# Reduce speed
dog = RobotDog(servo_speed=30, servo_accel=10)
```

**Servos making noise but not moving:**
- Check mechanical obstructions
- Verify servo torque (need ~10kg-cm)
- Check for stripped gears

## Sensor Issues

**Ultrasonic not working:**
```bash
# Check I2C
sudo i2cdetect -y 1

# Verify GPIO connections
# GPIO23 = TRIG, GPIO24 = ECHO
```

**Touch sensors not responding:**
```python
# Test sensors
left, right = dog.dual_touch.read()
print(f"Left: {left}, Right: {right}")

# Check GPIO17/27 connections
# Verify 3.3V power
```

**IMU wrong readings:**
```python
# Calibrate on level surface
dog.calibrate_imu()

# Check I2C address (should be 0x6F)
sudo i2cdetect -y 1 | grep 6F
```

**Sound direction not working:**
- Check SPI interface enabled: `sudo raspi-config nonint do_spi 0`
- Verify BUSY line connection
- Check TR16F064B chip connections

## Software Issues

**Import errors:**
```bash
# Ensure Byte installed
pip install -e .

# Check Python version (needs 3.7+)
python3 --version
```

**Permission errors:**
```bash
# Add user to GPIO groups
sudo usermod -a -G gpio,i2c,spi $USER
sudo reboot
```

**Camera not working:**
```bash
# Enable camera
sudo raspi-config nonint do_camera 0

# Check CSI cable connection
# Reboot after enabling
```

**Audio not working:**
```bash
# Run audio configuration
sudo ./bin/i2samp.sh

# Check I2S interface
```

## Movement Issues

**Byte falls over:**
- Reduce servo speed: `RobotDog(servo_speed=30)`
- Check surface is level
- Calibrate IMU: `dog.calibrate_imu()`
- Verify all servos connected

**Uneven walking:**
- Calibrate servos: `dog.calibrate_servos()`
- Check individual leg positions
- Verify servo angles are correct

**Slow response:**
- Increase servo speed: `dog.servo_speed = 75`
- Check for mechanical obstructions
- Verify power supply adequate

## Power Issues

**Battery drains quickly:**
- Normal: ~1.5 hours runtime
- Check battery capacity (should be 3000mAh each)
- Reduce servo speed to save power
- Disable unused sensors

**Servos lose power:**
- Check 5V power supply
- Verify battery voltage (should be 7.4V)
- Check RobotHat power distribution

## More Help

- [FAQ](faq.md) - Common questions
- [Debugging Guide](debugging.md) - Diagnostic tools
- [GitHub Issues](https://github.com/matiasrodlo/byte/issues) - Report bugs
