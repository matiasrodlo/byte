# Troubleshooting Guide

Common issues and solutions for Byte robot problems.

## 🚨 Emergency Stop

If your robot is behaving unexpectedly:

```python
# Emergency stop - immediately stop all movements
dog.do_action('sit')  # This will stop all current movements
dog.close()           # Close all connections
```

## 🔧 Hardware Issues

### Servo Problems

**Problem: Servos not moving**
```bash
# Check power supply
sudo dmesg | grep -i power

# Check PWM connections
sudo i2cdetect -y 1

# Verify servo connections on RobotHat
```

**Solutions:**
- Ensure 5V power supply is connected
- Check servo wire connections
- Verify PWM pins are not damaged
- Calibrate servos: `dog.calibrate_servos()`

**Problem: Servos moving erratically**
```python
# Reduce servo speed and acceleration
dog = RobotDog(servo_speed=30, servo_accel=10)
```

**Problem: Servos making noise but not moving**
- Check for mechanical obstructions
- Verify servo torque is sufficient
- Check for stripped gears

### Sensor Issues

**Problem: Ultrasonic sensor not working**
```bash
# Check I2C bus
sudo i2cdetect -y 1

# Check sensor address (usually 0x57)
sudo i2cdetect -y 1 | grep 57
```

**Solutions:**
- Verify I2C is enabled: `sudo raspi-config nonint do_i2c 0`
- Check sensor connections
- Restart I2C service: `sudo systemctl restart i2c-dev`

**Problem: Touch sensors not responding**
```python
# Test touch sensors
left, right = dog.dual_touch.read()
print(f"Left: {left}, Right: {right}")
```

**Solutions:**
- Check GPIO connections
- Verify sensor power supply
- Clean sensor contacts

**Problem: IMU giving incorrect readings**
```python
# Calibrate IMU
dog.calibrate_imu()

# Check readings
pitch, roll, yaw = dog.imu.read()
print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
```

**Solutions:**
- Keep robot still during calibration
- Check IMU connections
- Verify sensor is not loose

### Camera Issues

**Problem: Camera not detected**
```bash
# Check camera interface
vcgencmd get_camera

# Enable camera
sudo raspi-config nonint do_camera 0
```

**Problem: Camera feed is black**
- Check camera cable connections
- Verify camera module is seated properly
- Check camera power supply

## 💻 Software Issues

### Import Errors

**Problem: `ModuleNotFoundError: No module named 'byte'`**
```bash
# Install package in development mode
cd /path/to/byte
pip install -e .

# Or install dependencies
pip install -r requirements.txt
```

**Problem: `ImportError: No module named 'robot_hat'`**
```bash
# Install robot_hat dependency
pip install robot_hat

# Or install all dependencies
pip install -r requirements.txt
```

### Permission Errors

**Problem: `PermissionError: [Errno 13] Permission denied`**
```bash
# Add user to required groups
sudo usermod -a -G gpio,i2c,spi $USER

# Reboot to apply changes
sudo reboot
```

**Problem: `OSError: [Errno 1] Operation not permitted`**
```bash
# Check if running as root (not recommended)
whoami

# Run with proper permissions
sudo python3 your_script.py
```

### Connection Errors

**Problem: `ConnectionError: Failed to connect to robot hardware`**
```python
# Check hardware connections
try:
    dog = RobotDog()
except ConnectionError as e:
    print(f"Hardware connection failed: {e}")
    print("Check power supply and connections")
```

**Solutions:**
- Verify RobotHat is properly connected
- Check power supply voltage
- Ensure all cables are seated properly

## 🎯 Movement Issues

### Gait Problems

**Problem: Robot falls over when walking**
```python
# Reduce movement speed
dog = RobotDog(servo_speed=30, servo_accel=10)

# Use slower gait
dog.do_action('walk', speed=30)
```

**Solutions:**
- Check surface is level and stable
- Verify all servos are calibrated
- Reduce movement speed
- Check for loose mechanical parts

**Problem: Robot moves in wrong direction**
```python
# Check servo mapping
# Servos 0-3: Front legs
# Servos 4-7: Back legs
# Servos 8-9: Head
# Servos 10-11: Tail
```

**Problem: Robot shakes or vibrates**
- Check for loose screws
- Verify servo mounting
- Reduce servo speed
- Check for mechanical interference

## 🔋 Power Issues

### Battery Problems

**Problem: Robot shuts down unexpectedly**
```python
# Check battery level (if supported)
level = dog.get_battery_level()
print(f"Battery: {level}%")
```

**Solutions:**
- Charge batteries fully
- Check battery connections
- Verify battery voltage
- Consider using external power supply

**Problem: Servos lose power during movement**
- Use higher capacity power supply
- Check power cable connections
- Verify power supply voltage (5V minimum)

## 🎨 LED Issues

**Problem: RGB LED not working**
```python
# Test LED functionality
dog.rgb_strip.set_color(255, 0, 0)  # Red
time.sleep(1)
dog.rgb_strip.set_color(0, 255, 0)  # Green
time.sleep(1)
dog.rgb_strip.set_color(0, 0, 255)  # Blue
```

**Solutions:**
- Check LED strip connections
- Verify PWM pin connections
- Check power supply to LED strip

## 🔍 Debugging Tools

### System Information

```bash
# Check Raspberry Pi model
cat /proc/device-tree/model

# Check Python version
python3 --version

# Check installed packages
pip list | grep byte
```

### Hardware Diagnostics

```python
from byte import RobotDog

def hardware_test():
    """Comprehensive hardware test."""
    try:
        dog = RobotDog()
        print("✅ RobotDog initialized successfully")
        
        # Test sensors
        distance = dog.ultrasonic.read()
        print(f"✅ Ultrasonic: {distance} cm")
        
        left, right = dog.dual_touch.read()
        print(f"✅ Touch sensors: Left={left}, Right={right}")
        
        pitch, roll, yaw = dog.imu.read()
        print(f"✅ IMU: Pitch={pitch}, Roll={roll}, Yaw={yaw}")
        
        # Test LED
        dog.rgb_strip.set_color(0, 255, 0)
        print("✅ RGB LED: Green")
        time.sleep(1)
        dog.rgb_strip.off()
        
        # Test basic movement
        dog.do_action('sit')
        print("✅ Movement: Sit")
        time.sleep(1)
        dog.do_action('stand')
        print("✅ Movement: Stand")
        
        print("\n🎉 All hardware tests passed!")
        
    except Exception as e:
        print(f"❌ Hardware test failed: {e}")
    finally:
        dog.close()

if __name__ == "__main__":
    hardware_test()
```

### Logging

```python
import logging

# Enable debug logging
logging.basicConfig(level=logging.DEBUG)

# Create robot with logging
dog = RobotDog()
```

## 📞 Getting Help

### Before Asking for Help

1. **Check this troubleshooting guide**
2. **Run hardware diagnostics**
3. **Check system logs**: `sudo dmesg | tail -20`
4. **Verify connections and power supply**
5. **Test with minimal code example**

### Information to Provide

When reporting issues, include:

- **Raspberry Pi model and OS version**
- **Python version**: `python3 --version`
- **Byte version**: `pip show byte`
- **Error messages and stack traces**
- **Hardware configuration**
- **Steps to reproduce the issue**

### Support Channels

- **GitHub Issues**: For bug reports and feature requests
- **Discussions**: For questions and community help
- **Documentation**: Check this guide and API reference
- **Examples**: Review working code in `samples/` directory

## 🔄 Common Solutions

### Quick Fixes

```bash
# Restart services
sudo systemctl restart i2c-dev
sudo systemctl restart bluetooth

# Clear Python cache
find . -name "*.pyc" -delete
find . -name "__pycache__" -delete

# Reinstall package
pip uninstall byte
pip install -e .
```

### Reset to Default

```python
# Reset robot to default state
dog.do_action('sit')
dog.rgb_strip.off()
dog.close()

# Reinitialize with default settings
dog = RobotDog()
```

Remember: **When in doubt, start with `dog.do_action('sit')` to stop all movements safely!** 