# Installation Guide

This guide will walk you through installing Byte on your Raspberry Pi.

## 📋 Prerequisites

### Hardware Requirements
- **Raspberry Pi 4** (recommended) or Pi 3B+
- **RobotHat** expansion board
- **12× Metal gear servos** (legs, head, tail)
- **5MP Camera module**
- **Ultrasonic sensor**
- **Dual touch sensors**
- **IMU sensor (SH3001)**
- **RGB LED strip**
- **2× 18650 batteries** with holder
- **MicroSD card** (32GB+ recommended)

### Software Requirements
- **Raspberry Pi OS** (Debian-based)
- **Python 3.7+**
- **Git**

## 🚀 Installation Steps

### 1. Prepare Raspberry Pi

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y python3-pip python3-venv git

# Enable I2C and SPI
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
```

### 2. Clone Repository

```bash
# Clone the repository
git clone https://github.com/matiasrodlo/byte.git
cd byte

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

```bash
# Install Python dependencies
pip install -r requirements.txt

# Or install the package directly
pip install -e .
```

### 4. Configure Audio (Optional)

```bash
# Run audio configuration script
sudo ./i2samp.sh
```

### 5. Install System Service (Optional)

```bash
# Install system service for auto-start
sudo ./bin/robotdog_app_install.sh
```

## 🔧 Hardware Assembly

### Servo Connections
Connect servos to RobotHat according to this mapping:

| Servo | Function | Pin |
|-------|----------|-----|
| 1-4   | Front legs | PWM1-4 |
| 5-8   | Back legs | PWM5-8 |
| 9-10  | Head | PWM9-10 |
| 11-12 | Tail | PWM11-12 |

### Sensor Connections
- **Ultrasonic**: I2C bus
- **Touch sensors**: GPIO pins
- **IMU**: I2C bus
- **Camera**: CSI port
- **RGB LED**: PWM pin

## ✅ Verification

### Test Basic Installation

```python
from byte import RobotDog

# Initialize robot
dog = RobotDog()

# Test basic functionality
print("Robot initialized successfully!")
print(f"Version: {dog.__version__}")

# Test sensors
distance = dog.ultrasonic.read()
print(f"Distance: {distance} cm")

# Cleanup
dog.close()
```

### Test Movement

```python
from byte import RobotDog

dog = RobotDog()

# Test basic movements
dog.do_action('sit')
dog.do_action('stand')
dog.do_action('bark')

dog.close()
```

## 🐛 Troubleshooting

### Common Issues

**Import Error: No module named 'byte'**
```bash
# Make sure you're in the correct directory
cd /path/to/byte
pip install -e .
```

**Permission Denied**
```bash
# Add user to required groups
sudo usermod -a -G gpio,i2c,spi $USER
# Reboot required
sudo reboot
```

**Servo Not Moving**
- Check power supply (servos need 5V)
- Verify PWM connections
- Check servo calibration

**Sensors Not Working**
- Verify I2C is enabled: `sudo i2cdetect -y 1`
- Check sensor connections
- Verify sensor addresses

## 📚 Next Steps

After successful installation:

1. **[Quick Start Guide](../guides/quickstart.md)** - Learn basic operations
2. **[Hardware Setup](hardware.md)** - Detailed hardware configuration
3. **[Basic Usage](../guides/basic-usage.md)** - Fundamental robot control
4. **[Examples](../examples/basic.md)** - Working code examples

## 🆘 Getting Help

- **GitHub Issues**: Report installation problems
- **Hardware Issues**: Check connections and power supply
- **Software Issues**: Verify Python version and dependencies
- **Community**: Ask questions in discussions 