# Installation

## Prerequisites

- Raspberry Pi 4 (recommended) or Pi 3B+
- Raspberry Pi OS
- Python 3.7+
- RobotHat expansion board

## Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y python3-pip python3-venv git

# Enable I2C and SPI
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
```

## Install Byte

```bash
# Clone repository
git clone https://github.com/matiasrodlo/byte.git
cd byte

# Install package
pip install -e .
```

## Optional Configuration

```bash
# Configure audio
sudo ./bin/i2samp.sh

# Install system service
sudo ./bin/robotdog_app_install.sh
```

## Verify Installation

```python
from byte import RobotDog

dog = RobotDog()
print("Installation successful!")
dog.close()
```

## Troubleshooting

**Permission errors**: Add user to GPIO groups
```bash
sudo usermod -a -G gpio,i2c,spi $USER
sudo reboot
```

**Import errors**: Ensure package is installed
```bash
pip install -e .
```

For more help, see [Troubleshooting Guide](../guides/troubleshooting.md).
