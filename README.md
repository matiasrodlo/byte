# Byte

Raspberry Pi quadruped robot with 12 servos, sensors, and AI capabilities.

<img width="1200" height="801" alt="image" src="https://github.com/user-attachments/assets/a75ad140-43c3-42d9-b60d-fe0a085a250a" />

## Installation

```bash
git clone https://github.com/matiasrodlo/byte.git
cd byte
pip install -e .
```

## Usage

```python
from byte import RobotDog

dog = RobotDog()
dog.do_action('sit')
dog.do_action('walk', steps=5)
dog.close()
```

## Hardware

- Raspberry Pi 4 (recommended) or Pi 3B+
- RobotHat expansion board v2.0+
- 12× metal gear servos (MG996R)
- 5MP camera module
- Ultrasonic sensor (HC-SR04)
- Dual touch sensors
- IMU sensor (SH3001)
- RGB LED strip (WS2812B)
- 2× 18650 batteries

## Architecture

Byte uses **inverse kinematics** for leg positioning based on a 2-DOF planar mechanism. The kinematic model uses the law of cosines to solve for joint angles given desired foot positions.

### Physical Specifications

**Body dimensions:**
- Length: 117.0 mm (±0.5 mm)
- Width: 98.0 mm (±0.5 mm)
- Default height: 80.0 mm (adjustable 20-95 mm)

**Leg kinematics:**
- Leg segment (L₁): 42.0 mm (thigh)
- Foot segment (L₂): 76.0 mm (shank)
- Maximum reach: 118.0 mm (L₁ + L₂)
- Workspace: Circular annulus, r ∈ [34, 118] mm

**Coordinate system:**
- Body-fixed frame: X (forward), Y (left), Z (up)
- Leg frame: Y (forward/backward), Z (vertical)

### Servo Configuration

**Servo layout:**
- Legs: 8 servos (2 per leg) on PWM 0-11
  - Front Left: PWM 2, 3 (hip, thigh)
  - Front Right: PWM 7, 8 (hip, thigh)
  - Rear Left: PWM 0, 1 (hip, thigh)
  - Rear Right: PWM 10, 11 (hip, thigh)
- Head: 3 servos (yaw, roll, pitch) on PWM 4, 5, 6
- Tail: 1 servo on PWM 9

**Servo specifications:**
- Model: MG996R metal gear servos
- Torque: 10 kg·cm @ 5.0V
- Speed: 0.17 s/60° @ 5.0V
- Range: 0-180° mechanical

### Sensor Configuration

**GPIO pin assignments:**
- Ultrasonic (HC-SR04): GPIO23 (TRIG), GPIO24 (ECHO)
  - Range: 2-400 cm, Accuracy: ±3 mm
- Touch sensors: GPIO17 (left), GPIO27 (right)
  - Type: Capacitive digital
- IMU (SH3001): I2C bus (SDA=GPIO2, SCL=GPIO3, Address: 0x6F)
  - Accelerometer: ±2g to ±16g selectable
  - Gyroscope: ±250 to ±2000 dps selectable
- RGB LED (WS2812B): GPIO18 (PWM)
  - Protocol: One-wire, 8-16 LEDs configurable

### Control System

**PID controller:**
- Proportional gain (Kₚ): 0.033
- Integral gain (Kᵢ): 0.0 (disabled)
- Derivative gain (Kd): 0.0 (disabled)
- Control law: `u(t) = Kₚ·e(t)` (proportional-only)

**Gait parameters:**
- Walking: 8 sections/cycle, duty factor 0.75, 3-point support
- Trotting: 2 sections/cycle, duty factor 0.5, 2-point support

## Documentation

### User Guides
- [Installation](docs/installation/installation.md)
- [Quick Start](docs/guides/quickstart.md)
- [API Reference](docs/api/robotdog.md)
- [Examples](docs/examples/basic.md)
- [FAQ](docs/guides/faq.md)

### Technical Documentation
- [Technical Specifications](docs/technical/specifications.md) - Complete engineering specifications
- [Kinematics Theory](docs/technical/kinematics.md) - Inverse kinematics formulation and derivations
- [Control Theory](docs/technical/control.md) - PID control and gait generation

## License

MIT License
