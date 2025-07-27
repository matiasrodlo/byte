# Hardware Setup Guide

Complete hardware assembly and connection guide for Byte robot.

## 📋 Parts List

### Essential Components
- **Raspberry Pi 4** (4GB or 8GB recommended)
- **RobotHat** expansion board
- **12× Metal gear servos** (MG996R or equivalent)
- **5MP Camera module** (Raspberry Pi Camera v2)
- **Ultrasonic sensor** (HC-SR04 or I2C version)
- **Dual touch sensors** (capacitive touch modules)
- **IMU sensor** (SH3001 6-axis)
- **RGB LED strip** (WS2812B, 8-16 LEDs)
- **2× 18650 batteries** with holder
- **MicroSD card** (32GB+ Class 10)

### Mechanical Parts
- **Aluminum alloy body frame**
- **Leg assemblies** (4×)
- **Head assembly**
- **Tail assembly**
- **Screws and nuts** (M3×6, M3×8, M3×10)
- **Standoffs** (M3×10, M3×15)
- **Cable ties and organizers**

### Tools Required
- **Phillips screwdriver** (PH1, PH2)
- **Hex wrench set** (2mm, 2.5mm, 3mm)
- **Wire strippers**
- **Soldering iron** (optional)
- **Multimeter** (for voltage testing)

## 🔧 Assembly Steps

### 1. Prepare RobotHat

```bash
# Mount RobotHat on Raspberry Pi
# Ensure proper alignment with GPIO pins
# Secure with standoffs
```

**Connections:**
- Align RobotHat with Raspberry Pi GPIO header
- Press down firmly until seated
- Secure with M3×10 standoffs
- Verify no bent pins

### 2. Servo Installation

#### Leg Servos (8×)
```
Front Left Leg:
- Servo 0: Hip joint
- Servo 1: Thigh joint
- Servo 2: Calf joint

Front Right Leg:
- Servo 3: Hip joint
- Servo 4: Thigh joint
- Servo 5: Calf joint

Back Left Leg:
- Servo 6: Hip joint
- Servo 7: Thigh joint
- Servo 8: Calf joint

Back Right Leg:
- Servo 9: Hip joint
- Servo 10: Thigh joint
- Servo 11: Calf joint
```

#### Head Servos (2×)
```
Head Assembly:
- Servo 12: Pan (left/right)
- Servo 13: Tilt (up/down)
```

#### Tail Servos (2×)
```
Tail Assembly:
- Servo 14: Wag (left/right)
- Servo 15: Lift (up/down)
```

### 3. Servo Wiring

#### PWM Connections
Connect servos to RobotHat PWM ports:

| Servo | Function | PWM Port | Wire Color |
|-------|----------|----------|------------|
| 0-3   | Front legs | PWM1-4 | Red/Black/White |
| 4-7   | Back legs | PWM5-8 | Red/Black/White |
| 8-9   | Head | PWM9-10 | Red/Black/White |
| 10-11 | Tail | PWM11-12 | Red/Black/White |

**Wire Colors:**
- **Red**: Power (5V)
- **Black**: Ground (GND)
- **White**: Signal (PWM)

### 4. Sensor Installation

#### Ultrasonic Sensor
```
Location: Front of robot head
Connections:
- VCC → 5V
- GND → GND
- TRIG → GPIO23
- ECHO → GPIO24
```

#### Touch Sensors
```
Left Touch Sensor:
- VCC → 3.3V
- GND → GND
- OUT → GPIO17

Right Touch Sensor:
- VCC → 3.3V
- GND → GND
- OUT → GPIO27
```

#### IMU Sensor (SH3001)
```
Location: Center of robot body
Connections:
- VCC → 3.3V
- GND → GND
- SCL → GPIO3 (I2C1)
- SDA → GPIO2 (I2C1)
```

#### RGB LED Strip
```
Location: Top of robot body
Connections:
- VCC → 5V
- GND → GND
- DIN → GPIO18 (PWM)
```

### 5. Camera Installation

```
Camera Module:
- Connect CSI cable to Raspberry Pi
- Mount camera in head assembly
- Secure with provided screws
- Route cable through body
```

### 6. Power System

#### Battery Installation
```
Battery Holder:
- Insert 2× 18650 batteries
- Connect to RobotHat power input
- Secure battery holder to body
- Route power cables neatly
```

#### Power Distribution
```
RobotHat Power:
- Input: 7.4V from batteries
- Output: 5V for servos and sensors
- Output: 3.3V for logic circuits
```

## 🔌 Connection Diagram

```
Raspberry Pi 4
    ↓
RobotHat
    ↓
┌─────────────────────────────────────┐
│ PWM1-4:  Front Leg Servos (0-3)    │
│ PWM5-8:  Back Leg Servos (4-7)     │
│ PWM9-10: Head Servos (8-9)         │
│ PWM11-12: Tail Servos (10-11)      │
│ GPIO17:  Left Touch Sensor         │
│ GPIO23:  Ultrasonic TRIG           │
│ GPIO24:  Ultrasonic ECHO           │
│ GPIO27:  Right Touch Sensor        │
│ I2C1:    IMU Sensor (SH3001)       │
│ PWM18:   RGB LED Strip             │
│ CSI:     Camera Module             │
└─────────────────────────────────────┘
```

## ⚡ Power Requirements

### Voltage Levels
- **Servos**: 5V ±0.5V
- **Sensors**: 3.3V ±0.3V
- **Logic**: 3.3V ±0.3V
- **LED Strip**: 5V ±0.5V

### Current Requirements
- **Idle**: ~500mA
- **Walking**: ~1.5A
- **Peak (all servos)**: ~3A

### Battery Specifications
- **Type**: 2× 18650 Li-ion
- **Voltage**: 7.4V nominal
- **Capacity**: 3000mAh each
- **Runtime**: ~1.5 hours typical

## 🔍 Testing Connections

### Pre-Power Test
```bash
# Check continuity
multimeter continuity test:
- Power rails
- Ground connections
- Signal lines
```

### Post-Power Test
```python
from byte import RobotDog

def connection_test():
    """Test all hardware connections."""
    try:
        dog = RobotDog()
        print("✅ RobotHat connection: OK")
        
        # Test servos
        dog.do_action('sit')
        print("✅ Servo connections: OK")
        
        # Test sensors
        distance = dog.ultrasonic.read()
        print(f"✅ Ultrasonic: {distance} cm")
        
        left, right = dog.dual_touch.read()
        print(f"✅ Touch sensors: {left}, {right}")
        
        pitch, roll, yaw = dog.imu.read()
        print(f"✅ IMU: {pitch}, {roll}, {yaw}")
        
        # Test LED
        dog.rgb_strip.set_color(0, 255, 0)
        print("✅ RGB LED: OK")
        
        print("\n🎉 All connections verified!")
        
    except Exception as e:
        print(f"❌ Connection test failed: {e}")
    finally:
        dog.close()

if __name__ == "__main__":
    connection_test()
```

## 🛠️ Calibration

### Servo Calibration
```python
# Run after assembly
dog.calibrate_servos()

# Manual calibration if needed
for i in range(16):
    dog.set_servo_angle(i, 90)  # Center position
```

### IMU Calibration
```python
# Keep robot still on level surface
dog.calibrate_imu()

# Verify calibration
pitch, roll, yaw = dog.imu.read()
print(f"Calibrated values: Pitch={pitch}, Roll={roll}, Yaw={yaw}")
```

## 🔧 Maintenance

### Regular Checks
- **Tighten screws** monthly
- **Check servo connections** weekly
- **Clean sensors** as needed
- **Inspect cables** for wear

### Troubleshooting
- **Loose connections**: Re-seat and secure
- **Damaged cables**: Replace immediately
- **Servo issues**: Check power and connections
- **Sensor problems**: Verify voltage levels

## 📚 Next Steps

After hardware assembly:

1. **[Installation Guide](installation.md)** - Software setup
2. **[Quick Start Guide](../guides/quickstart.md)** - First run
3. **[Calibration Guide](../guides/calibration.md)** - Fine-tuning
4. **[Testing Guide](../examples/tests.md)** - Hardware verification

## ⚠️ Safety Notes

- **Disconnect power** before making connections
- **Check polarity** on all connections
- **Use proper tools** to avoid damage
- **Test incrementally** to isolate issues
- **Keep workspace clean** and organized 