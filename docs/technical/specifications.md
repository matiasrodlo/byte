# Technical Specifications

## Physical Dimensions

### Body Structure

| Parameter | Value | Unit | Tolerance | Notes |
|-----------|-------|------|-----------|-------|
| Body Length | 117.0 | mm | ±0.5 | Distance between front and rear leg attachment points |
| Body Width | 98.0 | mm | ±0.5 | Distance between left and right leg attachment points |
| Default Height | 80.0 | mm | ±2.0 | Vertical distance from ground to body center |
| Height Range | 20-95 | mm | - | Adjustable operating range |

### Leg Kinematics

| Parameter | Symbol | Value | Unit | Tolerance | Notes |
|-----------|--------|-------|------|-----------|-------|
| Leg Segment Length | L₁ | 42.0 | mm | ±0.3 | Upper leg (thigh) segment |
| Foot Segment Length | L₂ | 76.0 | mm | ±0.3 | Lower leg (shank) segment |
| Total Leg Reach | R_max | 118.0 | mm | - | Maximum reach: L₁ + L₂ |
| Leg Workspace Radius | R_ws | 95.0 | mm | - | Practical working radius |

### Coordinate System

**Body-Fixed Frame (B):**
- Origin: Body center of mass
- X-axis: Forward direction (positive forward)
- Y-axis: Lateral direction (positive left)
- Z-axis: Vertical direction (positive upward)

**Leg Frame (L):**
- Origin: Leg attachment point on body
- Y-axis: Forward/backward in leg plane
- Z-axis: Vertical in leg plane

## Servo Specifications

### MG996R Servo Parameters

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Operating Voltage | 4.8-7.2 | V | Nominal: 5.0V |
| Stall Torque | 10.0 | kg·cm | At 5.0V, 0° |
| No-Load Speed | 0.17 | s/60° | At 5.0V |
| Operating Angle | 0-180 | deg | Mechanical range |
| Control Signal | 0.5-2.5 | ms | PWM pulse width (1.5ms = 90°) |
| PWM Frequency | 50 | Hz | Standard servo frequency |
| Resolution | ~0.1 | deg | Effective resolution |
| Backlash | <2.0 | deg | Typical mechanical play |

### Servo Layout

| Component | Servos | PWM Ports | Function |
|-----------|--------|-----------|----------|
| Front Left Leg | 2 | 2, 3 | Hip, Thigh |
| Front Right Leg | 2 | 7, 8 | Hip, Thigh |
| Rear Left Leg | 2 | 0, 1 | Hip, Thigh |
| Rear Right Leg | 2 | 10, 11 | Hip, Thigh |
| Head | 3 | 4, 5, 6 | Yaw, Pitch, Roll |
| Tail | 1 | 9 | Wag |

**Total:** 12 servos

### Servo Speed Limits

| Component | Max Speed | Unit | Notes |
|-----------|-----------|------|-------|
| Leg Servos | 428 | deg/s | Maximum safe speed for leg servos |
| Head Servos | 300 | deg/s | Maximum safe speed for head servos |
| Tail Servo | 500 | deg/s | Maximum safe speed for tail servo |

## Sensor Specifications

### IMU (SH3001)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| I2C Address | 0x6F | - | 7-bit address |
| Accelerometer Range | ±2g, ±4g, ±8g, ±16g | g | Selectable |
| Gyroscope Range | ±250, ±500, ±1000, ±2000 | dps | Selectable |
| Resolution (2g range) | 16384 | LSB/g | 1g = 16384 counts |
| Resolution (4g range) | 8192 | LSB/g | 1g = 8192 counts |
| Sampling Rate | 100-1000 | Hz | Configurable |
| Noise Density (Accel) | 0.4 | mg/√Hz | Typical |
| Noise Density (Gyro) | 0.03 | dps/√Hz | Typical |

### Ultrasonic Sensor (HC-SR04)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Operating Voltage | 5.0 | V | DC |
| Detection Range | 2-400 | cm | Effective range |
| Accuracy | ±3 | mm | At 20°C |
| Beam Angle | 15 | deg | -3dB beamwidth |
| Response Time | <100 | ms | Typical |
| Resolution | 1 | mm | Limited by timing |

### Touch Sensors

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Type | Capacitive | - | Digital output |
| Operating Voltage | 3.3 | V | Logic level |
| Response Time | <10 | ms | Typical |
| GPIO Pins | 17, 27 | - | Left, Right |

### RGB LED Strip (WS2812B)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| LED Count | 8-16 | - | Configurable |
| Control Protocol | WS2812B | - | One-wire protocol |
| PWM Frequency | 800 | kHz | Data transmission |
| Color Depth | 24-bit | bit | 8-bit per channel |
| GPIO Pin | 18 | - | PWM-capable pin |

### Camera Module

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Resolution | 5.0 | MP | 2592×1944 max |
| Sensor Size | 1/4" | inch | Optical format |
| Frame Rate | 30 | fps | At 1080p |
| Interface | CSI | - | Camera Serial Interface |

## Power System

### Battery Specifications

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Battery Type | 18650 Li-ion | - | 2× cells in series |
| Nominal Voltage | 7.4 | V | 2× 3.7V |
| Capacity | 3000 | mAh | Per cell |
| Total Capacity | 3000 | mAh | Series configuration |
| Discharge Rate | 2C | - | 6A continuous |

### Power Consumption

| Mode | Current | Power | Notes |
|------|---------|-------|-------|
| Idle | 500 | mA | 3.7 W | Standby, servos at rest |
| Walking | 1.5 | A | 11.1 W | Typical locomotion |
| Peak | 3.0 | A | 22.2 W | All servos at max load |
| Runtime | ~1.5 | hours | Typical usage pattern |

### Power Distribution

| Rail | Voltage | Current Limit | Components |
|------|---------|---------------|------------|
| Servo Rail | 5.0 | V | 3.0 A | All 12 servos |
| Logic Rail | 3.3 | V | 500 mA | Sensors, GPIO |
| System Rail | 5.0 | V | 1.0 A | Raspberry Pi |

## Control System

### PID Controller Parameters

| Parameter | Symbol | Value | Unit | Notes |
|-----------|--------|-------|------|-------|
| Proportional Gain | Kₚ | 0.033 | - | Roll/pitch balance |
| Integral Gain | Kᵢ | 0.0 | - | Disabled |
| Derivative Gain | Kd | 0.0 | - | Disabled |

**Control Law:**
```
u(t) = Kₚ·e(t) + Kᵢ·∫e(τ)dτ + Kd·de(t)/dt
```

Where:
- `u(t)`: Control output (servo offset in radians)
- `e(t)`: Error signal (target_rpy - measured_rpy)
- `Kₚ, Kᵢ, Kd`: PID gains

### Gait Parameters

#### Walking Gait

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Cycle Sections | 8 | - | Complete gait cycle |
| Steps per Section | 6 | - | Interpolation steps |
| Step Height | 20 | mm | Leg lift height |
| Step Width | 80 | mm | Forward stride length |
| Center of Gravity Offset | -15 | mm | Body balance point |
| Duty Factor | 0.75 | - | Contact time ratio |

#### Trotting Gait

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Cycle Sections | 2 | - | Diagonal pairs |
| Steps per Section | 3 | - | Interpolation steps |
| Step Height | 20 | mm | Leg lift height |
| Step Width | 100 | mm | Forward stride length |
| Center of Gravity Offset | -17 | mm | Body balance point |
| Duty Factor | 0.5 | - | Contact time ratio |

## Kinematic Constraints

### Joint Limits

| Joint | Min Angle | Max Angle | Unit | Notes |
|-------|-----------|-----------|------|-------|
| Leg Hip | -90 | +90 | deg | Lateral rotation |
| Leg Thigh | -45 | +135 | deg | Forward/backward |
| Head Yaw | -90 | +90 | deg | Left/right rotation |
| Head Roll | -70 | +70 | deg | Lateral tilt |
| Head Pitch | -45 | +30 | deg | Up/down tilt |

### Workspace Constraints

| Constraint | Value | Unit | Notes |
|------------|-------|------|-------|
| Maximum Reach | 118 | mm | L₁ + L₂ |
| Minimum Height | 20 | mm | Ground clearance |
| Maximum Height | 95 | mm | Extended leg height |
| Lateral Range | ±49 | mm | Half body width |

## Environmental Specifications

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Operating Temperature | 0-40 | °C | Recommended range |
| Storage Temperature | -20-60 | °C | Non-operating |
| Humidity | 10-90 | % RH | Non-condensing |
| Altitude | 0-2000 | m | Above sea level |

## Mechanical Tolerances

| Component | Tolerance | Notes |
|-----------|-----------|-------|
| Body Dimensions | ±0.5 mm | Manufacturing tolerance |
| Leg Segments | ±0.3 mm | Link length accuracy |
| Servo Positioning | ±2.0 deg | Mechanical backlash |
| Assembly Alignment | ±1.0 deg | Leg attachment angle |

## References

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
2. Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2009). *Robotics: Modelling, Planning and Control*. Springer.
3. Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
4. MG996R Servo Datasheet. Tower Pro.
5. SH3001 IMU Datasheet. Shenzhen Hope Microelectronics.
6. HC-SR04 Ultrasonic Sensor Datasheet. ElecFreaks.

