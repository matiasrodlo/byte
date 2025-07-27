# Control Theory

## PID Controller

### Control Law

Byte uses a Proportional-Integral-Derivative (PID) controller for body orientation stabilization:

```
u(t) = Kₚ·e(t) + Kᵢ·∫₀ᵗ e(τ)dτ + Kd·(de(t)/dt)
```

Where:
- `u(t)`: Control output (servo offset in radians)
- `e(t)`: Error signal = `target_rpy - measured_rpy` (degrees)
- `Kₚ`: Proportional gain = 0.033
- `Kᵢ`: Integral gain = 0.0 (disabled)
- `Kd`: Derivative gain = 0.0 (disabled)

### Current Configuration

Byte currently uses **proportional-only control** (P-controller):

```
u(t) = Kₚ·e(t)
```

**Rationale:**
- **Integral term disabled:** Prevents integral windup during transient disturbances
- **Derivative term disabled:** Avoids amplification of sensor noise from IMU
- **Proportional gain:** Empirically tuned for stable balance during locomotion

### Tuning Methodology

The proportional gain `Kₚ = 0.033` was determined through:

1. **Initial estimate:** Based on system dynamics and servo response time
2. **Empirical tuning:** Tested values from 0.01 to 0.1 in increments of 0.01
3. **Stability criteria:**
   - No oscillations during steady-state walking
   - Adequate response to pitch/roll disturbances
   - Smooth recovery from external perturbations

### Control Loop

```
┌─────────┐     ┌──────────┐     ┌─────────┐     ┌─────────┐
│ Target  │────▶│   Error  │────▶│   PID   │────▶│  Servos  │
│   RPY   │     │  e(t)    │     │Controller│     │  Offset  │
└─────────┘     └──────────┘     └─────────┘     └─────────┘
     ▲                                │                  │
     │                                │                  │
     └────────────────────────────────┴──────────────────┘
                              │
                              ▼
                        ┌─────────┐
                        │   IMU   │
                        │  (SH3001)│
                        └─────────┘
```

### Error Signal

The error signal is computed as:

```
e_roll(t) = target_roll - measured_roll
e_pitch(t) = target_pitch - measured_pitch
```

Where:
- `target_roll, target_pitch`: Desired body orientation (typically 0°)
- `measured_roll, measured_pitch`: IMU-measured orientation

### Control Output

The control output is applied as a servo offset:

```
servo_offset_roll = Kₚ · e_roll(t) · (π/180)  # Convert deg to rad
servo_offset_pitch = Kₚ · e_pitch(t) · (π/180)
```

This offset is added to the nominal leg angles to maintain balance.

## Gait Control

### Walking Gait

The walking gait uses a **sequential leg lifting pattern** with 8 sections per cycle:

```
Time:  0    1    2    3    4    5    6    7    8
       │    │    │    │    │    │    │    │    │
FL:    ↑    ▬    ▬    ▬    ▬    ▬    ▬    ▬    ▬
RL:    ▬    ▬    ▬    ↑    ▬    ▬    ▬    ▬    ▬
FR:    ▬    ▬    ▬    ▬    ▬    ↑    ▬    ▬    ▬
RR:    ▬    ▬    ↑    ▬    ▬    ▬    ▬    ▬    ▬
       │    │    │    │    │    │    │    │    │
       └────┴────┴────┴────┴────┴────┴────┴────┘
            Complete Gait Cycle
```

**Characteristics:**
- **Duty factor:** 0.75 (75% contact, 25% swing)
- **Stability:** 3-point support during swing phase
- **Speed:** Slower but more stable than trotting

### Trotting Gait

The trotting gait uses **diagonal leg pairs** with 2 sections per cycle:

```
Time:  0    1    2
       │    │    │
FL:    ↑    ▬    ▬
RL:    ▬    ↑    ▬
FR:    ▬    ↑    ▬
RR:    ↑    ▬    ▬
       │    │    │
       └────┴────┘
    Complete Cycle
```

**Characteristics:**
- **Duty factor:** 0.5 (50% contact, 50% swing)
- **Stability:** 2-point support during swing phase
- **Speed:** Faster but less stable than walking

## Trajectory Generation

### Cosine Trajectory (Horizontal Motion)

For smooth horizontal leg motion during swing phase:

```
y(step) = y_origin + (step_width/2) · (cos(θ) - direction) · direction
```

Where:
- `θ = step · π / (STEP_COUNT - 1)` (normalized phase [0, π])
- `direction`: +1 for forward, -1 for backward

**Properties:**
- Zero velocity at endpoints (smooth start/stop)
- Maximum velocity at midpoint
- Continuous acceleration profile
- Minimizes mechanical stress

### Linear Trajectory (Vertical Motion)

For vertical leg lift:

```
z(step) = Z_ORIGIN - STEP_HEIGHT · (step / (STEP_COUNT - 1))
```

**Properties:**
- Constant velocity vertical motion
- Simple computation
- Adequate for small vertical displacements (20 mm)

## Stability Analysis

### Static Stability

Static stability requires the center of mass projection to remain within the support polygon.

**Support Polygon:** Convex hull of all feet in contact with ground.

**Stability Margin:**
```
margin = min(distance(CoM_projection, support_polygon_edge))
```

For stable walking:
- **Walking gait:** margin > 0 (3-point support)
- **Trotting gait:** margin > 0 (2-point support, more critical)

### Dynamic Stability

Dynamic stability is maintained through:

1. **Gait selection:** Choose gait based on speed/stability tradeoff
2. **Center of gravity offset:** Adjusted based on movement direction
   - Forward: CoG offset = -15 to -17 mm
   - Backward: CoG offset = +8 mm
3. **IMU feedback:** PID control for roll/pitch compensation

### ZMP (Zero Moment Point) Analysis

For dynamic stability, the Zero Moment Point should remain within the support polygon:

```
ZMP = CoM - (CoM_height / g) · CoM_acceleration
```

Where:
- `CoM`: Center of mass position
- `g`: Gravitational acceleration (9.81 m/s²)
- `CoM_acceleration`: Acceleration of center of mass

## References

1. Franklin, G. F., Powell, J. D., & Workman, M. L. (1998). *Digital Control of Dynamic Systems* (3rd ed.). Addison-Wesley. Chapter 4: PID Control.
2. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 8: Motion Control.
3. Vukobratović, M., & Borovac, B. (2004). Zero-moment point—thirty five years of its life. *International Journal of Humanoid Robotics*, 1(1), 157-173.

