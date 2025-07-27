# Inverse Kinematics Theory

## Coordinate Systems

### Body-Fixed Frame (B)

The body-fixed frame is attached to the robot's center of mass with:
- **X-axis**: Forward direction (positive forward)
- **Y-axis**: Lateral direction (positive left)
- **Z-axis**: Vertical direction (positive upward)

### Leg Frame (L)

Each leg has its own coordinate frame with:
- **Y-axis**: Forward/backward motion in the leg plane
- **Z-axis**: Vertical motion in the leg plane
- Origin at the leg attachment point on the body

## Forward Kinematics

### Leg Geometry

Each leg consists of two segments:
- **Leg segment (L₁)**: Length = 42.0 mm (thigh)
- **Foot segment (L₂)**: Length = 76.0 mm (shank)

The leg forms a 2-DOF planar mechanism in the Y-Z plane.

### Forward Kinematics Equation

Given joint angles (α, β), the foot position in leg coordinates is:

```
y = L₁·sin(α) + L₂·sin(α + β)
z = L₁·cos(α) + L₂·cos(α + β)
```

Where:
- `α`: Hip joint angle (leg angle) in radians
- `β`: Thigh joint angle (foot angle) in radians
- `y`: Forward position in leg frame (mm)
- `z`: Vertical position in leg frame (mm)

## Inverse Kinematics

### Problem Statement

Given a desired foot position (y, z) in leg coordinates, find the joint angles (α, β) that achieve this position.

### Solution Method: Law of Cosines

The inverse kinematics solution uses the law of cosines on the triangle formed by the two leg segments and the distance vector to the foot.

#### Step 1: Calculate Distance to Foot

```
u = √(y² + z²)
```

Where `u` is the Euclidean distance from the leg attachment point to the desired foot position.

#### Step 2: Calculate Foot Angle (β)

Using the law of cosines on the triangle with sides L₁, L₂, and u:

```
cos(β) = (L₁² + L₂² - u²) / (2·L₁·L₂)
β = arccos(cos(β))
```

**Constraint:** `-1 ≤ cos(β) ≤ 1` → `u ≤ L₁ + L₂` (workspace limit)

#### Step 3: Calculate Leg Angle (α)

The leg angle consists of two components:
1. Angle to the distance vector: `θ₁ = atan2(y, z)`
2. Angle within the triangle: `θ₂ = arccos((L₁² + u² - L₂²) / (2·L₁·u))`

Total leg angle:
```
α = θ₁ + θ₂ + pitch_offset
```

Where `pitch_offset` accounts for body pitch angle in the field coordinate system.

### Complete Algorithm

```python
def fieldcoord2polar(coord, leg_length, foot_length, pitch_offset):
    """
    Convert field coordinates to joint angles using inverse kinematics.
    
    Parameters:
    -----------
    coord : tuple
        (y, z) position in leg frame (mm)
    leg_length : float
        Length of leg segment L₁ (mm)
    foot_length : float
        Length of foot segment L₂ (mm)
    pitch_offset : float
        Body pitch angle compensation (rad)
    
    Returns:
    --------
    alpha : float
        Leg joint angle (degrees)
    beta : float
        Foot joint angle (degrees)
    """
    y, z = coord
    
    # Step 1: Calculate distance to foot
    u = sqrt(y**2 + z**2)
    
    # Step 2: Calculate foot angle using law of cosines
    cos_beta = (foot_length**2 + leg_length**2 - u**2) / (2 * foot_length * leg_length)
    cos_beta = clamp(cos_beta, -1.0, 1.0)  # Ensure valid range
    beta = acos(cos_beta)
    
    # Step 3: Calculate leg angle
    theta1 = atan2(y, z)  # Angle to distance vector
    cos_theta2 = (leg_length**2 + u**2 - foot_length**2) / (2 * leg_length * u)
    cos_theta2 = clamp(cos_theta2, -1.0, 1.0)
    theta2 = acos(cos_theta2)
    alpha = theta1 + theta2 + pitch_offset
    
    # Convert to degrees
    return degrees(alpha), degrees(beta)
```

## Body Pose Transformation

### Euler Angle Representation

The body pose is represented using Euler angles (roll, pitch, yaw) following the ZYX convention:

1. **Yaw (ψ)**: Rotation about Z-axis
2. **Pitch (θ)**: Rotation about Y-axis
3. **Roll (φ)**: Rotation about X-axis

### Rotation Matrices

The rotation matrix from body frame to world frame is:

```
R = R_z(ψ) · R_y(θ) · R_x(φ)
```

Where:

```
R_x(φ) = [1      0         0    ]
         [0  cos(φ)  -sin(φ) ]
         [0  sin(φ)   cos(φ) ]

R_y(θ) = [ cos(θ)  0  sin(θ) ]
         [   0     1    0    ]
         [-sin(θ)  0  cos(θ) ]

R_z(ψ) = [cos(ψ)  -sin(ψ)  0]
         [sin(ψ)   cos(ψ)  0]
         [  0        0     1]
```

### Leg Attachment Points

The four leg attachment points in the body frame are:

```
BODY_STRUCT = [
    [-BODY_WIDTH/2, -BODY_LENGTH/2,  0],  # Front-left
    [ BODY_WIDTH/2, -BODY_LENGTH/2,  0],  # Front-right
    [-BODY_WIDTH/2,  BODY_LENGTH/2,  0],  # Rear-left
    [ BODY_WIDTH/2,  BODY_LENGTH/2,  0]   # Rear-right
]
```

### Coordinate Transformation

To transform a leg's foot position from body frame to world frame:

```
P_world = P_body + R · BODY_STRUCT[i]
```

Where:
- `P_body`: Body center position in world frame
- `R`: Rotation matrix from body to world
- `BODY_STRUCT[i]`: Leg attachment point in body frame

## Workspace Analysis

### Reachable Workspace

The reachable workspace for each leg is a circular annulus:

**Inner radius:** `r_min = |L₁ - L₂| = |42 - 76| = 34 mm`  
**Outer radius:** `r_max = L₁ + L₂ = 42 + 76 = 118 mm`

### Singularity Analysis

Singularities occur when:
1. **Extended singularity:** `u = L₁ + L₂` → Leg fully extended
2. **Folded singularity:** `u = |L₁ - L₂|` → Leg fully folded

At singularities, the Jacobian matrix becomes singular, and small changes in desired position require large joint velocity changes.

### Joint Limits

The practical workspace is further constrained by joint limits:
- **Hip joint:** ±90° (lateral rotation)
- **Thigh joint:** -45° to +135° (forward/backward)

## Gait Generation

### Walking Gait

The walking gait uses a sequential leg lifting pattern with 8 sections per cycle:

```
Section 1: Lift front-left leg
Section 2: Pause
Section 3: Lift rear-left leg
Section 4: Pause
Section 5: Lift front-right leg
Section 6: Pause
Section 7: Lift rear-right leg
Section 8: Pause
```

**Trajectory Function:**
```
y(step) = y_origin + (step_width/2) · (cos(π·step/(steps-1)) - direction)
z(step) = z_origin - step_height · (step/(steps-1))
```

### Trotting Gait

The trotting gait uses diagonal leg pairs with 2 sections per cycle:

```
Section 1: Lift front-left + rear-right (diagonal pair)
Section 2: Lift front-right + rear-left (diagonal pair)
```

**Trajectory Function:**
Same as walking, but with different timing and coordination.

## Stability Analysis

### Static Stability

For static stability, the projection of the center of mass must remain within the support polygon formed by the feet in contact with the ground.

**Support Polygon:** Convex hull of all feet in contact.

**Stability Margin:** Minimum distance from center of mass projection to support polygon edge.

### Dynamic Stability

Dynamic stability is maintained through:
1. **Gait selection:** Walking (3-point support) vs. trotting (2-point support)
2. **Center of gravity offset:** Adjusted based on movement direction
3. **IMU feedback:** PID control for roll/pitch balance

## References

1. Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson. Chapter 4: Forward Kinematics, Chapter 5: Inverse Kinematics.
2. Siciliano, B., et al. (2009). *Robotics: Modelling, Planning and Control*. Springer. Chapter 7: Kinematics.
3. Spong, M. W., et al. (2006). *Robot Modeling and Control*. Wiley. Chapter 3: Forward and Inverse Kinematics.

