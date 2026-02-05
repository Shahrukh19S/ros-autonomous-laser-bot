# Controller Documentation

This document provides detailed explanations of the three smooth controllers implemented in this project.

## Table of Contents

1. [Overview](#overview)
2. [SmoothController1](#smoothcontroller1)
3. [SmoothController2](#smoothcontroller2)
4. [SmoothController1TF](#smoothcontroller1tf)
5. [Comparison](#comparison)
6. [Usage Examples](#usage-examples)

## Overview

All three controllers implement smooth navigation algorithms to move a robot from its current position to a target goal position. They differ in their approach to coordinate transformation, pose estimation, and control law implementation.

### Common Features

- **Goal-oriented navigation**: All controllers navigate to a specified goal position
- **Velocity control**: Output `Twist` messages for robot motion
- **Smooth motion**: Designed to provide smooth, continuous motion

### Key Differences

| Controller | Pose Input | Coordinate System | TF Usage |
|------------|-----------|-------------------|----------|
| SmoothController1 | Direct (x, y, θ) | Global frame | No |
| SmoothController2 | Direct (x, y, θ) | Robot frame (transformed) | No |
| SmoothController1TF | TF lookup | Global frame | Yes |

## SmoothController1

### Description

`SmoothController1` is a basic smooth controller that uses direct pose feedback in the global coordinate frame. It implements a simple state machine-based control strategy.

### Implementation Details

**File:** `catkin_ws/src/ros2/scripts/smooth1.py`

**Initialization:**
```python
controller = SmoothController1(goalIx, goalIy)
```
- `goalIx`: Target X coordinate in global frame
- `goalIy`: Target Y coordinate in global frame

**Control Method:**
```python
twist = controller.get_twist(x, y, theta)
```
- `x`: Current robot X position
- `y`: Current robot Y position
- `theta`: Current robot orientation (radians)

### Control Logic

The controller uses a sequential approach:

1. **X-axis alignment**: First moves toward the goal X coordinate
   - If `dth = goalIx - x > 0`: Move forward with linear velocity 0.2 m/s

2. **Orientation adjustment**: Once X is reached, adjust orientation
   - Target orientation: `thetades = 1.466` radians (~84°)
   - If `angle = thetades - theta > 0`: Rotate with angular velocity -0.4 rad/s

3. **Y-axis movement**: After orientation, move toward goal Y
   - If `dty = y - goalIy > 0`: Move forward
   - Otherwise: Stop (goal reached)

### Characteristics

- **Pros:**
  - Simple and easy to understand
  - No external dependencies beyond ROS
  - Fast execution

- **Cons:**
  - Sequential approach may not be optimal
  - Hard-coded orientation target
  - No obstacle avoidance
  - Limited to specific goal positions

### Parameters

- Linear velocity: `0.2 m/s`
- Angular velocity: `-0.4 rad/s`
- Target orientation: `1.466 rad` (hard-coded)

## SmoothController2

### Description

`SmoothController2` implements a more sophisticated control algorithm using coordinate transformation and proportional control gains. It transforms the goal position to the robot's local coordinate frame and uses a polar coordinate representation.

### Implementation Details

**File:** `catkin_ws/src/ros2/scripts/smooth2.py`

**Initialization:**
```python
controller = SmoothController2(goalIx, goalIy, g0)
```
- `goalIx`: Target X coordinate in global frame
- `goalIy`: Target Y coordinate in global frame
- `g0`: Target orientation (radians)

**Control Method:**
```python
twist = controller.get_twist(x, y, theta)
```

### Control Algorithm

The controller implements the following steps:

1. **Coordinate Transformation:**
   - Robot pose: `El = [x, y, θ]ᵀ`
   - Goal pose: `Gl = [goalIx, goalIy, g0]ᵀ`
   - Rotation matrix `Rcw` transforms global to robot frame:
     ```
     Rcw = [cos(θ)  sin(θ)  0]
           [-sin(θ) cos(θ)  0]
           [0       0       1]
     ```
   - Goal in robot frame: `gr = Rcw × (Gl - El)`

2. **Polar Coordinate Conversion:**
   - Distance: `ρ = √(grx² + gry²)`
   - Heading angle: `α = atan2(gry, grx)`
   - Orientation error: `gr0` (goal orientation - current orientation)

3. **Control Law:**
   - Linear velocity: `v = K_ρ × ρ`
   - Angular velocity: `ω = K_α × α - K_θ × gr0`
   
   Where:
   - `K_ρ = 0.6`: Linear velocity gain
   - `K_α = 1.6`: Angular velocity gain for heading
   - `K_θ = 0.3`: Angular velocity gain for orientation

### Characteristics

- **Pros:**
  - Simultaneous position and orientation control
  - Smooth, continuous motion
  - Tunable control gains
  - Mathematically sound approach
  - Handles orientation goals

- **Cons:**
  - Requires NumPy dependency
  - More complex implementation
  - Requires tuning of control gains

### Parameters

- `K_rho = 0.6`: Linear velocity gain
- `K_alpha = 1.6`: Angular velocity gain
- `K_theta = 0.3`: Orientation gain

### Tuning Guidelines

- **Increase K_rho**: Faster linear motion, but may overshoot
- **Increase K_alpha**: Faster heading correction, but may oscillate
- **Increase K_theta**: Faster orientation correction

## SmoothController1TF

### Description

`SmoothController1TF` is similar to `SmoothController1` but uses ROS TF (Transform) framework to obtain robot pose instead of receiving it as parameters. This makes it more robust and aligned with ROS best practices.

### Implementation Details

**File:** `catkin_ws/src/ros2/scripts/smooth1_tf.py`

**Initialization:**
```python
controller = SmoothController1TF(listener, goalIx, goalIy)
```
- `listener`: TF TransformListener instance
- `goalIx`: Target X coordinate
- `goalIy`: Target Y coordinate

**Control Method:**
```python
twist = controller.get_twist()
```
No parameters needed - pose is obtained via TF lookup.

### Control Logic

1. **TF Lookup:**
   - Queries transform from `/odom` to `/base_footprint`
   - Extracts position (x, y) and orientation (θ)
   - Handles TF exceptions gracefully

2. **Same Control Strategy as SmoothController1:**
   - Sequential X → orientation → Y approach
   - Same velocity parameters

### Characteristics

- **Pros:**
  - Uses ROS TF framework (best practice)
  - No need to manually pass pose data
  - More robust to timing issues
  - Better integration with ROS ecosystem

- **Cons:**
  - Same limitations as SmoothController1
  - Requires TF tree to be properly set up
  - Slightly more overhead from TF lookups

### TF Frame Structure

```
/odom (global frame)
  └── /base_footprint (robot base frame)
```

The controller looks up the transform from `/odom` to `/base_footprint` to get the robot's pose in the global frame.

## Comparison

### Performance

| Aspect | SmoothController1 | SmoothController2 | SmoothController1TF |
|--------|-------------------|-------------------|---------------------|
| Speed | Fast | Medium | Medium |
| Smoothness | Medium | High | Medium |
| Accuracy | Medium | High | Medium |
| Orientation Control | Limited | Full | Limited |

### Use Cases

- **SmoothController1**: Simple navigation tasks, learning/teaching
- **SmoothController2**: Precise navigation with orientation requirements
- **SmoothController1TF**: ROS-integrated applications, when TF is available

## Usage Examples

### Example 1: Using SmoothController1

```python
from smooth1 import SmoothController1

controller = SmoothController1(goal_x=2.0, goal_y=-2.0)

# In your control loop
x, y, theta = get_robot_pose()  # Your pose retrieval method
twist = controller.get_twist(x, y, theta)
cmd_vel_publisher.publish(twist)
```

### Example 2: Using SmoothController2

```python
from smooth2 import SmoothController2
import math

controller = SmoothController2(
    goalIx=2.0, 
    goalIy=-2.0, 
    g0=math.pi/2  # 90 degrees
)

# In your control loop
x, y, theta = get_robot_pose()
twist = controller.get_twist(x, y, theta)
cmd_vel_publisher.publish(twist)
```

### Example 3: Using SmoothController1TF

```python
import tf
from smooth1_tf import SmoothController1TF

listener = tf.TransformListener()
controller = SmoothController1TF(listener, goal_x=2.0, goal_y=-2.0)

# In your control loop
twist = controller.get_twist()  # No pose parameters needed!
cmd_vel_publisher.publish(twist)
```

## Mathematical Background

For detailed mathematical theory behind these controllers, refer to:
- `kinematics3_quad.pdf` - Contains kinematic equations and control theory
- [ROS Navigation Stack Documentation](http://wiki.ros.org/navigation)

## Troubleshooting

### Controller doesn't reach goal

- Check goal coordinates are within world bounds
- Verify robot pose is being updated correctly
- For SmoothController2: Adjust control gains

### Oscillatory behavior

- Reduce control gains (especially K_alpha for SmoothController2)
- Check update rate (should be ~10 Hz)

### TF lookup failures (SmoothController1TF)

- Ensure Stage is running and publishing TF
- Check TF tree: `rosrun tf view_frames`
- Wait a few seconds after launching Stage before starting controller

---

For questions or issues, please refer to the main [README.md](../README.md) or [open an issue](https://github.com/Shahrukh19S/ros-autonomous-laser-bot/issues) on GitHub.
