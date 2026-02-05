# System Architecture

This document describes the overall architecture of the ROS Laser Bot project.

## System Overview

The ROS Laser Bot project consists of several interconnected components working together to achieve autonomous navigation:

```
┌─────────────────┐
│   Stage Sim     │
│   (World Env)   │
└────────┬────────┘
         │
         ├─── Publishes: /base_scan, /odom, /tf
         │
┌────────▼─────────────────────────────┐
│         ROS Master                    │
│    (Topic/Service/TF Coordinator)    │
└────────┬──────────────────────────────┘
         │
         ├─── Subscribes: /cmd_vel
         │
┌────────▼────────┐
│  Robot Model    │
│  (TurtleBot)    │
└─────────────────┘

         ▲
         │
         │ Publishes: /cmd_vel
         │
┌────────┴────────┐
│   Controller    │
│  (go_to_goal)   │
└────────┬────────┘
         │
         ├─── Subscribes: /odom, /base_scan
         ├─── Uses: TF transforms
         │
┌────────▼────────┐
│  SmoothController│
│  (1, 2, or TF)  │
└─────────────────┘
```

## Component Breakdown

### 1. Simulation Environment (Stage)

**Package:** `a2_world`

**Components:**
- **World File** (`autolab.world`): Defines the simulation environment
  - Floor plan with obstacles
  - Target positions
  - Robot model configuration
- **Robot Model** (`irobot_create.inc`): iRobot Create/TurtleBot model
- **Laser Sensor** (`hokuyo_URG-04LX-UG01.inc`): Hokuyo laser scanner model

**Responsibilities:**
- Simulate robot physics and dynamics
- Provide sensor data (laser scans)
- Publish odometry information
- Maintain TF transform tree
- Render visualization

**Topics Published:**
- `/base_scan` (sensor_msgs/LaserScan): Laser range data
- `/odom` (nav_msgs/Odometry): Odometry information
- `/tf` (tf2_msgs/TFMessage): Transform tree

**Topics Subscribed:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

### 2. Navigation Controller

**Package:** `ros2`

**Main Script:** `go_to_goal.py`

**Responsibilities:**
- Initialize ROS node
- Set up TF listener
- Create controller instance
- Continuously:
  - Get robot pose (via TF or direct)
  - Compute control command
  - Publish velocity commands

**Control Flow:**
```
Initialize Node
    ↓
Create TF Listener
    ↓
Create Controller
    ↓
Loop:
    Get Robot Pose
        ↓
    Compute Twist (via Controller)
        ↓
    Publish /cmd_vel
        ↓
    Sleep (10 Hz)
```

### 3. Controller Implementations

#### SmoothController1
- **Input:** Direct pose (x, y, θ)
- **Output:** Twist message
- **Method:** Sequential state machine

#### SmoothController2
- **Input:** Direct pose (x, y, θ)
- **Output:** Twist message
- **Method:** Coordinate transformation + proportional control

#### SmoothController1TF
- **Input:** None (uses TF)
- **Output:** Twist message
- **Method:** Same as SmoothController1 but with TF integration

## Data Flow

### Sensor Data Flow

```
Stage Simulator
    │
    ├─── Laser Scan ────► /base_scan ────► (Available for future use)
    │
    └─── Odometry ──────► /odom ─────────► (Available for future use)
```

### Control Data Flow

```
go_to_goal.py
    │
    ├─── TF Lookup ──────► /tf ──────────► Get robot pose
    │
    ├─── Controller ──────► Compute twist
    │
    └─── Publish ─────────► /cmd_vel ─────► Stage Simulator ──► Robot moves
```

## Coordinate Frames

### TF Tree Structure

```
/odom (Global frame)
  │
  └── /base_footprint (Robot base frame)
```

- **/odom**: Global odometry frame (fixed, represents world)
- **/base_footprint**: Robot base frame (moves with robot)

### Coordinate Conventions

- **X-axis**: Forward direction
- **Y-axis**: Left direction
- **Z-axis**: Up (right-hand rule)
- **Theta (θ)**: Rotation around Z-axis (counter-clockwise positive)

## Launch File Architecture

### all.launch
```
all.launch
    ├── world.launch (Stage simulator)
    └── teleop.launch (Manual control - optional)
```

### world.launch
- Starts Stage simulator
- Loads autolab.world
- Publishes sensor data and TF

### teleop.launch
- Starts keyboard teleoperation
- Allows manual robot control
- Useful for testing

## Message Types

### geometry_msgs/Twist
```python
linear:
  x: float  # Forward velocity (m/s)
  y: float  # Lateral velocity (m/s)
  z: float  # Vertical velocity (m/s)
angular:
  x: float  # Roll rate (rad/s)
  y: float  # Pitch rate (rad/s)
  z: float  # Yaw rate (rad/s)
```

### sensor_msgs/LaserScan
- Range data from Hokuyo laser
- 240° field of view
- Range: 0.20m to 4.0m
- 100 samples (downsampled from 683)

### nav_msgs/Odometry
- Robot pose and velocity
- Covariance information
- Timestamp

## Timing and Rates

- **Controller update rate**: 10 Hz (100ms period)
- **TF lookup rate**: 10 Hz
- **Stage update rate**: ~30 Hz (simulator dependent)
- **Laser scan rate**: ~10 Hz (configurable in world file)

## Dependencies

### ROS Packages
- `rospy`: Python ROS client library
- `geometry_msgs`: Twist message definitions
- `tf`: Transform library
- `stage_ros`: Stage simulator ROS interface
- `turtlebot_teleop`: Teleoperation utilities

### Python Libraries
- `numpy`: Numerical operations (SmoothController2)
- `math`: Mathematical functions
- `tf.transformations`: Quaternion/euler conversions

## Extension Points

### Adding New Controllers

1. Create new controller class in `scripts/`
2. Implement `get_twist()` method
3. Update `go_to_goal.py` to use new controller
4. Document in `docs/CONTROLLERS.md`

### Adding Obstacle Avoidance

1. Subscribe to `/base_scan` topic
2. Process laser data
3. Modify controller to avoid obstacles
4. Integrate with existing controllers

### Adding Path Planning

1. Implement path planning algorithm
2. Generate waypoints
3. Modify controller to follow waypoints
4. Add visualization (RViz markers)

## Performance Considerations

### Computational Load
- SmoothController1: Low (~0.1ms per iteration)
- SmoothController2: Medium (~1ms per iteration, includes matrix operations)
- SmoothController1TF: Low-Medium (~0.5ms per iteration, includes TF lookup)

### Memory Usage
- Minimal: Controllers are stateless (except goal position)
- TF buffer: ~10MB (default)

### Network Bandwidth
- `/cmd_vel`: ~100 bytes/sec (10 Hz)
- `/base_scan`: ~10 KB/sec (if subscribed)
- `/odom`: ~500 bytes/sec
- `/tf`: ~1 KB/sec

## Security Considerations

- No network exposure (local ROS master)
- No authentication required (local use)
- Input validation: Goal coordinates should be within world bounds
- No file system access beyond workspace

---

For implementation details, see individual component documentation in `docs/`.
