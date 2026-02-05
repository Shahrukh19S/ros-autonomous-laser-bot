# ROS Laser Bot - TurtleBot Navigation Project

A ROS Kinetic-based autonomous navigation project for TurtleBot using laser sensor feedback. This project implements three different smooth controllers to navigate a robot to a target position in a simulated environment using Stage simulator.

**Repository:** [https://github.com/Shahrukh19S/ros-autonomous-laser-bot](https://github.com/Shahrukh19S/ros-autonomous-laser-bot)

---

## 📹 Demo

### Simulation Environment (Autolab World)

The Stage simulator runs the Autolab world with the robot, target, and obstacles. Below is the environment layout used in the project.

![Autolab simulation environment](Run%20World%20Package.png)

*Autolab world: simulation environment with robot, target (red square), and obstacles.*

---

## 🎯 Project Overview

This project demonstrates autonomous robot navigation using:
- **ROS Kinetic** framework
- **TurtleBot** teleoperation packages
- **Hokuyo URG-04LX-UG01** laser sensor
- **Stage** simulator for environment simulation
- Three different **smooth controllers** for goal-oriented navigation

## ✨ Features

- **Three Controller Implementations:**
  - `SmoothController1`: Basic smooth controller with direct pose feedback
  - `SmoothController2`: Advanced controller with coordinate transformation and PID-like control
  - `SmoothController1TF`: TF-based controller using ROS transform framework

- **Simulated Environment:**
  - Stage-based simulation environment
  - Autolab world with obstacles
  - Configurable target positions
  - Laser sensor integration

- **Modular Design:**
  - Separate world package for simulation setup
  - Controller package with reusable controller classes
  - Easy-to-use launch files

## 📋 Prerequisites

- **ROS Kinetic** (Ubuntu 16.04)
- **Catkin** build system
- **Stage ROS** (`stage_ros` package)
- **TurtleBot Teleop** (`turtlebot_teleop` package)
- **Python 2.7** (ROS Kinetic requirement)
- **NumPy** (for SmoothController2)

## 🚀 Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/Shahrukh19S/ros-autonomous-laser-bot.git
cd ros-autonomous-laser-bot
```

### 2. Build the Workspace

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Run the Simulation

```bash
# Launch the world and teleoperation
roslaunch a2_world all.launch

# In a new terminal, run the go-to-goal controller
rosrun ros2 go_to_goal.py
```

For detailed build instructions, see [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md).

## 📁 Project Structure

```
ros-autonomous-laser-bot/
├── catkin_ws/                    # Catkin workspace
│   └── src/
│       ├── a2_world/             # World package (simulation environment)
│       │   ├── launch/           # Launch files
│       │   │   ├── all.launch    # Main launch file
│       │   │   ├── world.launch  # Stage simulator launch
│       │   │   └── teleop.launch # Teleoperation launch
│       │   └── world/            # World files
│       │       ├── autolab.world # Main world file
│       │       └── hokuyo_URG-04LX-UG01.inc  # Laser sensor model
│       └── ros2/                 # Controller package
│           └── scripts/          # Python scripts
│               ├── go_to_goal.py      # Main navigation script
│               ├── smooth1.py         # SmoothController1 implementation
│               ├── smooth2.py         # SmoothController2 implementation
│               └── smooth1_tf.py      # SmoothController1TF implementation
├── BUILD_INSTRUCTIONS.md         # Detailed build guide
├── CONTRIBUTING.md               # Contribution guidelines
├── LICENSE                       # License file
└── README.md                     # This file
```

## 🎮 Usage

### Running Different Controllers

The `go_to_goal.py` script can be configured to use different controllers by modifying the imports and controller instantiation:

**SmoothController1:**
```python
from smooth1 import SmoothController1
controller = SmoothController1(1.85, -2)
twist = controller.get_twist(x, y, theta)
```

**SmoothController2:**
```python
from smooth2 import SmoothController2
controller = SmoothController2(2, -2, pi)
twist = controller.get_twist(x, y, theta)
```

**SmoothController1TF:**
```python
from smooth1_tf import SmoothController1TF
controller = SmoothController1TF(listener, 1.85, -2)
twist = controller.get_twist()
```

### Setting Target Position

Modify the goal coordinates in `go_to_goal.py`:
```python
controller = SmoothController1(goal_x, goal_y)  # For SmoothController1
controller = SmoothController2(goal_x, goal_y, goal_theta)  # For SmoothController2
controller = SmoothController1TF(listener, goal_x, goal_y)  # For SmoothController1TF
```

## 🔧 Configuration

### World Configuration

The simulation world can be modified in `catkin_ws/src/a2_world/world/autolab.world`:
- Target positions
- Obstacle placement
- Robot initial pose
- Laser sensor parameters

### Controller Parameters

**SmoothController2** uses the following gains (modifiable in `smooth2.py`):
- `K_rho = 0.6`: Linear velocity gain
- `K_alpha = 1.6`: Angular velocity gain for heading
- `K_theta = 0.3`: Angular velocity gain for orientation

## 📚 Documentation

- [Build Instructions](BUILD_INSTRUCTIONS.md) - Detailed setup and build guide
- [Controller Documentation](docs/CONTROLLERS.md) - Detailed controller explanations
- [Kinematics Theory](kinematics3_quad.pdf) - Mathematical background
- [Project Tasks](Ros%20Laser%20Bot(ALL%20TASKS).pdf) - Original project requirements

## 🧪 Testing

1. Launch the simulation:
   ```bash
   roslaunch a2_world all.launch
   ```

2. In another terminal, run the controller:
   ```bash
   rosrun ros2 go_to_goal.py
   ```

3. Observe the robot navigating to the target position (red square at [2, -2])

## 🤝 Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS Kinetic community
- TurtleBot project
- Stage simulator developers
- Hokuyo for laser sensor specifications

## 📧 Support

For issues and questions, please open an issue on the [GitHub repository](https://github.com/Shahrukh19S/ros-autonomous-laser-bot/issues).

---

**Note:** This project requires ROS Kinetic, which is designed for Ubuntu 16.04. For newer Ubuntu versions, consider migrating to ROS Noetic or ROS 2.
