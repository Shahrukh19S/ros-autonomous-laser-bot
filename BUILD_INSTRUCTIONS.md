# Build Instructions

This document provides detailed step-by-step instructions for building and running the ROS Laser Bot project on your system.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [ROS Kinetic Installation](#ros-kinetic-installation)
3. [Dependencies Installation](#dependencies-installation)
4. [Workspace Setup](#workspace-setup)
5. [Building the Project](#building-the-project)
6. [Running the Project](#running-the-project)
7. [Troubleshooting](#troubleshooting)

## System Requirements

### Operating System
- **Ubuntu 16.04** (Xenial) - Recommended
- **Ubuntu 18.04** (Bionic) - May work with adjustments
- **Ubuntu 20.04+** - Requires ROS Noetic migration (not covered in this guide)

### Hardware Requirements
- Minimum 2GB RAM
- 10GB free disk space
- Internet connection for package installation

### Software Requirements
- Python 2.7
- Git
- Build tools (gcc, g++, make, cmake)

## ROS Kinetic Installation

### Step 1: Configure Ubuntu Repositories

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### Step 2: Set Up Keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### Step 3: Update Package Index

```bash
sudo apt update
```

### Step 4: Install ROS Kinetic Desktop Full

```bash
sudo apt install ros-kinetic-desktop-full
```

### Step 5: Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Step 6: Environment Setup

Add ROS Kinetic to your bashrc:

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 7: Install Dependencies for Building Packages

```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Dependencies Installation

### Required ROS Packages

Install the following ROS packages:

```bash
sudo apt install ros-kinetic-stage-ros
sudo apt install ros-kinetic-turtlebot-teleop
sudo apt install ros-kinetic-tf
sudo apt install ros-kinetic-geometry-msgs
```

### Python Dependencies

Install NumPy (required for SmoothController2):

```bash
sudo apt install python-numpy
```

Or using pip:

```bash
pip install numpy
```

## Workspace Setup

### Step 1: Navigate to Project Directory

```bash
cd ros-autonomous-laser-bot
```

### Step 2: Verify Package Structure

Ensure your workspace structure looks like this:

```
catkin_ws/
└── src/
    ├── a2_world/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   └── world/
    └── ros2/
        ├── CMakeLists.txt
        ├── package.xml
        └── scripts/
```

## Building the Project

### Step 1: Navigate to Workspace

```bash
cd catkin_ws
```

### Step 2: Source ROS Setup

```bash
source /opt/ros/kinetic/setup.bash
```

### Step 3: Install Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Workspace

```bash
catkin_make
```

**Expected Output:**
```
Base path: /path/to/catkin_ws
Source space: /path/to/catkin_ws/src
Build space: /path/to/catkin_ws/build
Devel space: /path/to/catkin_ws/devel
Install space: /path/to/catkin_ws/install
...
[ 50%] Built target ...
[100%] Built target ...
```

### Step 5: Source the Workspace

```bash
source devel/setup.bash
```

**Important:** Add this to your `~/.bashrc` for persistence:

```bash
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
```

### Step 6: Verify Installation

Check that packages are recognized:

```bash
rospack find a2_world
rospack find ros2
```

Both commands should return valid paths.

## Running the Project

### Method 1: Using Launch Files (Recommended)

**Terminal 1 - Launch Simulation:**
```bash
cd catkin_ws
source devel/setup.bash
roslaunch a2_world all.launch
```

This will:
- Start the Stage simulator with the autolab world
- Launch the teleoperation node (optional, for manual control)

**Terminal 2 - Run Controller:**
```bash
cd catkin_ws
source devel/setup.bash
rosrun ros2 go_to_goal.py
```

### Method 2: Manual Launch

**Terminal 1 - Start Stage:**
```bash
cd catkin_ws
source devel/setup.bash
roslaunch a2_world world.launch
```

**Terminal 2 - Run Controller:**
```bash
cd catkin_ws
source devel/setup.bash
rosrun ros2 go_to_goal.py
```

**Terminal 3 - Optional Teleoperation:**
```bash
cd catkin_ws
source devel/setup.bash
roslaunch a2_world teleop.launch
```

### Verifying ROS Nodes

In a new terminal, check running nodes:

```bash
rosnode list
```

You should see:
- `/go_to_goal`
- `/stageros`
- `/teleop_key` (if teleop is running)

### Checking Topics

```bash
rostopic list
```

Expected topics:
- `/cmd_vel` - Velocity commands
- `/base_scan` - Laser scan data
- `/odom` - Odometry data
- `/tf` - Transform tree

### Visualizing with RViz (Optional)

```bash
rosrun rviz rviz
```

Add displays for:
- RobotModel
- LaserScan
- TF
- Odometry

## Troubleshooting

### Issue: `catkin_make` fails with "No rule to make target"

**Solution:**
- Ensure CMakeLists.txt.txt is renamed to CMakeLists.txt
- Check that all package.xml files are valid
- Run `catkin_make clean` and rebuild

### Issue: "Package 'a2_world' not found"

**Solution:**
```bash
cd catkin_ws
source devel/setup.bash
rospack profile
```

### Issue: Stage window doesn't appear

**Solution:**
- Check if X11 forwarding is enabled (for SSH)
- Verify display: `echo $DISPLAY`
- Try running with `--gui` flag

### Issue: "ImportError: No module named numpy"

**Solution:**
```bash
sudo apt install python-numpy
# or
pip install numpy
```

### Issue: TF lookup exceptions

**Solution:**
- Ensure Stage is running before starting the controller
- Wait a few seconds after launching Stage before running the controller
- Check TF tree: `rosrun tf view_frames`

### Issue: Robot doesn't move

**Solution:**
- Verify `/cmd_vel` topic is publishing: `rostopic echo /cmd_vel`
- Check controller goal coordinates match target position
- Verify robot pose: `rostopic echo /odom`

### Issue: Permission denied for Python scripts

**Solution:**
```bash
chmod +x catkin_ws/src/ros2/scripts/*.py
```

### Issue: ROS Kinetic not found

**Solution:**
- Verify installation: `rosversion -d` should return "kinetic"
- Check setup.bash is sourced: `echo $ROS_DISTRO`
- Reinstall if necessary

## Additional Resources

- [ROS Kinetic Installation Guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Catkin Workspace Tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- [Stage Simulator Documentation](http://wiki.ros.org/stage)
- [TurtleBot Documentation](http://wiki.ros.org/turtlebot)

## Next Steps

After successful build:
1. Read [Controller Documentation](docs/CONTROLLERS.md) to understand controller implementations
2. Experiment with different goal positions
3. Modify controller parameters
4. Try different world configurations

---

**Note:** If you encounter issues not covered here, please open an issue on the [GitHub repository](https://github.com/Shahrukh19S/ros-autonomous-laser-bot/issues) with:
- Error messages
- System information (`lsb_release -a`)
- ROS version (`rosversion -d`)
- Steps to reproduce
