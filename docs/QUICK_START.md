# Quick Start Guide

Get up and running with ROS Laser Bot in 5 minutes!

## Prerequisites Check

Before starting, ensure you have:
- ✅ ROS Kinetic installed
- ✅ Catkin workspace set up
- ✅ Required packages installed

Quick check:
```bash
rosversion -d  # Should output "kinetic"
rospack find stage_ros  # Should return a path
```

## Installation Steps

### 1. Clone and Navigate

```bash
git clone https://github.com/Shahrukh19S/ros-autonomous-laser-bot.git
cd ros-autonomous-laser-bot
```

### 2. Fix Filename (One-time)

```bash
cd catkin_ws/src/ros2
mv CMakeLists.txt.txt CMakeLists.txt  # Linux/Mac
# Or on Windows PowerShell:
# Rename-Item CMakeLists.txt.txt CMakeLists.txt
cd ../../..
```

### 3. Build

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Run

**Terminal 1:**
```bash
cd catkin_ws
source devel/setup.bash
roslaunch a2_world all.launch
```

**Terminal 2:**
```bash
cd catkin_ws
source devel/setup.bash
rosrun ros2 go_to_goal.py
```

## What You Should See

1. **Stage window** opens showing:
   - Robot (gray circle with triangle)
   - Red target square at position [2, -2]
   - Environment obstacles

2. **Robot starts moving** toward the target

3. **Robot reaches target** and stops

## Switching Controllers

Edit `catkin_ws/src/ros2/scripts/go_to_goal.py`:

**For SmoothController1:**
```python
from smooth1 import SmoothController1
controller = SmoothController1(1.85, -2)
twist = controller.get_twist(x, y, theta)  # Uncomment this line
# twist = controller.get_twist()  # Comment this line
```

**For SmoothController2:**
```python
from smooth2 import SmoothController2
controller = SmoothController2(2, -2, 3.14159)  # pi
twist = controller.get_twist(x, y, theta)  # Uncomment this line
# twist = controller.get_twist()  # Comment this line
```

Then rebuild:
```bash
cd catkin_ws
catkin_make
```

## Troubleshooting

**Problem:** `catkin_make` fails
- **Solution:** Check CMakeLists.txt.txt is renamed to CMakeLists.txt

**Problem:** Robot doesn't move
- **Solution:** Wait 2-3 seconds after launching Stage before running controller

**Problem:** Import errors
- **Solution:** Make sure you've sourced: `source devel/setup.bash`

**Problem:** Stage window doesn't appear
- **Solution:** Check X11 forwarding if using SSH, or run on local machine

## Next Steps

- Read [BUILD_INSTRUCTIONS.md](../BUILD_INSTRUCTIONS.md) for detailed setup
- Explore [CONTROLLERS.md](CONTROLLERS.md) to understand controller implementations
- Modify goal positions and experiment!

---

**Need help?** Check [BUILD_INSTRUCTIONS.md](../BUILD_INSTRUCTIONS.md) or open an issue on GitHub.
