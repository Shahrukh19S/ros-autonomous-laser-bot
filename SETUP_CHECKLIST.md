# Setup Checklist

Use this checklist to verify your ROS Laser Bot installation is complete and ready to use.

## Pre-Installation

- [ ] Ubuntu 16.04 (or compatible) installed
- [ ] Internet connection available
- [ ] At least 10GB free disk space
- [ ] Terminal/command line access

## ROS Kinetic Installation

- [ ] ROS Kinetic repositories configured
- [ ] ROS keys added
- [ ] `sudo apt update` completed successfully
- [ ] `ros-kinetic-desktop-full` installed
- [ ] `rosdep init` completed
- [ ] `rosdep update` completed
- [ ] ROS setup.bash added to ~/.bashrc
- [ ] `rosversion -d` returns "kinetic"
- [ ] `echo $ROS_DISTRO` returns "kinetic"

## Dependencies

- [ ] `ros-kinetic-stage-ros` installed
- [ ] `ros-kinetic-turtlebot-teleop` installed
- [ ] `ros-kinetic-tf` installed
- [ ] `ros-kinetic-geometry-msgs` installed
- [ ] Python NumPy installed (`python-numpy` or `pip install numpy`)
- [ ] All packages verified: `rospack find stage_ros` returns path

## Project Setup

- [ ] Repository cloned/downloaded
- [ ] Navigated to project directory
- [ ] CMakeLists.txt.txt renamed to CMakeLists.txt in `catkin_ws/src/ros2/`
- [ ] Project structure verified:
  - [ ] `catkin_ws/src/a2_world/` exists
  - [ ] `catkin_ws/src/ros2/` exists
  - [ ] All launch files present
  - [ ] All Python scripts present

## Building

- [ ] Navigated to `catkin_ws` directory
- [ ] `catkin_make` completed without errors
- [ ] `source devel/setup.bash` executed
- [ ] Packages recognized:
  - [ ] `rospack find a2_world` returns path
  - [ ] `rospack find ros2` returns path
- [ ] Workspace sourced in ~/.bashrc (optional but recommended)

## Verification

### Test 1: Launch Simulation
- [ ] `roslaunch a2_world all.launch` starts successfully
- [ ] Stage window appears
- [ ] Robot visible in simulation
- [ ] Target (red square) visible

### Test 2: Check Topics
In a new terminal (while simulation is running):
- [ ] `rostopic list` shows expected topics:
  - [ ] `/cmd_vel`
  - [ ] `/base_scan`
  - [ ] `/odom`
  - [ ] `/tf`
- [ ] `rostopic echo /odom` shows data
- [ ] `rostopic echo /base_scan` shows laser data

### Test 3: Run Controller
- [ ] `rosrun ros2 go_to_goal.py` starts without errors
- [ ] Robot begins moving
- [ ] Robot navigates toward target
- [ ] No Python import errors
- [ ] No TF lookup exceptions (after initial startup)

### Test 4: Verify TF Tree
- [ ] `rosrun tf view_frames` generates frames.pdf
- [ ] `/odom` and `/base_footprint` frames present
- [ ] Transform between frames exists

## Optional: Advanced Setup

- [ ] RViz installed and configured
- [ ] Git repository initialized (if contributing)
- [ ] Documentation read and understood
- [ ] Different controllers tested
- [ ] Goal positions modified and tested

## Troubleshooting

If any item fails:

1. **ROS not found**: Reinstall ROS Kinetic following [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md)
2. **Package not found**: Run `rospack profile` and verify sourcing
3. **Build errors**: Check CMakeLists.txt and package.xml files
4. **Import errors**: Verify Python dependencies and script permissions
5. **TF errors**: Ensure Stage is running before controller starts

## Next Steps

Once all items are checked:

1. ✅ Read [docs/QUICK_START.md](docs/QUICK_START.md) for usage examples
2. ✅ Explore [docs/CONTROLLERS.md](docs/CONTROLLERS.md) to understand controllers
3. ✅ Review [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for system design
4. ✅ Experiment with different goal positions
5. ✅ Try different controllers
6. ✅ Modify controller parameters

## Support

If you're stuck:
- Check [BUILD_INSTRUCTIONS.md](BUILD_INSTRUCTIONS.md) troubleshooting section
- Review error messages carefully
- [Open an issue](https://github.com/Shahrukh19S/ros-autonomous-laser-bot/issues) on GitHub with:
  - System information
  - Error messages
  - Steps to reproduce

---

**Status:** ☐ Not Started | ☐ In Progress | ☐ Complete

**Date Completed:** ___________

**Notes:**
_________________________________________________
_________________________________________________
_________________________________________________
