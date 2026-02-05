cmake_minimum_required(VERSION 3.0.2)
project(ros2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/go_to_goal.py
  scripts/smooth1.py
  scripts/smooth2.py
  scripts/smooth1_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
