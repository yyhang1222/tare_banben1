cmake_minimum_required(VERSION 3.0.2)
project(env_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# scripts are installed as executables
catkin_install_python(PROGRAMS
  scripts/env_publisher.py
  scripts/env_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
