#
# Personal Robotics Laboratory
# Catkin/Rosbuild Switching Template
#
# This CMakeLists.txt file is designed to correctly switch two CMakeLists,
# one for rosbuild and one for catkin.
#
# To use this file, create two CMakeLists:
#   - catkin.cmake : the CMakeLists used in ROS catkin builds
#   - rosbuild.cmake : the CMakeLists used for ROS rosbuild builds
#
cmake_minimum_required(VERSION 2.8.3)
project(block_sorting)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/run_demo.py
    DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

