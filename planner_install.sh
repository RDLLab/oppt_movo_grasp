#!/bin/bash

# Get the name of the directory where the repository is located
DIRECTORY=$(cd `dirname $0` && pwd)
OPPT_DIR=${DIRECTORY}/planner/oppt

## Go to oppt directory
cd ${OPPT_DIR}

## Runt the install dependencies script
chmod +x install_dependencies.sh && ./install_dependencies.sh --use-ros

# Source ros-kinetic before installing oppt
source /opt/ros/kinetic/setup.sh

## Build and install OPPT
cd ${OPPT_DIR}/src/

# Check if the build folder doesn't exist yet
if [ ! -d "build" ]
then
   mkdir build
fi
cd build
cmake -DCMAKE_INSTALL_PREFIX=${OPPT_DIR}/install ..
make -j$(nproc) && make install

# Define catkin_ws path
CATKIN_WS_PATH="${DIRECTORY}/demo_planner_ws"
# Create a catkin workspace
mkdir -p ${CATKIN_WS_PATH}/src

# Copy module package into catkin src
ln -s ${DIRECTORY}/planner/popcorn_oppt_ros_interface ${CATKIN_WS_PATH}/src/
ln -s ${DIRECTORY}/planner/popcorn_oppt_ros_interface_msgs ${CATKIN_WS_PATH}/src/

# Compile the catkin workspace and install
cd ${CATKIN_WS_PATH}

# Install dependencies required for this demo
sudo apt install ros-kinetic-moveit
catkin_make install





