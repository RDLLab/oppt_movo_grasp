#!/bin/bash

# Get the name of the directory where the repository is located
DIRECTORY=$(cd `dirname $0` && pwd)
CATKIN_WS_PATH="${DIRECTORY}/demo_movo_ws"
# Create a catkin workspace 
mkdir -p ${CATKIN_WS_PATH}/src

# Copy module package into catkin src
ln -s ${DIRECTORY}/movo ${CATKIN_WS_PATH}/src/

# Compile the catkin workspace and install
cd ${CATKIN_WS_PATH}
catkin_make


