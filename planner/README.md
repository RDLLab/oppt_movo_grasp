Planner Module : POMDP Simple Grasping Demo
==========================================================================

## Module Description
In this demo, we frame the problem of grasping as a POMDP, Partially Observable Markov's Decision Process, 
and use [OPPT](https://github.com/RDLLab/oppt) ("Online POMDP Planning Toolkit") to generate the policy for grasping.

The Planner module consists of both OPPT and the POMDP model for this demo.


## Content description

### [oppt](oppt)
A copy of the version of "OPPT" used for this demo can be found on "popcorn_oppt_ros_interface/src/oppt" in this module.

### [PlannerNode](popcorn_oppt_ros_interface/src/popcorn_oppt_ros_interface.cpp)
The "popcorn_oppt_ros_interface/src/popcorn_oppt_ros_interface.cpp" is a small ROS server node that allows interfacing to the planner from other modules. 

### [MovoModels](popcorn_oppt_ros_interface/src/MovoModels)
This directory located at "popcorn_oppt_ros_interface/src/MovoModels", contains the environment and agent models as "SDF" files. These are required material for our planner's representation of the environment.

### [MovoPlugins](popcorn_oppt_ros_interface/src/MovoPlugins)
This directory located at "popcorn_oppt_ros_interface/src/MovoPlugins", contains plugins for our Online POMDP Solver used for this demo. Please refer to the [OPPT](https://github.com/RDLLab/oppt) documentation for further information on how these components play a part in our planner.

### [cfg](popcorn_oppt_ros_interface/src/cfg)
For settings and user defined parameters, "OPPT" makes use of a configuration file. The configurations file that is used for this demo 
can be found under the "popcorn_oppt_ros_interface/src/cfg" directory of this module.


## Installation
The following is an example of a step-step installation for the planning component
```bash
# Create workspace directory at a suitable location. This example would use the local Desktop as the example location.
planner@plannerPC:~$ cd ~/Desktop
# Clone the repository
planner@plannerPC:~$ git clone https://github.com/RDLLab/oppt_movo_grasp.git
# Save the repo for easier reading
planner@plannerPC:~$ REPO_ROOT=~/Desktop/oppt_movo_grasp
# Install the required dependencies for oppt
planner@plannerPC:~$ cd ${REPO_ROOT}/oppt
# Execute script to install oppt dependcies. We need ROS for this demo.  
planner@plannerPC:~$ chmod +x install_dependencies.sh && ./install_dependencies --use-ros


# Source ros-kinetic before installing oppt
planner@plannerPC:~$ source /opt/ros/kinetic/setup.sh

# Install oppt
## Build and install OPPT
planner@plannerPC:~$ OPPT_DIR=${REPO_ROOT}/oppt
planner@plannerPC:~$ cd ${OPPT_DIR}/src
planner@plannerPC:~$ mkdir build && cd build

# Default location used is user local, but prefix can be set up to the user
planner@plannerPC:~$ cmake -DCMAKE_INSTALL_PREFIX=<install_dir>
# Install the oppt software
planner@plannerPC:~$ make -j$(nproc) && make install

# Build the catkin workspace for the popcorn oppt ros interface
planner@plannerPC:~$ CATKIN_WS_DIR=${REPO_ROOT}/demo_planner_ws
planner@plannerPC:~$ mkdir -p ${CATKIN_WS_DIR}/src

# Symlink the corresponding module into the src directory
planner@plannerPC:~$ ln -s ${REPO_ROOT}/planner/popcorn_oppt_ros_interface ${CATKIN_WS_DIR}/src/
planner@plannerPC:~$ ln -s ${REPO_ROOT}/planner/popcorn_oppt_ros_interface_msgs ${CATKIN_WS_DIR}/src/

## Go go to catkin_ws
planner@plannerPC:~$ cd ${CATKIN_WS_DIR}

# Install dependencies required for this demo
planner@plannerPC:~$ sudo apt install ros-kinetic-moveit
planner@plannerPC:~$ catkin_make install
```

### PATH TO OPPT INSTALLATION ###
If installing manually, make sure that the *oppt* library can be found from CMake. This would be required for 
proper compilation of the "popcorn_oppt_ros_interface".



## Launching the planning module
```bash
# Source the catkin workspace 
planner@plannerPC:~$ source <PLANNER_MODULE_WS>/devel/setup.bash
# Source the oppt resource path
planner@plannerPC:~$ source <PLANNER_MODULE_WS>/install/share/oppt/setup.bash
# Launch the planner
planner@plannerPC:~$ roslaunch popcorn_oppt_ros_interface planner.launch
```

