MOVO Module : POMDP Simple Grasping Demo
==========================================================================

## Module Description
The MOVO module contains every component that is executed on the Kinova MOVO robot. The content here should be installed into "movo2" pc of 
the mobile manipulator.


### Movo manipulator configurations
To make sure that the configurations of the MOVO used for the demo match the ones we use, see our MOVO configuration file [here](https://github.com/RDLLab/oppt_movo_grasp/blob/master/movo/movo_config.bash).


## Content description

### Movo module components:
All of our components in this module are located inside ROS packages named popcorn_(NAME OF PACKAGE). These include

### [popcorn grasper](popcorn_grasper)
A system that provides estimates on whether the gripper has completed a grasp of an object or not.

### [popcorn vision](popcorn_vision)
A system that provides computation of the pose information for the object to be grasped.
Please see the "VisionReadme" file [here](../guides/VisionReadme.md) for using your custom vision detection system.

### [popcorn manager](popcorn_manager)
- Demo main: The accumulator of components and entry point for the demo.
- MovoPlannerInterface: A ROS client interface between the "Planner module" and the "MOVO module" to query the next instructions from the planner.
- MovoControllerInterface: A component to query states of the robot and execute control commands.
- MovoVisionInterface: A local interface to the popcorn vision module.



## Installation
The following is an example of a step-step installation for the movo component
```bash
# Create workspace directory at a suitable location. This example would use the local Desktop as the example location.
movo@movo2:~$ cd ~/Desktop

# Clone the repository
movo@movo2:~$ git clone https://github.com/RDLLab/oppt_movo_grasp.git

# Create a catkin workspace and build the appropiate module in the system
movo@movo2:~$ cd ~/Desktop/oppt_movo_grasp
movo@movo2:~$ mkdir -p demo_movo_catkin_ws/src

# Copy or symlink the corresponding module into the src directory
movo@movo2:~$ ln -s ~/Desktop/oppt_movo_grasp/movo ~/Desktop/oppt_movo_grasp/demo_movo_catkin_ws/src/

# Compile the catkin workspace
movo@movo2:~$ cd ~/Desktop/oppt_movo_grasp/demo_movo_catkin_ws
movo@movo2:~$ catkin_make install
```




## Launching the movo module

```bash
# Source the catkin_ws setup file
movo@movo2:~$ source <MOVO_MODULE_CATKIN_WS>/devel/setup.bash
```

- Run launchfile to initialize sensors of the robot for the demo.

``` bash
# Run this version if using the default vision module
movo@movo2:~$ roslaunch popcorn_grasper demo.launch static_table:={true,false}  

# Run this version if using your own vision module. Make sure you read the documentation on  
# the vision directory for more details on how to do it.
movo@movo2:~$ roslaunch popcorn_grasper demo.launch use_default_vision:=false static_table:={true,false}
```  

- Run launchfile to initialize demo execution.  
``` bash
movo@movo2:~$ roslaunch popcorn_manager start_demo.launch
```





