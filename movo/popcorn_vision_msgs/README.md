# popcorn_vision

## ref
* tabletop object localization
  * http://wiki.ros.org/tabletop_object_detector
  * http://wiki.ros.org/tabletop_objects
  * https://github.com/wg-perception


## Adding a new object to be detected:
The following are steps required to add a new object into the perception 

1. Add the corresponding mesh file in ".ply" format for the object into the meshes directory in the "popcorn_vision" package.
If no good ".ply" file is available, a model can be created in a 3-D modelling software like blender to then generate the ".ply" file.

2. Add the object's dimension details into the popcorn_tabletop_objects.yaml file under the config directory of the "popcorn_vision" package.

3. Modify the source file "tabletop_object_localization.cpp" with the following:
- Under the loadObjects() function, add the name of the new object into the object_names vector. The name added must match its corresponding ".ply" file.
- Verify that object primitive type is considered in TabletopObjectLocalization::getLocalizationResult(), which packs the results into a message structure for the action server.


4. In the appropiate oppt_action_server python script (i.e two_oppt_ros_interface.py), make sure to specify the name of the object to be considered as the target object. 
This is specified in the block of code that looks like:
    obj_str = 'tube@table'
    pickup_plan_success, pickup_result, objs, obstacles = grasper.pickup(obj_str)
    mode = grasper._localization_client.get_mode()
    rospy.loginfo('Localization mode is ' + str(mode))
    using_tracking = mode == 'kinect_tracking' or mode == 'deep_kinect_tracking' or mode == 'deep'
    last_pose = objs[0].primitive_poses[0]


# Upgrade to ROS Kinetic

## MOVEIT Python commander interface
The python moveit_commander interface is broken due to a known dependency bug in Ros Kinetic (https://github.com/ros-planning/moveit/issues/86)
I couldn't fix this even trying with different versions of the dependency package. Rewriting the interface in C++ should fix things.

## KINOVA STARTUP CONFIGURATION FILE (~/movo_ws/src/movo_common/movo_config/movo_config.bash)
* To select the camera to be used:
Modify the configuration file "movo_config.bash". Particularly, the camera configurations section.
These flags indicate how the camera topics are "REMAPPED" into custom movo ros camera topics with predefined ros namespaces in (~/movo_ws/src/movo_robot/movo_bringup/launch/perception/movo_perception.launch).
* The source files for the ROS INDIGO (Ubuntu 14) version didn't had this remapping, and topics were directly associated with the kinect camera names (Therefore some topics and links had to be changed from popcorn source files)
Make sure these remappings are adecuate (there is a commented section that asks for some remapping that might need to be looked at).

## REALSENSE_ROS
The corresponding ROS package for the REAL SENSE CAMERA can be found in https://github.com/IntelRealSense/realsense-ros

## INTEL REALSENSE SOFTWARE
To play around with the camera with intel software run "realsense-viewer"

## CAMERA CONFIGURATION PARAMETERS
The camera configuration parameters for the real sense camera are configured at the startup launch (also in (~/movo_ws/src/movo_robot/movo_bringup/launch/perception/movo_perception.launch))
