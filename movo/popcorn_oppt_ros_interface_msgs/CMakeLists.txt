cmake_minimum_required(VERSION 2.8.3)
project(popcorn_oppt_ros_interface_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  message_generation
)

add_message_files(
  FILES
  OPPTState.msg
  OPPTObservation.msg
  OPPTJointPositionAction.msg
)

add_action_files(
  DIRECTORY action
  FILES OPPTPlan.action OPPTInitBelief.action OPPTTest.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
