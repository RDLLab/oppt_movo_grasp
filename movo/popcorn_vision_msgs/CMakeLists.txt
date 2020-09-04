cmake_minimum_required(VERSION 2.8.3)
project(popcorn_vision_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# OpenMP for multi-threaded pcl tracking
#add_compile_options(-fopenmp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  shape_msgs
  moveit_msgs  
  visualization_msgs
  trajectory_msgs
  actionlib
  actionlib_msgs
  message_generation
)
#find_package(PCL REQUIRED)
#add_definitions(${PCL_DEFINITIONS})
#find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES TabletopObjectLocalization.action GetCandyHeight.action
)

add_service_files(
  FILES
  CloudToMesh.srv
  GetPose.srv
  GetMarkers.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
  shape_msgs
  visualization_msgs
)

#add_library(popcorn_vision
#    src/pcl_tools.cpp
#    src/pcl_tracker.cpp
#)
#target_link_libraries(popcorn_vision ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES popcorn_vision
  CATKIN_DEPENDS roscpp rospy message_runtime actionlib_msgs
)

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
#)

#add_executable(tabletop_object_localization
#  src/tabletop_object_localization.cpp
#)

#add_dependencies(tabletop_object_localization popcorn_vision_generate_messages_cpp)

#target_link_libraries(tabletop_object_localization
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  popcorn_vision
#)

#add_executable(pcl_to_mesh_server
#  src/pcl_to_mesh_server.cpp
#)
#add_dependencies(pcl_to_mesh_server popcorn_vision_gencpp)
#target_link_libraries(pcl_to_mesh_server
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  popcorn_vision
#)
