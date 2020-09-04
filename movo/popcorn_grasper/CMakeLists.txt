cmake_minimum_required(VERSION 2.8.3)
project(popcorn_grasper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    actionlib
    std_msgs
    actionlib_msgs
    moveit_msgs
)
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_python_setup()


generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  moveit_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_executable(primitive_shape_grasp_planner
#  src/primitive_shape_grasp_planner.cpp
#)

#add_dependencies(primitive_shape_grasp_planner popcorn_grasper_generate_messages_cpp)

#target_link_libraries(primitive_shape_grasp_planner
#  ${catkin_LIBRARIES}
#)
