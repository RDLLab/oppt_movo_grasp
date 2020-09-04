cmake_minimum_required(VERSION 2.8.3)
project(popcorn_oppt_ros_interface)
include(FindPkgConfig)
set(CMAKE_CXX_STANDARD 11)

# Set catkin-ws root
set(CATKIN_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(OPPT_INSTALL_PATH ${CATKIN_ROOT_PATH}/../planner/oppt/install)
message(STATUS "OPPT INSTALL PATH IS" : ${OPPT_INSTALL_PATH})
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED roscpp genmsg actionlib_msgs actionlib geometry_msgs popcorn_oppt_ros_interface_msgs)
find_package(catkin REQUIRED roscpp actionlib_msgs actionlib popcorn_oppt_ros_interface_msgs)
find_package(oppt REQUIRED PATHS ${OPPT_INSTALL_PATH} NO_DEFAULT_PATH)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES popcorn_oppt_ros_interface
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS popcorn_oppt_ros_interface_msgs
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(${catkin_INCLUDE_DIRS})
#include_directories(include)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

## Check if we can use IK engine
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckIKSupport.cmake)

## Include the CMakeLists that adds the solver source files to the ABT_SRC variable
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddSolverSource.cmake)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/plugins/transitionPlugins/MovoTransitionPlugins/IKSolutions/CMakeLists.txt" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/plugins/transitionPlugins/MovoTransitionPlugins/PRM/CMakeLists.txt")
   add_definitions(-DENABLE_MACRO_ACTIONS)
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(popcorn_oppt_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Build popcorn_oppt_ros_interface_node
add_executable(popcorn_oppt_ros_interface_node 
               src/popcorn_oppt_ros_interface.cpp               
               src/MachineState.cpp
               ${IK_SRC}
               ${PRM_SRC}
               ${ABT_SRC})

target_include_directories(popcorn_oppt_ros_interface_node
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/MovoPlugins/shared>
        ${SOLVERS_PATH}
        ${catkin_INCLUDE_DIRS}
        oppt)
        
##target_compile_features(popcorn_oppt_ros_interface_node PUBLIC cxx_std_11)

add_dependencies(popcorn_oppt_ros_interface_node popcorn_oppt_ros_interface_msgs_generate_messages_cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(popcorn_oppt_ros_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(popcorn_oppt_ros_interface_node
   ${catkin_LIBRARIES}
   ${oppt_LIBRARIES}
   ${FLANN_LIBRARIES} 
   ${OMPL_LIBRARIES}  
   orocos-kdl
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS popcorn_oppt_ros_interface popcorn_oppt_ros_interface_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

set(POPCORN_INSTALL_PATH "oppt")

include(${CMAKE_CURRENT_LIST_DIR}/src/MovoPlugins/CMakeLists.txt)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/src/MovoModels DESTINATION ${CMAKE_INSTALL_DATADIR}/${POPCORN_INSTALL_PATH})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_popcorn_oppt_ros_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
