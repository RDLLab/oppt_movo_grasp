# - Find Gazebo
# Find Gazebo and it's root include path
#
#   gazebo_FOUND       - True if Gazebo found.
#   gazebo_INCLUDE_DIRS - where to find Gazebo headersz
#   gazebo_LIBRARIES   - Gazebo library.
#

find_package(gazebo REQUIRED)
if(NOT gazebo_FOUND)
   message(FATAL_ERROR "Gazebo could not be found")
endif()

FIND_PATH( GAZEBO_INCLUDE_PATH "gazebo.hh"
           PATH_SUFFIXES "gazebo-${GAZEBO_MAJOR_VERSION}/gazebo" )
if (NOT GAZEBO_INCLUDE_PATH)
    message(FATAL_ERROR "Gazebo header could not be found")
endif()
list(APPEND GAZEBO_INCLUDE_DIRS "${GAZEBO_INCLUDE_PATH}")
message("-- Found Gazebo ${gazebo_VERSION}")
           

