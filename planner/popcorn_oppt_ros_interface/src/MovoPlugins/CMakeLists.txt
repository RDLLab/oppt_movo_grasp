#cmake_minimum_required(VERSION 3.5)
#project(MovoPlugins)

#set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++11 -pg")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -std=c++11 -pg")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++11 -DEIGEN_NO_DEBUG")

# RelWithDebInfo is the default build type
#if (NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE RelWithDebInfo) 
#endif()

include(GNUInstallDirs)
find_package(oppt)

# Include the CMakeLists of the different plugins
include(${CMAKE_CURRENT_LIST_DIR}/initialBeliefPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/terminalPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/observationPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/transitionPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/rewardPlugins/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/heuristicPlugins/CMakeLists.txt)

install(FILES ${CMAKE_CURRENT_LIST_DIR}/script/setup.sh DESTINATION ${CMAKE_INSTALL_DATADIR}/${POPCORN_INSTALL_PATH})

