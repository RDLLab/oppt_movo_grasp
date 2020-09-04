include(${CMAKE_CURRENT_LIST_DIR}/../ABT/robotModel/CMakeLists.txt)
set(SOLVER_SRC_PATH "${CMAKE_CURRENT_LIST_DIR}/../ABT/solver")

file(GLOB_RECURSE SRCSABSTRACTPROBLEM ${SOLVER_SRC_PATH}/abstract-problem/*.cpp)
file(GLOB_RECURSE SRCSBELIEFESTIMATORS ${SOLVER_SRC_PATH}/belief-estimators/*.cpp)
file(GLOB_RECURSE SRCSCHANGES ${SOLVER_SRC_PATH}/changes/*.cpp)
file(GLOB_RECURSE SRCINDEXING ${SOLVER_SRC_PATH}/indexing/*.cpp)
file(GLOB_RECURSE SRCMAPPINGS ${SOLVER_SRC_PATH}/mappings/*.cpp)
file(GLOB_RECURSE SRCSEARCH ${SOLVER_SRC_PATH}/search/*.cpp)
file(GLOB_RECURSE SRCSERIALIZATION ${SOLVER_SRC_PATH}/serialization/*.cpp)

set(ABT_SRC
    ${SRCSABSTRACTPROBLEM}
    ${SRCSBELIEFESTIMATORS}
    ${SRCSOPTIONS}
    ${SRCSCHANGES}  
    ${SRCINDEXING}
    ${SRCMAPPINGS}
    ${SRCSEARCH}
    ${SRCSERIALIZATION}
    ${ROBOT_MODEL_SRC}   
    ${SOLVER_SRC_PATH}/ActionNode.cpp
    ${SOLVER_SRC_PATH}/Agent.cpp 
    ${SOLVER_SRC_PATH}/BeliefNode.cpp
    ${SOLVER_SRC_PATH}/BeliefTree.cpp
    ${SOLVER_SRC_PATH}/Histories.cpp
    ${SOLVER_SRC_PATH}/HistoryEntry.cpp
    ${SOLVER_SRC_PATH}/HistorySequence.cpp
    ${SOLVER_SRC_PATH}/Simulator.cpp
    ${SOLVER_SRC_PATH}/Solver.cpp
    ${SOLVER_SRC_PATH}/StateInfo.cpp
    ${SOLVER_SRC_PATH}/StatePool.cpp)

if(NOT ${BLOCK_SOLVER_COMPILATION}) 
add_executable(multithreadedABT
               ${CMAKE_CURRENT_LIST_DIR}/main_abt.cpp                          
               ${ABT_SRC})
               
target_link_libraries (multithreadedABT                 
                       oppt)
endif() 


