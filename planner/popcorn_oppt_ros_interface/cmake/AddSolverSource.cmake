set(SOLVERS_PATH ${CMAKE_CURRENT_LIST_DIR}/../src/solvers)
set(ABT_PATH ${SOLVERS_PATH}/ABT)
set(SOLVER_SRC_PATH ${ABT_PATH}/solver)
include(${ABT_PATH}/robotModel/CMakeLists.txt)

file(GLOB_RECURSE SRCSABSTRACTPROBLEM ${ABT_PATH}/solver/abstract-problem/*.cpp)
file(GLOB_RECURSE SRCSBELIEFESTIMATORS ${ABT_PATH}/solver/belief-estimators/*.cpp)
file(GLOB_RECURSE SRCSOPTIONS ${ABT_PATH}/options/*.c*)
file(GLOB_RECURSE SRCSCHANGES ${ABT_PATH}/solver/changes/*.cpp)
file(GLOB_RECURSE SRCINDEXING ${ABT_PATH}/solver/indexing/*.cpp)
file(GLOB_RECURSE SRCMAPPINGS ${ABT_PATH}/solver/mappings/*.cpp)
file(GLOB_RECURSE SRCSEARCH ${ABT_PATH}/solver/search/*.cpp)
file(GLOB_RECURSE SRCSERIALIZATION ${ABT_PATH}/solver/serialization/*.cpp)

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
