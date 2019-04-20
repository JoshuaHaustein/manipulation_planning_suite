cmake_minimum_required(VERSION 2.8.3)
project(manipulation_planning_suite)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        cmake_modules
        sim_env
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen 3.2 REQUIRED)
find_package(ompl REQUIRED)
# find_package(Protobuf REQUIRED)

if (ompl_VERSION VERSION_GREATER_EQUAL "1.3.1")
        message("OMPL version is greater than 1.3.1. Setting compatibility flag to new version.")
       add_definitions(-DOMPL_NEW_VERSION)
endif()

add_subdirectory(proto)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
  LIBRARIES ${PROJECT_NAME} proto
  CATKIN_DEPENDS sim_env
  DEPENDS EIGEN OMPL
)

###########
## Build ##
###########
set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD 17)
add_definitions(-std=c++17)
add_definitions(-Wno-deprecated-register)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -DDEBUG_VISUALIZE -DDEBUG_PRINTOUTS")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#message("Proto source: " ${PROTO_SRCS})
#message("Proto header: " ${PROTO_HDRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include
        #  /usr/include/valgrind/
         ${Eigen_INCLUDE_DIRS}
#         ${CUDA_INCLUDE_DIRS}
         ${OMPL_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}
         ${PROTOBUF_INCLUDE_DIRS}
         ${CMAKE_CURRENT_BINARY_DIR}
         )
#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
# message(STATUS "dir='${dir}'")
#endforeach()

## Define all headers (clion needs this)
set(PLANNER_H
        include/mps/planner/ompl/control/Interfaces.h
        include/mps/planner/ompl/control/NaiveControlSampler.h
        include/mps/planner/ompl/control/RampVelocityControl.h
        include/mps/planner/ompl/control/SimEnvStatePropagator.h
        include/mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h
        include/mps/planner/ompl/state/Interfaces.h
        include/mps/planner/ompl/state/SimEnvState.h
        include/mps/planner/ompl/state/goal/ObjectsRelocationGoal.h
        include/mps/planner/ompl/planning/Essentials.h
        include/mps/planner/pushing/OraclePushPlanner.h
        include/mps/planner/pushing/Costs.h
        include/mps/planner/pushing/oracle/OracleControlSampler.h
        include/mps/planner/pushing/oracle/Oracle.h
        include/mps/planner/pushing/oracle/LearnedOracle.h
        include/mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h
        include/mps/planner/pushing/oracle/HumanOracle.h
        # include/mps/planner/pushing/oracle/ElasticBandRampComputer.h
        include/mps/planner/pushing/oracle/RampComputer.h
        include/mps/planner/pushing/oracle/DataGenerator.h
        include/mps/planner/pushing/algorithm/SingleExtendRRT.h
        include/mps/planner/pushing/algorithm/MultiExtendRRT.h
        include/mps/planner/pushing/algorithm/RearrangementPlanner.h
        include/mps/planner/pushing/algorithm/Shortcut.h
        include/mps/planner/sorting/PushSortingPlanner.h
        include/mps/planner/sorting/algorithm/MCTS.h
        include/mps/planner/util/Logging.h
        include/mps/planner/util/Playback.h
        include/mps/planner/util/Serialize.h
        include/mps/planner/util/Random.h
        include/mps/planner/util/Time.h
        include/mps/planner/util/Math.h
        include/mps/planner/util/yaml/OracleParsing.h
        include/mps/planner/util/yaml/SortingParsing.h
        include/mps/sdf/SDF.h
        include/mps/sdf/FMM.h
        )

## Define all source files
set(PLANNER_SRC
        src/mps/planner/ompl/control/RampVelocityControl.cpp
        src/mps/planner/ompl/control/Interfaces.cpp
        src/mps/planner/ompl/control/NaiveControlSampler.cpp
        src/mps/planner/ompl/control/SimEnvStatePropagator.cpp
        src/mps/planner/ompl/state/SimEnvState.cpp
        src/mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.cpp
        src/mps/planner/ompl/state/goal/ObjectsRelocationGoal.cpp
        src/mps/planner/ompl/state/Interfaces.cpp
        src/mps/planner/ompl/planning/Essentials.cpp
        src/mps/planner/util/Random.cpp
        src/mps/planner/util/Serialize.cpp
        src/mps/planner/util/Logging.cpp
        src/mps/planner/util/Time.cpp
        src/mps/planner/util/Playback.cpp
        src/mps/planner/util/Math.cpp
        src/mps/planner/util/yaml/OracleParsing.cpp
        src/mps/planner/util/yaml/SortingParsing.cpp
        src/mps/planner/pushing/OraclePushPlanner.cpp
        src/mps/planner/pushing/Costs.cpp
        src/mps/planner/pushing/oracle/OracleControlSampler.cpp
        src/mps/planner/pushing/oracle/Oracle.cpp
        src/mps/planner/pushing/oracle/LearnedOracle.cpp
        src/mps/planner/pushing/oracle/QuasiStaticSE2Oracle.cpp
        src/mps/planner/pushing/oracle/HumanOracle.cpp
        # src/mps/planner/pushing/oracle/ElasticBandRampComputer.cpp
        src/mps/planner/pushing/oracle/RampComputer.cpp
        src/mps/planner/pushing/oracle/DataGenerator.cpp
        src/mps/planner/pushing/algorithm/SingleExtendRRT.cpp
        src/mps/planner/pushing/algorithm/RearrangementPlanner.cpp
        src/mps/planner/pushing/algorithm/Shortcut.cpp
        src/mps/planner/pushing/algorithm/MultiExtendRRT.cpp
        src/mps/planner/sorting/PushSortingPlanner.cpp
        src/mps/planner/sorting/algorithm/MCTS.cpp
        src/mps/sdf/SDF.cpp
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
         ${PLANNER_SRC}
         ${PLANNER_H}
 )

#target_link_libraries(oracle_planner
#        ${catkin_LIBRARIES}
#        ${CUDA_CUBLAS_LIBRARIES}
#        )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(manipulation_planning_suite ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(manual_test_node test/c++/test_manual.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(manipulation_planning_suite_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(manual_test_node
   ${catkin_LIBRARIES}
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
# install(TARGETS manipulation_planning_suite manipulation_planning_suite_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_manipulation_planning_suite.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
