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
find_package(Eigen 3.3 REQUIRED)
find_package(OMPL REQUIRED)
find_package(CUDA REQUIRED)

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
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sim_env
  DEPENDS Eigen OMPL
)

###########
## Build ##
###########
set(CMAKE_CXX_COMPILER /usr/bin/clang++)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11)
add_definitions(-Wno-deprecated-register)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include
         ${Eigen_INCLUDE_DIRS}
         ${CUDA_INCLUDE_DIRS}
         ${ompl_INCLUDE_DIRS}
         ${catkin_INCLUDE_DIRS}
         )

set(PLANNER_SRC
        src/mps/planner/ompl/control/RampVelocityControl.cpp
        src/mps/planner/ompl/control/Interfaces.cpp
        src/mps/planner/ompl/control/OracleControlSampler.cpp
        src/mps/planner/ompl/control/NaiveControlSampler.cpp
        src/mps/planner/ompl/control/SimEnvStatePropagator.cpp
        src/mps/planner/ompl/state/SimEnvState.cpp
        src/mps/planner/ompl/state/goal/ObjectRelocationGoal.cpp
        src/mps/planner/ompl/state/Interfaces.cpp
        src/mps/planner/util/Random.cpp
        src/mps/planner/util/Serialize.cpp
        src/mps/planner/util/Logging.cpp
        src/mps/planner/util/Time.cpp
        src/mps/planner/pushing/PushPlannerDistanceMeasure.cpp
        src/mps/planner/pushing/OraclePushPlanner.cpp
        src/mps/planner/pushing/algorithm/RRT.cpp
)
## Declare a C++ library
 add_library(${PROJECT_NAME}
       ${PLANNER_SRC}
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
   ${CUDA_CUBLAS_LIBRARIES}
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
