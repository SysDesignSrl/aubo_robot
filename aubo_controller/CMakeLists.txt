cmake_minimum_required(VERSION 2.8.3)
project(aubo_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
  urdf
  industrial_msgs
  industrial_utils
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED system filesystem date_time thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs trajectory_msgs control_msgs actionlib
    actionlib_msgs urdf industrial_msgs industrial_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(aubo_controller src/aubo_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(aubo_controller ${catkin_LIBRARIES})

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

## Mark executables for installation
install(TARGETS aubo_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark libraries for installation
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# install(TARGETS aubo_joint_trajectory_action DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#set(${PROJECT_NAME}_SOURCES1
#    src/aubo_robot_simulator.cpp
#)
#ADD_EXECUTABLE(aubo_simulator_controller ${${PROJECT_NAME}_SOURCES1})
#TARGET_LINK_LIBRARIES(aubo_simulator_controller industrial_robot_client ${catkin_LIBRARIES})
#install(TARGETS aubo_simulator_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# catkin_install_python(PROGRAMS script/aubo_controller/aubo_robot_simulator
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
