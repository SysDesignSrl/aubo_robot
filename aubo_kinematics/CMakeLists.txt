cmake_minimum_required(VERSION 2.8.3)
project(aubo_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_core
  moveit_kinematics
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

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
  LIBRARIES
    aubo_i3_kinematic
    aubo_i5_kinematic
    aubo_i10_kinematic
    aubo_i3_moveit_kinematic_plugin
    aubo_i5_moveit_kinematic_plugin
    aubo_i10_moveit_kinematic_plugin
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning pluginlib
    tf_conversions
  DEPENDS Boost
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a C++ library
add_library(aubo_i3_kinematic src/kinematics.cpp)
set_target_properties(aubo_i3_kinematic PROPERTIES COMPILE_DEFINITIONS "AUBO_I3_PARAMS")

## Declare a C++ library
add_library(aubo_i5_kinematic src/kinematics.cpp)
set_target_properties(aubo_i5_kinematic PROPERTIES COMPILE_DEFINITIONS "AUBO_I5_PARAMS")

## Declare a C++ library
add_library(aubo_i10_kinematic src/kinematics.cpp)
set_target_properties(aubo_i10_kinematic PROPERTIES COMPILE_DEFINITIONS "AUBO_I10_PARAMS")


## Declare a C++ library
add_library(aubo_i3_moveit_kinematic_plugin src/aubo_moveit_plugin.cpp)
target_link_libraries(aubo_i3_moveit_kinematic_plugin
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} aubo_i3_kinematic)

## Declare a C++ library
add_library(aubo_i5_moveit_kinematic_plugin src/aubo_moveit_plugin.cpp)
target_link_libraries(aubo_i5_moveit_kinematic_plugin
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} aubo_i5_kinematic)

## Declare a C++ library
add_library(aubo_i10_moveit_kinematic_plugin src/aubo_moveit_plugin.cpp)
target_link_libraries(aubo_i10_moveit_kinematic_plugin
  ${catkin_LIBRARIES} ${Boost_LIBRARIES} aubo_i10_kinematic)


#############
## Install ##
#############

## Mark libraries for installation
install(
  TARGETS aubo_i3_kinematic aubo_i5_kinematic aubo_i10_kinematic
    aubo_i3_moveit_kinematic_plugin aubo_i5_moveit_kinematic_plugin aubo_i10_moveit_kinematic_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  FILES aubo_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
