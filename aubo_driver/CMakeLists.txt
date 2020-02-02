cmake_minimum_required(VERSION 2.8.3)
project(aubo_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  trajectory_msgs
  control_msgs
  actionlib
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
#find_package(python-numpy REQUIRED)

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
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs trajectory_msgs control_msgs
    actionlib actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/lib
  include/lib/otg
  ${catkin_INCLUDE_DIRS})

link_directories(
  lib/lib64
  lib/lib64/log4cplus
  lib/lib64/config
  lib/lib64/aubocontroller
  ${catkin_LIB_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/moving_cell.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(aubo_driver src/driver_node.cpp src/aubo_robot.cpp)

link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/lib)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(aubo_driver pthread auborobotcontroller otgLib ${catkin_LIBRARIES})


# if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#         link_directories(${catkin_LIB_DIRS}  lib/lib64 lib/lib64/log4cplus lib/lib64/config lib/lib64/aubocontroller)
# else()
#         link_directories(${catkin_LIB_DIRS}  lib/lib32)
# endif()
#
# ## Declare a C++ executable
# set(${PROJECT_NAME}_SOURCES1
#     src/driver_node.cpp
# )
# set(${PROJECT_NAME}_SOURCES2
#     src/testIO.cpp
# )
# set(${PROJECT_NAME}_SOURCES3
#     src/testAuboAPI.cpp
# )
#
# add_executable(aubo_driver ${${PROJECT_NAME}_SOURCES1})
# add_executable(testIO ${${PROJECT_NAME}_SOURCES2})
# add_executable(testAuboAPI ${${PROJECT_NAME}_SOURCES3})
# add_executable(aubo_gazebo_driver src/aubo_gazebo_driver.cpp)
#
# #LINK_LIBRARIES(${CMAKE_CURRENT_SOURCE_DIR}/lib)
#
# target_link_libraries(aubo_driver pthread ${catkin_LIBRARIES} auborobotcontroller otgLib)
# target_link_libraries(testAuboAPI ${catkin_LIBRARIES} auborobotcontroller otgLib)
# target_link_libraries(testIO ${catkin_LIBRARIES} otgLib)
# target_link_libraries(aubo_gazebo_driver ${catkin_LIBRARIES})
#
#
#
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#
# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
#  )
#
# add_dependencies(aubo_driver aubo_msgs_gencpp)
# add_dependencies(testIO aubo_msgs_gencpp)
# add_dependencies(testAuboAPI aubo_msgs_gencpp)
