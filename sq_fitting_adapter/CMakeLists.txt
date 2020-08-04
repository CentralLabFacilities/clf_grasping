cmake_minimum_required(VERSION 2.8.3)
project(sq_fitting_adapter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  clf_grasping_msgs
  geometry_msgs
  moveit_msgs
  rospy
  sensor_msgs
  sq_fitting_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_fitter
  CATKIN_DEPENDS actionlib clf_grasping_msgs geometry_msgs moveit_msgs sensor_msgs sq_fitting_ros #rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/vision_adapter_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
   
install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
