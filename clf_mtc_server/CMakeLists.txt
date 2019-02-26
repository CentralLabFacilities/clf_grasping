cmake_minimum_required(VERSION 2.8.3)
project(clf_mtc_server)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	eigen_conversions
	diagnostic_updater
	std_srvs
	clf_grasping_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mtc_demos
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(clf_mtc_server src/node.cpp src/server.cpp src/tiago/tiago_tasks.cpp)
target_link_libraries(clf_mtc_server ${catkin_LIBRARIES})


install(TARGETS
	clf_mtc_server 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)