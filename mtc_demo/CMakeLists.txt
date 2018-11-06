cmake_minimum_required(VERSION 2.8.3)
project(mtc_demo)

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

add_executable(tiago src/tiago.cpp src/server.cpp)
add_executable(mtc_server src/node.cpp src/server.cpp)
target_link_libraries(tiago ${catkin_LIBRARIES})
target_link_libraries(mtc_server ${catkin_LIBRARIES})

install(PROGRAMS
        scripts/spawn_scene.py
        scripts/clear_scene.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
	mtc_server tiago
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
