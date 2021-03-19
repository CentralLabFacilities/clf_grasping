cmake_minimum_required(VERSION 2.8.3)
project(mtc_demo)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
	clf_grasping_msgs
	diagnostic_updater
	eigen_conversions
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mtc_demos
  CATKIN_DEPENDS clf_grasping_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tiago src/stages/place_generator.hpp src/stages/grasp_generator.hpp src/tasks/clf_mtc_server_tiago.hpp src/tiago.cpp)
target_link_libraries(tiago ${catkin_LIBRARIES})

install(PROGRAMS
        scripts/clear_scene.py
        scripts/spawn_scene_cylinder.py
        scripts/spawn_scene_tea.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
	tiago
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
