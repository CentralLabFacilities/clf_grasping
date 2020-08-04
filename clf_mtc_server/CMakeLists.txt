cmake_minimum_required(VERSION 2.8.3)
project(clf_mtc_server)

if ("${CMAKE_VERSION}" VERSION_LESS "3.1")
	add_compile_options(-std=c++14)
else ()
	set(CMAKE_CXX_STANDARD 14)
endif ()

find_package(catkin REQUIRED COMPONENTS
	clf_grasping_msgs
	diagnostic_updater
	eigen_conversions
	moveit_msgs
	moveit_ros_planning_interface
	moveit_task_constructor_core
	rviz_marker_tools
	std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mtc_demos
  CATKIN_DEPENDS clf_grasping_msgs moveit_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}
		src/stages/generate_all_grasp_pose.cpp
		src/stages/grasp_generator_2_0.cpp

		src/node.cpp src/server.cpp 
		src/planning_scene.cpp

		src/tasks/common.cpp

    src/tasks/tiago_tasks.cpp
    )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS
	${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch/
	 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
	 
install(DIRECTORY config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
