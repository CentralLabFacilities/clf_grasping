cmake_minimum_required(VERSION 2.8.12)
project(clf_grasping_msgs)

set(MSG_DEPS geometry_msgs actionlib_msgs moveit_task_constructor_msgs sensor_msgs moveit_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

# ROS messages, services and actions
#add_message_files(DIRECTORY msg FILES
#)

add_service_files(DIRECTORY srv FILES
	CloudToCollision.srv
)

add_action_files(DIRECTORY action FILES
	PickUpPose.action
	GraspItem.action
	PlanGrasp.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
