#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "base_link";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.5;
	o.primitive_poses[0].position.y = 0.0;
	o.primitive_poses[0].position.z = 0.5;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.02;
	psi.applyCollisionObject(o);
}

void plan(Task &t) {
	spawnObject();

	std::string tool_frame = "gripper_grasping_frame";
	std::string eef = "gripper";
	std::string arm = "arm_torso"; //arm

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial_stage = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("closed");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
	grasp->setIKFrame(Eigen::Translation3d(0,0, -.03)*
	                  Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()),
	                  tool_frame);
	grasp->setProperty("max_ik_solutions", 4u);

	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->setProperty("eef", eef);
	pick->setProperty("object", std::string("object"));
	pick->setProperty("eef_frame", tool_frame);
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.x = 1.0;
	pick->setApproachMotion(approach, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "base_link";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	t.add(std::move(pick));



	try {
		if(!t.plan()) std::cout << "planning failed" << std::endl;
	} catch (const InitStageException &e) {
		std::cout << "planning failed with exception" << std::endl << e << t;
	}

}

int main(int argc, char** argv){
	ros::init(argc, argv, "tiago");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");
	std::string side;
	
	Task L("nope"), R("right");
	try {
		
		plan(R);
		
		std::cout << "waiting for any key\n";
		std::cin.ignore();
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	return 0;
}
