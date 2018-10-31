#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

void clearPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // fetch all objects
  auto objs = psi.getObjects();
  std::vector<std::string> ids;
  transform(objs.begin(), objs.end(), back_inserter(ids), [](auto val) { return val.first; });
  psi.removeCollisionObjects(ids);
}

void detachObjects(ros::NodeHandle nh)
{
  ROS_INFO_STREAM("Detaching Objects...");

  ros::ServiceClient client_get_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient client_apply_scene = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");
  client_apply_scene.waitForExistence();
  client_get_scene.waitForExistence();

  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES;
  client_get_scene.call(srv);
  auto scene = srv.response.scene;
  ROS_INFO_STREAM("current scene: " << scene);

  auto attached_objects = scene.robot_state.attached_collision_objects;
  // auto objects = scene.world.collision_objects;

  moveit_msgs::PlanningScene update;
  update.is_diff = true;
  update.robot_state.is_diff = true;

  for (auto attached_object : attached_objects)
  {
    attached_object.object.operation = attached_object.object.REMOVE;
    update.robot_state.attached_collision_objects.push_back(attached_object);
    update.world.collision_objects.push_back(attached_object.object);
  }
  moveit_msgs::ApplyPlanningScene req;
  req.request.scene = update;
  ROS_INFO_STREAM("sending scene: " << update);
  client_apply_scene.call(req);

  ros::spinOnce();
}

void spawnObject(ros::NodeHandle nh)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  ROS_INFO("Spawning objects...");
  moveit_msgs::CollisionObject o;
  o.id = "object";
  o.header.frame_id = "base_link";
  o.primitive_poses.resize(1);
  o.primitive_poses[0].position.x = 0.5;
  o.primitive_poses[0].position.y = 0.0;
  o.primitive_poses[0].position.z = 0.5;
  o.primitive_poses[0].orientation.w = 1.0;
  o.primitives.resize(1);
  o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  o.primitives[0].dimensions.resize(2);
  o.primitives[0].dimensions[0] = 0.2;
  o.primitives[0].dimensions[1] = 0.02;
  psi.applyCollisionObject(o);

  o.id = "table";
  o.header.frame_id = "base_link";
  o.primitive_poses.resize(1);
  o.primitive_poses[0].position.x = 0.6;
  o.primitive_poses[0].position.y = 0.0;
  o.primitive_poses[0].position.z = 0.36;
  o.primitive_poses[0].orientation.w = 1.0;
  o.primitives.resize(1);
  o.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  o.primitives[0].dimensions.resize(3);
  o.primitives[0].dimensions[0] = 0.4;
  o.primitives[0].dimensions[1] = 1;
  o.primitives[0].dimensions[2] = 0.03;
  psi.applyCollisionObject(o);
}

void plan(Task& t)
{
  std::string tool_frame = "gripper_grasping_frame";
  std::string eef = "gripper";
  std::string arm = "arm_torso";  // arm

  Stage* initial_stage = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  initial_stage = initial.get();
  t.add(std::move(initial));

  // planner used for connect
  auto pipeline = std::make_shared<solvers::PipelinePlanner>();
  pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setProperty("max_ik_solutions", 1u);

  // connect to pick
  stages::Connect::GroupPlannerVector planners = { { eef, pipeline }, { arm, pipeline } };
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
  grasp->setIKFrame(Eigen::Affine3d::Identity(), tool_frame);
  grasp->setProperty("max_ik_solutions", 1u);

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

  // carry
  auto home = std::make_unique<stages::MoveTo>("to home", pipeline);
  home->setProperty("group", "arm_torso");
  home->setProperty("goal", "home");
  t.add(std::move(home));

  try
  {
    if (!t.plan())
    {
      std::cout << "planning failed" << std::endl;
      return;
    }
  }
  catch (const InitStageException& e)
  {
    std::cout << "planning failed with exception" << std::endl << e << t;
  }

  std::cout << "Using Solution:" << std::endl;
  auto solution = t.solutions().front();
  std::cout << solution->comment() << solution->isFailure() << std::endl;

  std::cout << "Execute" << std::endl;
  t.execute(*solution);
  std::cout << "Done" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_mtc_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("~");
  std::string side;

  detachObjects(nh);
  clearPlanningScene();
  spawnObject(nh);

  Task L("nope"), R("right");
  try
  {
    plan(R);

    std::cout << "waiting for any key\n";
    std::cin.ignore();
  }
  catch (const InitStageException& e)
  {
    std::cerr << e;
    return EINVAL;
  }

  return 0;
}
