#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <map>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace moveit::task_constructor;

typedef Task (*TaskCreator)();

void clearPlanningScene()
{
  ROS_INFO_STREAM("Clear Planning Scene...");
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
  // ROS_INFO_STREAM("current scene: " << scene);

  auto attached_objects = scene.robot_state.attached_collision_objects;
  // auto objects = scene.world.collision_objects;

  moveit_msgs::PlanningScene update;
  update.is_diff = 1u;
  update.robot_state.is_diff = 1u;

  for (auto attached_object : attached_objects)
  {
    attached_object.object.operation = attached_object.object.REMOVE;
    update.robot_state.attached_collision_objects.push_back(attached_object);
    update.world.collision_objects.push_back(attached_object.object);
  }
  moveit_msgs::ApplyPlanningScene req;
  req.request.scene = update;
  // ROS_INFO_STREAM("sending scene: " << update);
  client_apply_scene.call(req);

  ros::spinOnce();
}

void spawnObject(ros::NodeHandle /*nh*/)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  ROS_INFO("Spawning objects...");
  moveit_msgs::CollisionObject o;
  o.id = "object";
  o.header.frame_id = "base_link";
  o.primitive_poses.resize(1);
  o.primitive_poses[0].position.x = 0.64;
  o.primitive_poses[0].position.y = 0.0;
  o.primitive_poses[0].position.z = 0.52;
  o.primitive_poses[0].orientation.w = 1.0;
  o.primitives.resize(1);
  o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  o.primitives[0].dimensions.resize(2);
  o.primitives[0].dimensions[0] = 0.18;
  o.primitives[0].dimensions[1] = 0.02;
  psi.applyCollisionObject(o);

  o.id = "table";
  o.header.frame_id = "base_link";
  o.primitive_poses.resize(1);
  o.primitive_poses[0].position.x = 0.78;
  o.primitive_poses[0].position.y = 0.0;
  o.primitive_poses[0].position.z = 0.42;
  o.primitive_poses[0].orientation.w = 1.0;
  o.primitives.resize(1);
  o.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  o.primitives[0].dimensions.resize(3);
  o.primitives[0].dimensions[0] = 0.4;
  o.primitives[0].dimensions[1] = 1;
  o.primitives[0].dimensions[2] = 0.03;
  psi.applyCollisionObject(o);
}

Task createCarry()
{
  ROS_INFO_STREAM("create carry task");
  Task t("task");
  t.loadRobotModel();

  std::string tool_frame = "gripper_grasping_frame";
  std::string eef = "gripper";
  std::string object = "object";
  std::string arm = "arm_torso";  // arm

  // don't spill liquid
  moveit_msgs::Constraints upright_constraint;
  upright_constraint.name = "gripper_grasping_frame:upright";
  upright_constraint.orientation_constraints.resize(1);
  {
    moveit_msgs::OrientationConstraint& c = upright_constraint.orientation_constraints[0];
    c.link_name = "gripper_grasping_frame";
    c.header.frame_id = "base_footprint";
    c.orientation.w = 1.0;
    c.absolute_x_axis_tolerance = 0.65;
    c.absolute_y_axis_tolerance = 0.65;
    c.absolute_z_axis_tolerance = M_PI;
    c.weight = 1.0;
  }

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setProperty("timeout", 10.0);
  sampling_planner->setProperty("max_ik_solutions", 8u);

  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(.3);
  cartesian_planner->setMaxAccelerationScaling(.3);
  cartesian_planner->setStepSize(.002);

  t.setProperty("group", arm);
  t.setProperty("eef", eef);
  t.setProperty("gripper", eef);

  Stage* current_state = nullptr;
  {
    auto initial = std::make_unique<stages::CurrentState>("current state");
    current_state = initial.get();
    t.add(std::move(initial));
  }

  {  // Open gripper first
    auto stage = std::make_unique<stages::MoveTo>("open gripper", sampling_planner);
    stage->setGroup("gripper");
    stage->setGoal("open");
    t.add(std::move(stage));
  }

  {
    stages::Connect::GroupPlannerVector planners = { { eef, sampling_planner }, { arm, sampling_planner } };
    auto stage = std::make_unique<stages::Connect>("move to pre-grasp pose", planners);
    stage->properties().configureInitFrom(Stage::PARENT);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach");
    stage->properties().set("link", tool_frame);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(.10, .20);

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = tool_frame;
    vec.vector.x = 1.0;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::GenerateGraspPose>("grasp work space pose");
    stage->properties().configureInitFrom(Stage::PARENT);
    stage->setPreGraspPose("open");
    stage->setObject(object);
    stage->setAngleDelta(M_PI / 6);
    stage->setMonitoredStage(current_state);

    auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setIKFrame(Eigen::Translation3d(0.05, 0, -.09), tool_frame);
    wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "object" });
    t.add(std::move(wrapper));
  }

  {
    //   auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
    // grasp_generator->setAngleDelta(.2);
    // grasp_generator->setPreGraspPose("open");
    // grasp_generator->setGraspPose("closed");
    // grasp_generator->setMonitoredStage(initial_stage);

    // auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
    // grasp->setIKFrame(Eigen::Affine3d::Identity(), tool_frame);
    // grasp->setProperty("max_ik_solutions", 1u);

    // auto pick = std::make_unique<stages::Pick>(std::move(grasp));
    // pick->setProperty("eef", eef);
    // pick->setProperty("object", std::string("object"));
    // pick->setProperty("eef_frame", tool_frame);
    // geometry_msgs::TwistStamped approach;
    // approach.header.frame_id = tool_frame;
    // approach.twist.linear.x = 1.0;
    // pick->setApproachMotion(approach, 0.05, 0.1);
  }

  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper->object collision");
    stage->allowCollisions(object, t.getRobotModel()->getJointModelGroup(eef)->getLinkModelNamesWithCollisionGeometry(),
                           true);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
    stage->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
    stage->setGoal("closed");
    t.add(std::move(stage));
  }

  Stage* object_grasped = nullptr;
  {
    auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object, tool_frame);
    object_grasped = stage.get();
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setMinMaxDistance(.01, .10);
    stage->setIKFrame(tool_frame);

    stage->properties().set("marker_ns", "lift");

    geometry_msgs::Vector3Stamped vec;
    vec.header.frame_id = "base_footprint";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<stages::MoveTo>("move back", sampling_planner);
    stage->properties().set("group", std::string("arm"));
    stage->properties().set("timeout", 10.0);
    stage->setPathConstraints(upright_constraint);
    // stage->setIKFrame("gripper_grasp_frame");
    // geometry_msgs::PointStamped point;
    // point.header.frame_id= "base_footprint";
    // point.point.x= .5;
    // point.point.y= 0;
    // point.point.z= 1.05;
    // stage->setGoal(point);
    stage->setGoal("transport");
    t.add(std::move(stage));
  }

  t.enableIntrospection();

  return t;
}

Task createPickNoLinear()
{
  Task t("task");

  std::string tool_frame = "gripper_grasping_frame";
  std::string eef = "gripper";
  std::string arm = "arm_torso";  // arm

  Stage* initial_stage = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  initial_stage = initial.get();
  t.add(std::move(initial));

  // planners
  auto pipeline = std::make_shared<solvers::PipelinePlanner>();
  pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setProperty("max_ik_solutions", 4u);

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setProperty("jump_threshold", 0.0);

  {  // Open gripper first
    auto stage = std::make_unique<stages::MoveTo>("open gripper", pipeline);
    stage->setGroup(eef);
    stage->setGoal("open");
    t.add(std::move(stage));
  }

  // connect to pick
  stages::Connect::GroupPlannerVector planners = { { eef, pipeline }, { arm, pipeline } };
  auto connect = std::make_unique<stages::Connect>("connect", planners);
  connect->properties().configureInitFrom(Stage::PARENT);
  t.add(std::move(connect));

  // grasp generator
  auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  grasp_generator->setAngleDelta(.2);
  grasp_generator->setPreGraspPose("open");
  grasp_generator->setGraspPose("closed2");
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
  approach.twist.linear.x = 0.0;
  pick->setApproachMotion(approach, 0.05, 0.1);

  geometry_msgs::TwistStamped lift;
  lift.header.frame_id = "base_link";
  lift.twist.linear.z = 1.0;
  pick->setLiftMotion(lift, 0.03, 0.05);

  t.add(std::move(pick));

  // carry
  auto home = std::make_unique<stages::MoveTo>("to transport", pipeline);
  home->setProperty("group", "arm");
  home->setProperty("goal", "transport");
  t.add(std::move(home));

  return t;
}

Task createPick()
{
  Task t("task");

  std::string tool_frame = "gripper_grasping_frame";
  std::string eef = "gripper";
  std::string arm = "arm_torso";  // arm

  Stage* initial_stage = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  initial_stage = initial.get();
  t.add(std::move(initial));

  // planners
  auto pipeline = std::make_shared<solvers::PipelinePlanner>();
  pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setProperty("max_ik_solutions", 1u);

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setProperty("jump_threshold", 0.0);

  {  // Open gripper first
    auto stage = std::make_unique<stages::MoveTo>("open gripper", pipeline);
    stage->setGroup(eef);
    stage->setGoal("open");
    t.add(std::move(stage));
  }

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
  auto home = std::make_unique<stages::MoveTo>("to transport", pipeline);
  home->setProperty("group", "arm");
  home->setProperty("goal", "transport");
  t.add(std::move(home));

  return t;
}

Task createPickPlace()
{
  Task t("task");

  std::string tool_frame = "gripper_grasping_frame";
  std::string eef = "gripper";
  std::string arm = "arm_torso";  // arm

  Stage* initial_stage = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  initial_stage = initial.get();
  t.add(std::move(initial));

  // planners
  auto pipeline = std::make_shared<solvers::PipelinePlanner>();
  pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setProperty("max_ik_solutions", 1u);

  auto cartesian = std::make_shared<solvers::CartesianPath>();
  cartesian->setProperty("jump_threshold", 0.0);

  {  // Open gripper first
    auto stage = std::make_unique<stages::MoveTo>("open gripper", pipeline);
    stage->setGroup(eef);
    stage->setGoal("open");
    t.add(std::move(stage));
  }

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
  home->setProperty("group", arm);
  home->setProperty("goal", "home");
  t.add(std::move(home));

  {
    // Place on table
    auto place = std::make_unique<stages::MoveTo>("move to place", pipeline);
    place->setProperty("group", arm);
    geometry_msgs::PoseStamped target;
    target.header.frame_id = "base_link";
    target.pose.position.x = 0.5;
    target.pose.position.y = 0;
    target.pose.position.z = 0.51;
    target.pose.orientation.w = 1;
    target.pose.orientation.z = 0;
    place->setGoal(target);
    t.add(std::move(place));

    // auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose", std::move(place));
    // wrapper->setMaxIKSolutions(8);
    // wrapper->setIKFrame(Eigen::Translation3d(0.05, 0, -.09), tool_frame);
    // wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "object" });
    // t.add(std::move(wrapper));
  }

  // release object
  auto pose_generator = new stages::GenerateGraspPose("generate release pose");
  pose_generator->setAngleDelta(.2);
  pose_generator->setPreGraspPose("open");
  pose_generator->setGraspPose("closed");

  auto ungrasp = std::make_unique<stages::SimpleUnGrasp>(std::unique_ptr<MonitoringGenerator>(pose_generator));
  ungrasp->setProperty("object", std::string("object"));
  ungrasp->setProperty("eef", eef);
  ungrasp->remove(-1);  // remove last stage (pose generator)

  // retract right hand
  auto retract = std::make_unique<stages::MoveRelative>("retract", cartesian);
  retract->restrictDirection(stages::MoveRelative::FORWARD);
  retract->setProperty("group", arm);
  retract->setIKFrame(tool_frame);
  retract->setProperty("marker_ns", std::string("retract"));
  geometry_msgs::TwistStamped motion;
  motion.header.frame_id = tool_frame;
  motion.twist.linear.z = -1.0;
  retract->setDirection(motion);
  retract->setProperty("min_distance", 0.05);
  retract->setProperty("max_distance", 0.1);
  ungrasp->insert(std::move(retract), -1);  // insert retract as last stage in ungrasp

  t.add(std::move(ungrasp));

  return t;
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

  std::map<std::string, TaskCreator> tasks;
  tasks["pick_place"] = &createPickPlace;
  tasks["pick"] = &createPick;
  tasks["carry"] = &createCarry;
  tasks["pick_nonlinear"] = &createPickNoLinear;

  while (true)
  {
    std::cout << std::endl << "Tasks: " << std::endl;
    std::cout << " detach; clear; spawn; reset q(quit)" << std::endl << " ";
    for (auto& kv : tasks)
    {
      std::cout << kv.first << "; ";
    }
    std::cout << std::endl << "ENTER TASK NAME: " << std::endl;

    std::string name;
    std::getline(std::cin, name);

    if (name == "q")
    {
      return 0;
    }

    if (name == "reset")
    {
      detachObjects(nh);
      clearPlanningScene();
      spawnObject(nh);
      continue;
    }

    if (name == "detach")
    {
      detachObjects(nh);
      continue;
    }

    if (name == "clear")
    {
      clearPlanningScene();
      continue;
    }

    if (name == "spawn")
    {
      spawnObject(nh);
      continue;
    }

    auto search = tasks.find(name);
    if (search != tasks.end())
    {
      std::cout << "Found " << search->first << " " << (search->second != nullptr) << '\n';
    }
    else
    {
      std::cout << "Not found\n";
      return 1;
    }

    Task t = (search->second)();
    std::cout << std::endl << "Planning Task... " << std::endl;

    try
    {
      if (!t.plan())
      {
        std::cout << "planning failed" << std::endl;
        continue;
      }
    }
    catch (const std::exception& e)
    {
      std::cout << "planning failed with exception:" << std::endl << e.what() << std::endl;
      continue;
    }

    std::cout << "which solution to execute? (-1) to cancel" << std::endl;
    std::getline(std::cin, name);
    int num = stoi(name);
    while (num != -1)
    {
      if (t.solutions().size() - 1 < num)
      {
        std::cout << "Solution:" << num;
        std::cout << " Error: only " << t.solutions().size() << " solutions" << std::endl;
        std::cout << "which solution to execute? (-1) to cancel" << std::endl;
        std::getline(std::cin, name);
        num = stoi(name);
      }
      else
      {
        auto it = t.solutions().begin();
        std::advance(it, num);

        auto solution = *it;
        std::cout << "Using Solution:" << num << std::endl;

        std::cout << "Execute..." << std::endl;
        t.execute(*solution);
        std::cout << "Done" << std::endl;
        num = -1;
      }
    }
  }

  return 0;
}
