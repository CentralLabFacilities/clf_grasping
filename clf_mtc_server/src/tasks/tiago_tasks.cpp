#include "clf_mtc_server/tasks/tiago_tasks.h"

//#include "clf_mtc_server/stages/generate_all_grasp_pose.h"
#include "clf_mtc_server/stages/grasp_generator_2_0.h"

#include <ros/console.h>
namespace ros { class NodeHandle; }

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
//#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

TiagoTasks::TiagoTasks(const std::string tool_frame, const std::string carry_pose)
  : tool_frame_(tool_frame), carry_pose_(carry_pose)
{
}

void TiagoTasks::init(ros::NodeHandle& /*unused*/)
{
}

Task TiagoTasks::createPickTask(std::string id, std::string support_id)
{
  ROS_INFO("TiagoTasks::createPickTask(id=%s, support_id=%s) called", id.c_str(), support_id.c_str());
  Task t("tiago_grasp");
  std::string eef = "gripper";
  std::string arm = "arm_torso";  // arm
  // If gripper_fingers is not empty, they will be allowed to touch the support surface
  //std::vector<std::string> gripper_fingers {"gripper_left_finger_link", "gripper_right_finger_link"};
  std::vector<std::string> gripper_fingers;

  Stage* initial_stage = nullptr;
  auto initial = std::make_unique<stages::CurrentState>("current state");
  initial_stage = initial.get();
  t.add(std::move(initial));

  auto fix = new stages::FixCollisionObjects();
  fix->setMaxPenetration(0.00);
  geometry_msgs::Vector3 correction;
  correction.z = 1;
  fix->setDirection(correction);
  t.add(Stage::pointer(fix));
  initial_stage = fix;

  // planner used for connect
  auto pipeline = std::make_shared<solvers::PipelinePlanner>();
  pipeline->setPlannerId("RRTConnect");
  pipeline->setProperty("max_ik_solutions", 1u);

  // connect to pick
  stages::Connect::GroupPlannerVector planners = { { eef, pipeline }, { arm, pipeline } };
  auto connect = std::make_unique<stages::Connect>("connect", planners);
  connect->properties().configureInitFrom(Stage::PARENT);
  t.add(std::move(connect));

  // grasp generator
  //auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  auto grasp_generator = std::make_unique<stages::GraspGenerator>("grasp generator");
  //grasp_generator->setAngleDelta(.2);
  grasp_generator->setPreGraspPose("open");
  grasp_generator->setGraspPose("closed");
  grasp_generator->setMonitoredStage(initial_stage);
  std::unique_ptr<stages::SimpleGrasp> grasp;
  if(!gripper_fingers.empty())
  {
    // forward properties from generator's to IK's solution (bottom -> up)
    //grasp_generator->setForwardedProperties({ "object", "eef", "pregrasp", "grasp" });
    // allow inheritance in top -> down fashion as well
    grasp_generator->properties().configureInitFrom(Stage::PARENT, { "object", "eef" });

    auto grasp_generator_wrapper = std::make_unique<SerialContainer>("grasp generator wrapper");
    grasp_generator_wrapper->properties().configureInitFrom(Stage::PARENT | Stage::INTERFACE, {"object", "eef"});
    grasp_generator_wrapper->properties().declare<std::string>("grasp");
    grasp_generator_wrapper->properties().declare<std::string>("pregrasp");
    grasp_generator_wrapper->properties().declare<std::string>("object");
    grasp_generator_wrapper->properties().declare<std::string>("eef");
    grasp_generator_wrapper->properties().declare<std::string>("target_pose");
    //grasp_generator_wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
    grasp_generator_wrapper->insert(std::move(grasp_generator), 0);

    {
      auto allow_gripper_touch_support = new stages::ModifyPlanningScene("allow gripper support collision");
      //allow_gripper_touch_support->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
      allow_gripper_touch_support->setForwardedProperties({"target_pose", "eef", "grasp", "pregrasp" });
      allow_gripper_touch_support->setCallback([support_id, gripper_fingers](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
        ROS_INFO("Allowing collision between support and gripper");
        collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
        for(const std::string& finger : gripper_fingers)
          acm.setEntry(finger, support_id, true);
      });
      grasp_generator_wrapper->insert(Stage::pointer(allow_gripper_touch_support), 0);
    }

    grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator_wrapper));
  }
  else
  {
    grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));
  }
  grasp->setIKFrame(Eigen::Isometry3d::Identity(), tool_frame_);
  grasp->setProperty("max_ik_solutions", 1u);
  //grasp->setProperty("ignore_collisions", true);

  auto pick = std::make_unique<stages::Pick>(std::move(grasp));
  pick->setProperty("eef", eef);
  pick->setProperty("object", id);
  pick->setProperty("eef_frame", tool_frame_);
  geometry_msgs::TwistStamped approach;
  approach.header.frame_id = tool_frame_;
  approach.twist.linear.x = 1.0;
  pick->setApproachMotion(approach, 0.05, 0.15);

  geometry_msgs::TwistStamped lift;
  lift.header.frame_id = "base_link";
  lift.twist.linear.z = 1.0;
  pick->setLiftMotion(lift, 0.03, 0.05);

  {
    auto allow_touch = new stages::ModifyPlanningScene("allow object support collision");
    PropertyMap& p = allow_touch->properties();
    p.declare<std::string>("support");
    p.declare<std::string>("object");
    p.set("support", support_id);
    p.set("object", id);

    allow_touch->setCallback([](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
      collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
      const std::string& support = p.get<std::string>("support");
      const std::string& object = p.get<std::string>("object");
      acm.setEntry(object, support, true);
    });
    pick->insert(Stage::pointer(allow_touch), -2); // insert before the last element
  }

  {
    auto disallow_touch = new stages::ModifyPlanningScene("disallow object support collision");
    PropertyMap& p = disallow_touch->properties();
    p.declare<std::string>("support");
    p.declare<std::string>("object");
    p.set("support", support_id);
    p.set("object", id);

    disallow_touch->setCallback([](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
      collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
      const std::string& support = p.get<std::string>("support");
      const std::string& object = p.get<std::string>("object");
      acm.setEntry(object, support, false);
    });
    pick->insert(Stage::pointer(disallow_touch), -1); // insert at the end (after the last element)
  }

  if(!gripper_fingers.empty())
  {
    auto allow_gripper_touch_support = new stages::ModifyPlanningScene("allow gripper support collision");
    allow_gripper_touch_support->setCallback([support_id, gripper_fingers](const planning_scene::PlanningScenePtr& scene, const PropertyMap& p) {
      ROS_INFO("Allowing collision between support and gripper");
      collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
      for(const std::string& finger : gripper_fingers)
        acm.setEntry(finger, support_id, true);
    });
    pick->insert(Stage::pointer(allow_gripper_touch_support), 0);
  }

  t.add(std::move(pick));

  // carry
  auto home = std::make_unique<stages::MoveTo>("to home", pipeline);
  home->setProperty("group", arm);
  home->setProperty("goal", carry_pose_);
  t.add(std::move(home));

  return t;
}

moveit::task_constructor::Task TiagoTasks::createPlaceTask(std::string /*surface*/,
                                                           geometry_msgs::PoseStamped /*unused*/)
{
  Task t("tiago_place");
  return t;
}
