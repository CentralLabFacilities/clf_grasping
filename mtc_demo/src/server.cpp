#include "mtc_demo/server.h"

Server::Server(TaskConstructor* tc)
  : nh_("mtc_server")
  , diagnostic_(nh_)
  , graspItemAs_(nh_, "grasp_object", boost::bind(&Server::executeGraspItem, this, _1), false)
  , planGraspAs_(nh_, "plan_grasp", boost::bind(&Server::executePlanGrasp, this, _1), false)
{
  tc_ = tc;

  // wait until time ROS_TIME is initialized
  ros::Duration d(0.50);
  d.sleep();

  clearPlanningSceneSrv_ = nh_.advertiseService("clear_planning_scene", &Server::clearPlanningScene, this);

  getSceneClient_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  applySceneClient_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");

  ROS_INFO_STREAM("Waiting for servers to come online...");
  graspItemAs_.start();
  planGraspAs_.start();
  getSceneClient_.waitForExistence();
  applySceneClient_.waitForExistence();

  diagnostic_.setHardwareID("none");
  diagnostic_.add("MTC Status", this, &Server::diagnosticTask);
  diagnostic_.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "OK");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate r(100);
  ROS_INFO_STREAM("Server started!");
  while (ros::ok())
  {
    diagnostic_.update();
    r.sleep();
  }
  spinner.stop();
}

void Server::diagnosticTask(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status");
}

bool Server::clearPlanningScene(ClearPlanningSceneReq& /*req*/, ClearPlanningSceneRes& /*res*/)
{
  detachObjects();
  clearWorldObjects();
  return true;
}

void Server::detachObjects()
{
  moveit_msgs::GetPlanningScene srv;
  srv.request.components.components =
      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES;
  getSceneClient_.call(srv);
  auto scene = srv.response.scene;
  ROS_DEBUG_STREAM("current scene: " << scene);

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
  ROS_DEBUG_STREAM("sending scene: " << update);
  applySceneClient_.call(req);
}

void Server::clearWorldObjects()
{
  // fetch all objects
  auto objs = psi_.getObjects();
  std::vector<std::string> ids;
  transform(objs.begin(), objs.end(), back_inserter(ids), [](auto val) { return val.first; });
  psi_.removeCollisionObjects(ids);
}

void Server::executePlanGrasp(const clf_grasping_msgs::PlanGraspGoalConstPtr& goal)
{
  clf_grasping_msgs::PlanGraspFeedback feedback;
  clf_grasping_msgs::PlanGraspResult result;

  auto task = tc_->createPickTask(goal->id);
  ROS_INFO_STREAM("Planning to grasp: " << goal->id);

  result.solutions.clear();

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    planGraspAs_.setAborted();
    return;
  }

  for (auto solution : task.solutions())
  {
    moveit_task_constructor_msgs::Solution msg;
    solution->fillMessage(msg);
    result.solutions.push_back(msg);
  }

  planGraspAs_.setSucceeded(result);
}

void Server::executeGraspItem(const clf_grasping_msgs::GraspItemGoalConstPtr& goal)
{
  // graspItemFeedback_;
  auto task = tc_->createPickTask(goal->id);
  ROS_INFO_STREAM("Planning to grasp: " << goal->id);

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    graspItemAs_.setAborted();
    return;
  }

  // ROS_INFO_STREAM( "Using Solution:" << std::endl <<
  auto solution = task.solutions().front();
  // std::cout << solution->comment() << solution->isFailure() << std::endl;

  if (graspItemAs_.isPreemptRequested() || !ros::ok())
  {
    graspItemAs_.setPreempted();
    return;
  }

  ROS_INFO_STREAM("Execute Solution...");
  task.execute(*solution);

  ROS_INFO_STREAM("Done!");
  graspItemAs_.setSucceeded(graspItemResult_);
}