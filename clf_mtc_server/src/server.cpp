#include "clf_mtc_server/server.h"

Server::Server(ros::NodeHandle nh, RobotTasks* tc)
  : nh_(nh)
  , diagnostic_(nh_)
  , pickAs_(nh_, "pick_object", boost::bind(&Server::executePick, this, _1), false)
  , planPickAs_(nh_, "plan_pick", boost::bind(&Server::executePlanPick, this, _1), false)
  , placeAs_(nh_, "place_object", boost::bind(&Server::executePlace, this, _1), false)
  , planPlaceAs_(nh_, "plan_place", boost::bind(&Server::executePlanPlace, this, _1), false)
{
  tc_ = tc;

  // wait until time ROS_TIME is initialized
  ros::Duration d(0.50);
  d.sleep();

  clearPlanningSceneSrv_ = nh_.advertiseService("clear_planning_scene", &Server::clearPlanningScene, this);

  getSceneClient_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  applySceneClient_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("/apply_planning_scene");

  // ROS_INFO_STREAM("Waiting for servers to come online...");
  pickAs_.start();
  planPickAs_.start();
  placeAs_.start();
  planPlaceAs_.start();
  ROS_INFO_STREAM("Waiting for moveit servers to come online...");
  getSceneClient_.waitForExistence();
  applySceneClient_.waitForExistence();

  ROS_INFO_STREAM("Init Robot Tasks...");
  tc->init(nh_);

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

  // to detach we have to also remove the removed objects from world?
  // http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_ros_api_tutorial.html#detach-an-object-from-the-robot
  // yes, "planning_scene.world.collision_objects.push_back(attached_object.object);" still uses the remove operation
  // it does not detach and add to the world without the remove operation
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

void Server::executePlanPick(const clf_grasping_msgs::PlanPickGoalConstPtr& goal)
{
  clf_grasping_msgs::PlanPickFeedback feedback;
  clf_grasping_msgs::PlanPickResult result;

  auto task = tc_->createPickTask(goal->id);
  ROS_INFO_STREAM("Planning to pick: " << goal->id);

  result.solutions.clear();

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    planPickAs_.setAborted();
    return;
  }

  for (auto solution : task.solutions())
  {
    moveit_task_constructor_msgs::Solution msg;
    solution->fillMessage(msg);
    result.solutions.push_back(msg);
  }

  planPickAs_.setSucceeded(result);
}

void Server::executePick(const clf_grasping_msgs::PickGoalConstPtr& goal)
{
  // PickFeedback_;
  auto task = tc_->createPickTask(goal->id);
  ROS_INFO_STREAM("Planning to pick: " << goal->id);

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    pickAs_.setAborted();
    return;
  }

  // ROS_INFO_STREAM( "Using Solution:" << std::endl <<
  auto solution = task.solutions().front();
  // std::cout << solution->comment() << solution->isFailure() << std::endl;

  if (pickAs_.isPreemptRequested() || !ros::ok())
  {
    pickAs_.setPreempted();
    return;
  }

  ROS_INFO_STREAM("Execute Solution...");
  task.execute(*solution);

  ROS_INFO_STREAM("Done!");
  pickAs_.setSucceeded(pickResult_);
}

void Server::executePlanPlace(const clf_grasping_msgs::PlanPlaceGoalConstPtr& goal)
{
  clf_grasping_msgs::PlanPlaceFeedback feedback;
  clf_grasping_msgs::PlanPlaceResult result;

  auto task = tc_->createPlaceTask(goal->surface, goal->place_pose);
  ROS_INFO_STREAM("Planning to place: ");  // << goal->id);

  result.solutions.clear();

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    planPlaceAs_.setAborted();
    return;
  }

  for (auto solution : task.solutions())
  {
    moveit_task_constructor_msgs::Solution msg;
    solution->fillMessage(msg);
    result.solutions.push_back(msg);
  }

  planPlaceAs_.setSucceeded(result);
}

void Server::executePlace(const clf_grasping_msgs::PlaceGoalConstPtr& goal)
{
  // PickFeedback_;
  auto task = tc_->createPlaceTask(goal->surface, goal->place_pose);
  ROS_INFO_STREAM("Planning to place: ");  // << goal->id);

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    placeAs_.setAborted();
    return;
  }

  // ROS_INFO_STREAM( "Using Solution:" << std::endl <<
  auto solution = task.solutions().front();
  // std::cout << solution->comment() << solution->isFailure() << std::endl;

  if (placeAs_.isPreemptRequested() || !ros::ok())
  {
    placeAs_.setPreempted();
    return;
  }

  ROS_INFO_STREAM("Execute Solution...");
  task.execute(*solution);

  ROS_INFO_STREAM("Done!");
  placeAs_.setSucceeded(placeResult_);
}