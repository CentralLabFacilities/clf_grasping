#include "clf_mtc_server/server.h"

#include "clf_mtc_server/planning_scene.h"
#include <moveit_msgs/MoveItErrorCodes.h>

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

  // ROS_INFO_STREAM("Waiting for servers to come online...");
  pickAs_.start();
  planPickAs_.start();
  placeAs_.start();
  planPlaceAs_.start();

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

void Server::storeTask(moveit::task_constructor::Task& t)
{
  tasks_.push_back(std::move(t));
  ROS_INFO("tasks_ now contains %lu tasks", tasks_.size());
}

void Server::diagnosticTask(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Status");
}

bool Server::clearPlanningScene(ClearPlanningSceneReq& req, ClearPlanningSceneRes& /*res*/)
{
  ROS_INFO("Server::clearPlanningScene service called");
  if(req.data) {
    ROS_INFO("request.data is true, so not deleting attached objects, only unattached objects");
  } else {
    ROS_INFO("request.data is false, so also deleting attached objects");
  }
  return ((req.data || ps::detachObjects()) && ps::clear());
}

void Server::executePlanPick(const clf_grasping_msgs::PlanPickGoalConstPtr& goal)
{
  clf_grasping_msgs::PlanPickFeedback feedback;
  clf_grasping_msgs::PlanPickResult result;

  auto task = tc_->createPickTask(goal->id, "table");
  ROS_INFO_STREAM("Planning to pick: " << goal->id);

  result.solutions.clear();

  if (!task.plan())
  {
    ROS_ERROR_STREAM("planning failed");
    planPickAs_.setAborted();
    storeTask(task);
  }

  for (auto solution : task.solutions())
  {
    moveit_task_constructor_msgs::Solution msg;
    solution->fillMessage(msg);
    result.solutions.push_back(msg);
  }

  planPickAs_.setSucceeded(result);
  storeTask(task);
}

void Server::executePick(const clf_grasping_msgs::PickGoalConstPtr& goal)
{
  // PickFeedback_;
  auto task = tc_->createPickTask(goal->id, "table");
  ROS_INFO_STREAM("Planning to pick: " << goal->id);

  if (!task.plan(1))
  {
    ROS_ERROR_STREAM("planning failed");
    pickResult_.error_code = -1; // PLANNING_FAILED
    pickResult_.result.result = pickResult_.result.NO_PLAN_FOUND;
    pickAs_.setAborted(pickResult_, "planning failed");
    storeTask(task);
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
  if(std::is_same<decltype(task.execute(*solution)), void>::value) { // in older mtc versions, Task::execute returns void
    task.execute(*solution);
    ROS_INFO("Task::execute has return type void, so cannot check result. Assuming success.");
    pickResult_.error_code = 1; // SUCCESS
  } else {
    moveit_msgs::MoveItErrorCodes execute_result = task.execute(*solution);
    ROS_INFO("Task::execute returned %i", execute_result.val);
    pickResult_.error_code = execute_result.val;
  }

  ROS_INFO_STREAM("Done!");
  if(pickResult_.error_code == 1) { // SUCCESS
    pickResult_.result.result = pickResult_.result.SUCCESS;
    pickAs_.setSucceeded(pickResult_, "success");
  } else {
    pickResult_.result.result = pickResult_.result.EXECUTION_FAILED;
    pickAs_.setAborted(pickResult_, "execution failed");
  }
  storeTask(task);
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
    storeTask(task);
    return;
  }

  for (auto solution : task.solutions())
  {
    moveit_task_constructor_msgs::Solution msg;
    solution->fillMessage(msg);
    result.solutions.push_back(msg);
  }

  planPlaceAs_.setSucceeded(result);
  storeTask(task);
}

void Server::executePlace(const clf_grasping_msgs::PlaceGoalConstPtr& goal)
{
  // PickFeedback_;
  auto task = tc_->createPlaceTask(goal->surface, goal->place_pose);
  ROS_INFO_STREAM("Planning to place: ");  // << goal->id);

  if (!task.plan(1))
  {
    ROS_ERROR_STREAM("planning failed");
    placeAs_.setAborted();
    storeTask(task);
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
  storeTask(task);
}