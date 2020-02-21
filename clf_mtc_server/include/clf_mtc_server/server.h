#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <std_srvs/Empty.h>
#include <clf_grasping_msgs/PickAction.h>
#include <clf_grasping_msgs/PlaceAction.h>
#include <clf_grasping_msgs/PlanPickAction.h>
#include <clf_grasping_msgs/PlanPlaceAction.h>

#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/PlanningScene.h>

#include "clf_mtc_server/robot_tasks.h"

typedef std_srvs::Empty::Request ClearPlanningSceneReq;
typedef std_srvs::Empty::Response ClearPlanningSceneRes;

class Server
{
public:
  Server(ros::NodeHandle nh, RobotTasks* tc);

private:
  RobotTasks* tc_;
  ros::NodeHandle nh_;

  // Keep old tasks in list
  void storeTask(moveit::task_constructor::Task& t);
  std::list<moveit::task_constructor::Task> tasks_;
  size_t max_tasks_{ 10 };

  void diagnosticTask(diagnostic_updater::DiagnosticStatusWrapper&);
  diagnostic_updater::Updater diagnostic_;

  // Action servers
  actionlib::SimpleActionServer<clf_grasping_msgs::PickAction> pickAs_;
  void executePick(const clf_grasping_msgs::PickGoalConstPtr& goal);
  clf_grasping_msgs::PickFeedback pickFeedback_;
  clf_grasping_msgs::PickResult pickResult_;

  actionlib::SimpleActionServer<clf_grasping_msgs::PlanPickAction> planPickAs_;
  void executePlanPick(const clf_grasping_msgs::PlanPickGoalConstPtr& goal);

  actionlib::SimpleActionServer<clf_grasping_msgs::PlaceAction> placeAs_;
  void executePlace(const clf_grasping_msgs::PlaceGoalConstPtr& goal);
  clf_grasping_msgs::PlaceFeedback placeFeedback_;
  clf_grasping_msgs::PlaceResult placeResult_;

  actionlib::SimpleActionServer<clf_grasping_msgs::PlanPlaceAction> planPlaceAs_;
  void executePlanPlace(const clf_grasping_msgs::PlanPlaceGoalConstPtr& goal);

  // Services
  bool clearPlanningScene(ClearPlanningSceneReq& req, ClearPlanningSceneRes& res);
  ros::ServiceServer clearPlanningSceneSrv_;
};