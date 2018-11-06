#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <std_srvs/Empty.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <clf_grasping_msgs/GraspItemAction.h>

#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/PlanningScene.h>

typedef std_srvs::Empty::Request ClearPlanningSceneReq;
typedef std_srvs::Empty::Response ClearPlanningSceneRes;

class Server
{
public:
  Server();

private:
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface psi_;

  void diagnosticTask(diagnostic_updater::DiagnosticStatusWrapper&);
  diagnostic_updater::Updater diagnostic_;

  actionlib::SimpleActionServer<clf_grasping_msgs::GraspItemAction> graspItemAs_;
  void executeGraspItem(const clf_grasping_msgs::GraspItemGoalConstPtr& goal);
  clf_grasping_msgs::GraspItemFeedback graspItemFeedback_;
  clf_grasping_msgs::GraspItemResult graspItemResult_;

  bool clearPlanningScene(ClearPlanningSceneReq& req, ClearPlanningSceneRes& res);
  ros::ServiceServer clearPlanningSceneSrv_;

  ros::ServiceClient getSceneClient_;
  ros::ServiceClient applySceneClient_;

  void detachObjects();
  void clearWorldObjects();
};