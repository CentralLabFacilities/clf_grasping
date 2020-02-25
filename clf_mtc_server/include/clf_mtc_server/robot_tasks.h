#pragma once

#include <ros/ros.h>
#include <moveit/task_constructor/task.h>

#include <clf_grasping_msgs/PickAction.h>
#include <clf_grasping_msgs/PlaceAction.h>

class RobotTasks
{
public:
  virtual void init(ros::NodeHandle&) = 0;
  virtual moveit::task_constructor::Task createPickTask(std::string object_id, std::string support_id) = 0;
  virtual moveit::task_constructor::Task createPlaceTask(std::string surface, geometry_msgs::PoseStamped) = 0;
};
