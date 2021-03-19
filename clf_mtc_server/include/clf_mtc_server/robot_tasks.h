#pragma once

#include <ros/ros.h>
#include <moveit/task_constructor/task.h>

#include <clf_grasping_msgs/PickAction.h>
#include <clf_grasping_msgs/PlaceAction.h>

class RobotTasks
{
public:
  virtual void init(ros::NodeHandle&) = 0;
  virtual moveit::task_constructor::Task createPickTask(const std::string& object_id,
                                                        const std::string& support_id) = 0;
  virtual moveit::task_constructor::Task createPlaceTask(const std::string& surface, const std::string& object_id,
                                                         const geometry_msgs::PoseStamped& place_pose) = 0;
};
