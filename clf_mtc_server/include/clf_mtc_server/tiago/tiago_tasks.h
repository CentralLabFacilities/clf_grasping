#pragma once

#include "clf_mtc_server/robot_tasks.h"

class TiagoTasks : public RobotTasks
{
public:
  void init(ros::NodeHandle&);
  moveit::task_constructor::Task createPickTask(std::string id);
  moveit::task_constructor::Task createPlaceTask(std::string surface, geometry_msgs::PoseStamped);
};