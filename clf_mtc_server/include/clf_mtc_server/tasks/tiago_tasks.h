#pragma once

#include "clf_mtc_server/robot_tasks.h"

class TiagoTasks : public RobotTasks
{
public:
  TiagoTasks(const std::string tool_frame, const std::string carry_pose);
  void init(ros::NodeHandle&);

  /** Create a pick task.
   * @param[in] id the planning scene id of the object to pick
   * @param[in] support_id the planning scene id of the surface the object stands on, e.g. "table"
   */
  moveit::task_constructor::Task createPickTask(std::string id, std::string support_id);
  moveit::task_constructor::Task createPlaceTask(std::string surface, geometry_msgs::PoseStamped);
private:
  const std::string tool_frame_;
  const std::string carry_pose_;
};
