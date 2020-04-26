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
  moveit::task_constructor::Task createPickTask(const std::string& id, const std::string& support_id);

  /** Create a place task.
   * @param[in] surface the planning scene id of the surface the object should be placed on, e.g. "table"
   * @param[in] object_id the planning scene id of the object to place
   * @param[in] place_pose where to place the object
   */
  moveit::task_constructor::Task createPlaceTask(const std::string& surface, const std::string& object_id,
                                                 const geometry_msgs::PoseStamped& place_pose);
private:
  const std::string tool_frame_;
  const std::string carry_pose_;
};
