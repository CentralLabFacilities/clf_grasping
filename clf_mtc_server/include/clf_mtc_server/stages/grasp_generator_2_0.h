#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>

namespace moveit {
namespace task_constructor {
namespace stages {

class GraspGenerator : public GeneratePose
{
public:
  GraspGenerator(const std::string& name = "grasp generator");

  void init(const moveit::core::RobotModelConstPtr& robot_model) override;
  void compute() override;

  void setEndEffector(const std::string& eef)
  {
    setProperty("eef", eef);
  }
  void setObject(const std::string& object)
  {
    setProperty("object", object);
  }
  //void setAngleDelta(double delta)
  //{
  //  setProperty("angle_delta", delta);
  //}

  void setPreGraspPose(const std::string& pregrasp)
  {
    properties().set("pregrasp", pregrasp);
  }
  void setPreGraspPose(const moveit_msgs::RobotState& pregrasp)
  {
    properties().set("pregrasp", pregrasp);
  }
  void setGraspPose(const std::string& grasp)
  {
    properties().set("grasp", grasp);
  }
  void setGraspPose(const moveit_msgs::RobotState& grasp)
  {
    properties().set("grasp", grasp);
  }

protected:
  void onNewSolution(const SolutionBase& s) override;
};
}
}
}
