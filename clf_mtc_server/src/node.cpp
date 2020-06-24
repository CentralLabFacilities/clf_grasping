#include "clf_mtc_server/server.h"
#include "clf_mtc_server/tasks/tiago_tasks.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clf_mtc_server");

  ros::NodeHandle nh("clf_mtc_server");

  std::string robot;
  if(!nh.getParam("robot", robot))
  {
    ROS_ERROR("Unable to retrieve parameter: robot");
  }

  if (robot == "tiago_cupro")
  {
    ROS_INFO("robot is tiago_cupro");
    Server server(nh, new TiagoTasks("cupro_grasping_frame", "home_carry"));
  }
  else if (robot == "tiago_schunk")
  {
    ROS_INFO("robot is tiago_schunk");
    Server server(nh, new TiagoTasks("gripper_grasping_frame", "home_carry"));
  }
  else
  {
    ROS_ERROR_STREAM("unknown robot type " << robot << ", possible are tiago_cupro, tiago_schunk");
  }

  ros::shutdown();

  return 0;
}
