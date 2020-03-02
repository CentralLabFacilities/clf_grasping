#include "clf_mtc_server/server.h"
#include "clf_mtc_server/tasks/tiago_tasks.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clf_mtc_server");

  ros::NodeHandle nh("clf_mtc_server");

  std::string robot = "tiago_cupro";
  nh.getParam("robot", robot);

  if (robot == "tiago_cupro")
  {
    ROS_INFO("robot is tiago_cupro");
    Server server(nh, new TiagoTasks("cupro_grasping_frame", "home_carry"));
  }
  else if (robot == "tiago_schunk")
  {
    ROS_INFO("robot is tiago_schunk");
    Server server(nh, new TiagoTasks("gripper_grasping_frame", "home"));
  }
  else
  {
    ROS_ERROR_STREAM("unkown robot type " << robot);
  }

  ros::shutdown();

  return 0;
}
