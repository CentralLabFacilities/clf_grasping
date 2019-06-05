#include "clf_mtc_server/server.h"
#include "clf_mtc_server/tasks/tiago_tasks_cupro.h"
#include "clf_mtc_server/tasks/tiago_tasks.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clf_mtc_server");

  ros::NodeHandle nh("clf_mtc_server");

  std::string robot = "tiago_cupro";
  nh.getParam("robot", robot);

  if (robot == "tiago_cupro")
  {
    Server server(nh, new TiagoTasksCupro());
  }
  else if (robot == "tiago_schunk")
  {
    Server server(nh, new TiagoTasks());
  }
  else
  {
    ROS_ERROR_STREAM("unkown robot type " << robot);
  }

  ros::shutdown();

  return 0;
}
