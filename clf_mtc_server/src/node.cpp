#include "clf_mtc_server/server.h"
#include "clf_mtc_server/tiago/tiago_tasks.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clf_mtc_server");

  ros::NodeHandle nh("clf_mtc_server");

  std::string robot = "tiago";
  nh.getParam("robot", robot);

  if (robot == "tiago")
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
