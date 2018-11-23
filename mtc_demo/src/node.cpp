#include "mtc_demo/server.h"
#include "mtc_demo/tiago_tasks.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_mtc");

  auto server = new Server(new TiagoTasks());

  return 0;
}
