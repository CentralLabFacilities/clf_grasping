#include "mtc_demo/server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_mtc");

  auto server = new Server();

  return 0;
}
