#include <moveit_msgs/AttachedCollisionObject.h>

namespace ps
{
std::vector<moveit_msgs::AttachedCollisionObject> getAttachedObjects();
bool detachObjects();
bool clear();
}  // namespace ps
