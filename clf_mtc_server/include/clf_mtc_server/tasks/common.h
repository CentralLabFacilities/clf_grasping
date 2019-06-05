#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>

namespace tasks
{
moveit::task_constructor::Task createInitialTask(moveit::task_constructor::Stage** initial_out);
}  // namespace tasks