#include <moveit/task_constructor/task.h>
namespace moveit { namespace task_constructor { class Stage; } }

namespace tasks
{
moveit::task_constructor::Task createInitialTask(moveit::task_constructor::Stage** initial_out);
}  // namespace tasks