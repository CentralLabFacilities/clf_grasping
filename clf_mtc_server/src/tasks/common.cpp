#include "clf_mtc_server/tasks/common.h"

#include <moveit/task_constructor/stages/current_state.h>

namespace tasks
{
using namespace moveit::task_constructor;

Task createInitialTask(Stage** initial_out)
{
  Task task("task");

  Stage* initial;
  task.add(Stage::pointer(initial = new stages::CurrentState("current state")));

  if (initial_out)
    *initial_out = initial;
  return task;
}
}  // namespace tasks