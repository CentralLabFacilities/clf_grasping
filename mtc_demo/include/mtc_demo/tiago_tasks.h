#pragma once

#include "mtc_demo/task_constructor.h"

class TiagoTasks : public TaskConstructor
{
public:
  moveit::task_constructor::Task createPickTask(std::string id);
};
