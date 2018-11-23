#pragma once

#include <moveit/task_constructor/task.h>

class TaskConstructor
{
public:
  virtual moveit::task_constructor::Task createPickTask(std::string id) = 0;
};
