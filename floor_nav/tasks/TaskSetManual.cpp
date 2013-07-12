#include <math.h>
#include "TaskSetManual.h"
#include "floor_nav/TaskSetManualConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskSetManual::iterate()
{
    env->setManualControl();
	return TaskStatus::TASK_COMPLETED;
}

TaskIndicator TaskSetManual::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactorySetManual);
