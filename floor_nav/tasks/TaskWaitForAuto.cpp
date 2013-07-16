#include <math.h>
#include "TaskWaitForAuto.h"
#include "floor_nav/TaskWaitForAutoConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForAuto::initialise(const TaskParameters & parameters) 
{
    ROS_INFO("Waiting for automatic control. Press the red button.");
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskWaitForAuto::iterate()
{
    if (!env->getManualControl()) {
        ROS_INFO("Automatic control detected. We are free!");
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForAuto::terminate()
{
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryWaitForAuto);
