#include <math.h>
#include "TaskWaitForAuto.h"
#include "floor_nav/TaskWaitForAutoConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskWaitForAuto::TaskWaitForAuto(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskWaitForAutoConfig,TaskWaitForAuto>("WaitForAuto","Wait for the control mux to switch to auto",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskWaitForAuto::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator ti = Parent::initialise(parameters);
    if (ti != TaskStatus::TASK_INITIALISED) {
        return ti;
    }
    ROS_INFO("Waiting for automatic control. Press the red button.");
    return ti;
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

DYNAMIC_TASK(TaskWaitForAuto);
