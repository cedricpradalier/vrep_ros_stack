#include <math.h>
#include "TaskSetHeading.h"
#include "floor_nav/TaskSetHeadingConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskSetHeading::TaskSetHeading(boost::shared_ptr<TaskEnvironment> tenv) 
    : TaskDefinitionWithConfig<TaskSetHeadingConfig,TaskSetHeading>("SetHeading","Reach a desired heading angle",true,-1.)
{
    env = boost::dynamic_pointer_cast<SimTasksEnv,TaskEnvironment>(tenv);
}

TaskIndicator TaskSetHeading::initialise(const TaskParameters & parameters) throw (InvalidParameter)
{
    TaskIndicator ti = Parent::initialise(parameters);
    if (ti != TaskStatus::TASK_INITIALISED) {
        return ti;
    }
    ROS_INFO("Setting heading to %.2f deg", cfg.target*180./M_PI);
    return ti;
}

TaskIndicator TaskSetHeading::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double alpha = remainder(cfg.target-tpose.theta,2*M_PI);
    if (fabs(alpha) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_theta*alpha;
    if (rot > cfg.max_angular_velocity) rot = cfg.max_angular_velocity;
    if (rot <-cfg.max_angular_velocity) rot =-cfg.max_angular_velocity;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskSetHeading::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskSetHeading);