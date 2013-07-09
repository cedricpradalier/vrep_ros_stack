#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <std_srvs/Empty.h>

#include "task_manager_lib/SequenceTask.h"
#include "task_manager_lib/TaskScheduler.h"

#include "task_manager_lib/DynamicTask.h"
#include "task_manager_lib/TaskIdleDefault.h"
#include "task_manager_lib/TaskWaitDefault.h"

// #include "task_manager_lib/TaskServerInterface.h"


#include "floor_nav/SimTasksEnv.h"


using namespace floor_nav;
using namespace task_manager_lib;


class TaskServer {
    protected:
        ros::NodeHandle nh;
        ros::ServiceServer service;
        std::string lib_path;
        boost::shared_ptr<TaskEnvironment> env;
        boost::shared_ptr<TaskDefinition> idle;
        boost::shared_ptr<TaskDefinition> wait;
        TaskScheduler ts;

        void reloadTasks() {
            ROS_INFO("Terminating all tasks");
            ts.terminateAllTasks();
            ts.stopScheduler();
            ROS_INFO("Clearing all dynamic tasks");
            ts.clearAllDynamicTasks();
            ROS_INFO("Reloading all dynamic tasks");
            ts.loadAllTasks(lib_path,env);
            ts.configureTasks();
            ts.printTaskDirectory(true);
            ts.startScheduler();
            ROS_INFO("Reload completed");
        }

        bool reloadSrv(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res)
        {
            reloadTasks();
            return true;
        }

    public:
        TaskServer() : nh("~"), lib_path("./lib"), env(new SimTasksEnv(nh)), idle(new TaskIdleDefault(env)), wait(new TaskWaitDefault(env)), ts(nh, idle, 0.5) {
            nh.getParam("lib_path",lib_path);
            service = nh.advertiseService("reload_tasks", &TaskServer::reloadSrv,this);
            ts.addTask(wait);
            ts.loadAllTasks(lib_path,env);
            ts.configureTasks();
            ts.printTaskDirectory(true);
            ts.startScheduler();
        }

};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task_server");//init ros
    TaskServer ts;
    ros::spin();
    return 0;
}
