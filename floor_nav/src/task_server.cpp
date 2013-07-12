#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <std_srvs/Empty.h>

#include "task_manager_lib/TaskServerDefault.h"


#include "floor_nav/SimTasksEnv.h"


using namespace floor_nav;
using namespace task_manager_lib;

class TaskServer : public TaskServerBase {
    protected: 
    public:
        TaskServer(TaskEnvironmentPtr _env) : TaskServerBase(_env,true) {
            start();
        }

};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"task_server");//init ros
    ros::NodeHandle nh("~");
    TaskEnvironmentPtr env(new SimTasksEnv(nh));
    TaskServer ts(env);
    ros::spin();
    return 0;
}
