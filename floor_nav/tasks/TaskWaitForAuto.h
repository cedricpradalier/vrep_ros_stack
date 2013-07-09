#ifndef TASK_WAIT4AUTO_H
#define TASK_WAIT4AUTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskWaitForAutoConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskWaitForAuto : public TaskDefinitionWithConfig<TaskWaitForAutoConfig, TaskWaitForAuto>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskWaitForAuto(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskWaitForAuto() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_WAIT4AUTO_H
