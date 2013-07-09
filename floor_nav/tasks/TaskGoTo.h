#ifndef TASK_GOTO_H
#define TASK_GOTO_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskGoToConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskGoTo : public TaskDefinitionWithConfig<TaskGoToConfig, TaskGoTo>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskGoTo(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskGoTo() {};

            virtual TaskIndicator initialise(const TaskParameters & parameters) throw (InvalidParameter);

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_GOTO_H
