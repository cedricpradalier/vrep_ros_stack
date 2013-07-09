#ifndef TASK_SET_MANUAL_H
#define TASK_SET_MANUAL_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskSetManualConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskSetManual : public TaskDefinitionWithConfig<TaskSetManualConfig, TaskSetManual>
    {

        protected:
            boost::shared_ptr<SimTasksEnv> env;
        public:
            TaskSetManual(boost::shared_ptr<TaskEnvironment> env); 
            virtual ~TaskSetManual() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
};

#endif // TASK_SET_MANUAL_H
