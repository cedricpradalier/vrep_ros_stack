#ifndef TASK_SET_MANUAL_H
#define TASK_SET_MANUAL_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskSetManualConfig.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskSetManual : public TaskInstance<TaskSetManualConfig,SimTasksEnv>
    {
        public:
            TaskSetManual(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskSetManual() {};

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactorySetManual : public TaskDefinition<TaskSetManualConfig, SimTasksEnv, TaskSetManual>
    {

        public:
            TaskFactorySetManual(TaskEnvironmentPtr env) : 
                Parent("SetManual","Set the control mux to manual mode",true,env) {}
            virtual ~TaskFactorySetManual() {};
    };
};

#endif // TASK_SET_MANUAL_H
