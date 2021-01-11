#ifndef I_MODULE_COMMAND_TASK_MANAGEMENT_H
#define I_MODULE_COMMAND_TASK_MANAGEMENT_H


#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_task_management.h"

#include "i_module_topic_events.h"
#include "i_module_events_task_management.h"

#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class TaskManagementCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS
};

class MACE_CORESHARED_EXPORT IModuleCommandTaskManagement  :
        public AbstractModule_EventListeners<Metadata_TaskManagement, IModuleEventsTaskManagement, TaskManagementCommands>,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandTaskManagement():
        AbstractModule_EventListeners()
    {
        IModuleGenericVehicleListener::SetUp<Metadata_TaskManagement, IModuleEventsTaskManagement, TaskManagementCommands>(this);


    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:


};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_TASK_MANAGEMENT_H
