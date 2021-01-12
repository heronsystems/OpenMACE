#ifndef I_MODULE_EVENTS_TASK_GENERATION_H
#define I_MODULE_EVENTS_TASK_GENERATION_H

#include "i_module_events_general.h"
#include "data_tasks/task_loiter_descriptor.h"

namespace MaceCore
{

class IModuleEventsTaskGeneration : public IModuleEventsGeneral
{
    public:

    //!
    //! \brief TaskGeneration_NewTaskList will be used to indicate that new tasks have been generated
    //! \param sender
    //! \param TaskDescriptorList
    //!
    virtual void TaskGeneration_NewTaskList(const ModuleBase *sender, std::list<std::shared_ptr<TaskDescriptor>> &taskList) = 0;



};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_TASK_GENERATION_H
