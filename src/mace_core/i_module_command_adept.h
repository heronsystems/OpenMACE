#ifndef I_MODULE_COMMAND_ADEPT_H
#define I_MODULE_COMMAND_ADEPT_H
#include "abstract_module_event_listeners.h"
#include "metadata_adept.h"

#include "i_module_topic_events.h"
#include "i_module_events_adept.h"
#include "i_module_events_vehicle.h"

#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class AdeptCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS
};

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandAdept :
        public AbstractModule_EventListeners<Metadata_Adept, IModuleEventsAdept, AdeptCommands>,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
    public:

        static ModuleClasses moduleClass;

    IModuleCommandAdept():
        AbstractModule_EventListeners()
    {
        IModuleGenericVehicleListener::SetUp<Metadata_Adept, IModuleEventsAdept, AdeptCommands>(this);

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:




};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_Adept_H
