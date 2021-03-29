#ifndef I_MODULE_COMMAND_ADEPT_H
#define I_MODULE_COMMAND_ADEPT_H
#include "abstract_module_event_listeners.h"
#include "metadata_adept.h"

#include "i_module_topic_events.h"
#include "i_module_events_adept.h"
#include "i_module_events_vehicle.h"

#include "i_module_command_generic_vehicle_listener.h"
#include "i_module_command_ai_support.h"

namespace MaceCore
{

enum class AdeptCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    AI_SUPPORT_GENERAL_COMMAND_ENUMS,
    AI_SUPPORT_TE_COMMAND_ENUMS,
    TEST_FUNCTION
};

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandAdept :
        public AbstractModule_EventListeners<Metadata_Adept, IModuleEventsAdept, AdeptCommands>,
        public IModuleGenericVehicleListener,
        public IModuleCommand_TEAISupport
{
    friend class MaceCore;

    public:
        static ModuleClasses moduleClass;

    IModuleCommandAdept():
        AbstractModule_EventListeners()
    {
        IModuleGenericVehicleListener::SetUp<Metadata_Adept, IModuleEventsAdept, AdeptCommands>(this);
        IModuleCommand_TEAISupport::SetUp<Metadata_Adept, IModuleEventsAdept, AdeptCommands>(this);

        AddCommandLogic<int>(AdeptCommands::TEST_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TestFunction(vehicleID);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    //!
    //! \brief TestFunction
    //! \param vehicleID
    //!
    virtual void TestFunction(const int &vehicleID) = 0;
};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_Adept_H
