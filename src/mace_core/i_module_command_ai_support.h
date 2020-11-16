#ifndef I_MODULE_COMMAND_AI_SUPPORT_H
#define I_MODULE_COMMAND_AI_SUPPORT_H
#include "abstract_module_event_listeners.h"
#include "metadata_adept.h"

#include "i_module_topic_events.h"
#include "i_module_events_adept.h"
#include "i_module_events_vehicle.h"

#include "data_generic_command_item/command_item_components.h"
namespace MaceCore
{

#define AI_SUPPORT_COMMAND_ENUM NEW_TEST_PARAMETERIZATION, /* Current reference frame for altitude is currently undefined. | */ \
INITIALIZE_TEST_CONDITIONS /* Positive altitude over mean sea level (MSL) | */ \

class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommand_AISupport
{
    friend class MaceCore;

public:

    IModuleCommand_AISupport() = default;

    virtual ~IModuleCommand_AISupport() = default;

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {
        ptr->template AddCommandLogic<command_item::Action_InitializeTestSetup>(CT::INITIALIZE_TEST_CONDITIONS, [this](const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewCommand_InitializationCriteria(initialization, sender);
        });
    }

public:

    virtual void NewCommand_InitializationCriteria(const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

};


} //end of namespace MaceCore

#endif // I_MODULE_COMMAND_AI_SUPPORT_H
