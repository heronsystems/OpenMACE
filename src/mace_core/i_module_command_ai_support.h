#ifndef I_MODULE_COMMAND_AI_SUPPORT_H
#define I_MODULE_COMMAND_AI_SUPPORT_H

#include "mace_core_global.h"
#include "abstract_module_event_listeners.h"

#include "data_generic_item/mace/ai_test_parameterization.h"
#include "data_generic_command_item/mace/ai_items/ai_command_components.h"

namespace MaceCore
{

#define AI_SUPPORT_GENERAL_COMMAND_ENUMS TAG_EVENT_TO_LOGS, /* Command that denotes for the receiver to mark the information to local logs | */ \
    EXECUTE_PROCEDURAL_ACTION /* Command that progresses the action space of the receiving module | */ \

class MACE_CORESHARED_EXPORT IModuleCommand_GeneralAISupport
{
    friend class MaceCore;

public:

    IModuleCommand_GeneralAISupport() = default;

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {
        ptr->template AddCommandLogic<command_item::Action_EventTag>(CT::TAG_EVENT_TO_LOGS, [this](const command_item::Action_EventTag &logEvent, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewAICommand_WriteToLogs(logEvent, sender);
        });

        ptr->template AddCommandLogic<command_item::Action_ProceduralCommand>(CT::EXECUTE_PROCEDURAL_ACTION, [this](const command_item::Action_ProceduralCommand &procedural, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewAICommand_ExecuteProcedural(procedural, sender);
        });
    }

public:

    virtual void NewAICommand_WriteToLogs(const command_item::Action_EventTag &logEvent, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    virtual void NewAICommand_ExecuteProcedural(const command_item::Action_ProceduralCommand &procedural, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;
};


#define AI_SUPPORT_VEHICLE_COMMAND_ENUMS INITIALIZE_TEST_CONDITIONS /* Command that should enable the accompanying module to execute any required setup procedures before running the test | */

class MACE_CORESHARED_EXPORT IModuleCommand_VehicleAISupport : public IModuleCommand_GeneralAISupport
{
    friend class MaceCore;

public:

    IModuleCommand_VehicleAISupport():
        IModuleCommand_GeneralAISupport()
    {

    }

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {
        IModuleCommand_GeneralAISupport::SetUp(ptr);

        ptr->template AddCommandLogic<command_item::Action_InitializeTestSetup>(CT::INITIALIZE_TEST_CONDITIONS, [this](const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewAICommand_HWInitializationCriteria(initialization, sender);
        });
    }

public:
    virtual void NewAICommand_HWInitializationCriteria(const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

};



#define AI_SUPPORT_TE_COMMAND_ENUMS NEW_EVALUATION_TRIAL /* Command that should enable the accompanying module to execute any required setup procedures before running the test | */

class MACE_CORESHARED_EXPORT IModuleCommand_TEAISupport : public IModuleCommand_GeneralAISupport
{
    friend class MaceCore;

public:

    IModuleCommand_TEAISupport() = default;

    template<typename T, typename TT, typename CT>
    void SetUp(AbstractModule_EventListeners<T, TT, CT> *ptr)
    {
        IModuleCommand_GeneralAISupport::SetUp(ptr);

        ptr->template AddCommandLogic<DataGenericItem::AI_TestParameterization>(CT::NEW_EVALUATION_TRIAL, [this](const DataGenericItem::AI_TestParameterization &obj, const OptionalParameter<ModuleCharacteristic> &sender)
        {
            NewAICommand_NewEvaluationTrial(obj, sender);
        });
    }

public:
    virtual void NewAICommand_NewEvaluationTrial(const DataGenericItem::AI_TestParameterization &obj, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

};
} //end of namespace MaceCore

#endif // I_MODULE_COMMAND_AI_SUPPORT_H
