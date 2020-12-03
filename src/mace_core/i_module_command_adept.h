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
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    STARTTEST,
    ENDTEST,
    MARKTIME
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
        AddCommandLogic<DataGenericItem::DataGenericItem_MLTest>(AdeptCommands::STARTTEST, [this](const DataGenericItem::DataGenericItem_MLTest &test, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableTestStart(test);
        });

        AddCommandLogic<int>(AdeptCommands::ENDTEST, [this](const int value,const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(value), UNUSED(sender);
            NewlyAvailableTestEnd();
        });

        AddCommandLogic<std::string>(AdeptCommands::MARKTIME, [this](const std::string &time, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableTimeMark(time);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:

    //!
    //! \brief NewlyAvailableTestStart New test available subscriber
    //! \param test Test parameters
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableTestStart(const DataGenericItem::DataGenericItem_MLTest &test) = 0;

    //!
    //! \brief NewlyAvailableTestEnd New test end signal available subscriber
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableTestEnd() = 0;

    //!
    //! \brief NewlyAvailableTimeMark New Time mark available for this agent's log
    //! \param time Time to be marked
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableTimeMark(const std::string &time) = 0;
};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_Adept_H
