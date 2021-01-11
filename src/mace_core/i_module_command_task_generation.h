#ifndef I_MODULE_COMMAND_TASK_GENERATION_H
#define I_MODULE_COMMAND_TASK_GENERATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_task_generation.h"

#include "i_module_topic_events.h"
#include "i_module_events_task_generation.h"
#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class TaskGenerationCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    UPDATED_MAP_LAYER,
    TEST_FUNCTION
};


//class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandTaskGeneration :
        public AbstractModule_EventListeners<Metadata_Task_Generation, IModuleEventsTaskGeneration, TaskGenerationCommands>,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;

public:

    static ModuleClasses moduleClass;

    IModuleCommandTaskGeneration():
        AbstractModule_EventListeners()
    {
        IModuleGenericVehicleListener::SetUp<Metadata_Task_Generation, IModuleEventsTaskGeneration, TaskGenerationCommands>(this);

        AddCommandLogic<std::string>(TaskGenerationCommands::UPDATED_MAP_LAYER, [this](const std::string &layerName, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            LayeredMapLayerUpdated(layerName);
        });


        AddCommandLogic<int>(TaskGenerationCommands::TEST_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TestFunction(vehicleID);
        });

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

    virtual void LayeredMapLayerUpdated(const std::string &layerName) = 0;

    virtual void TestFunction(const int &vehicleID) = 0;

};

} //end namespace mace


#endif // I_MODULE_COMMAND_TASK_GENERATION_H
