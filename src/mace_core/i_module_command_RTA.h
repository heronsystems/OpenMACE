#ifndef I_RTA_H
#define I_RTA_H

#include "abstract_module_event_listeners.h"
#include "metadata_rta.h"
#include "i_module_topic_events.h"
#include "i_module_events_rta.h"

#include "i_module_command_generic_boundaries.h"
#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class RTACommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    TEST_FUNCTION,
    NEWLY_UPDATED_GRID_SPACING,
    NEW_TASK_LIST
};

class MACE_CORESHARED_EXPORT IModuleCommandRTA :
        public AbstractModule_EventListeners<Metadata_RTA, IModuleEventsRTA, RTACommands>,
        public IModuleGenericBoundaries,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandRTA():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<Metadata_RTA, IModuleEventsRTA, RTACommands>(this);
        IModuleGenericVehicleListener::SetUp<Metadata_RTA, IModuleEventsRTA, RTACommands>(this);


        AddCommandLogic<int>(RTACommands::TEST_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TestFunction(vehicleID);
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(RTACommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });

        AddCommandLogic<int>(RTACommands::NEWLY_UPDATED_GRID_SPACING, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdatedGridSpacing();
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }


    //!
    //! \brief TestFunction
    //! \param vehicleID
    //!
    virtual void TestFunction(const int &vehicleID) = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin New global origin subscriber
    //! \param position New global origin position
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;

    //!
    //! \brief NewlyUpdatedGridSpacing New grid spacing value available
    //!
    virtual void NewlyUpdatedGridSpacing() = 0;

};

} //End MaceCore Namespace

#endif // I_RTA_H

