#ifndef I_MODULE_COMMAND_PATH_PLANNING_H
#define I_MODULE_COMMAND_PATH_PLANNING_H

#include <string>
#include <map>

#include "abstract_module_event_listeners.h"
#include "metadata_path_planning.h"

#include "i_module_topic_events.h"
#include "i_module_events_path_planning.h"

#include "i_module_command_generic_boundaries.h"
#include "i_module_command_generic_vehicle_listener.h"

#include "base/state_space/goal_state.h"

namespace MaceCore
{

enum class PathPlanningCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_UPDATED_OPERATIONAL_FENCE,
    NEWLY_LOADED_OCCUPANCY_MAP,
    NEWLY_UPDATED_OCCUPANCY_MAP,
    NEWLY_AVAILABLE_MISSION,
    PROCESS_TARGET_STATE
};

class MACE_CORESHARED_EXPORT IModuleCommandPathPlanning  :
        public AbstractModule_EventListeners<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>,
        public IModuleGenericBoundaries,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandPathPlanning():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>(this);
        IModuleGenericVehicleListener::SetUp<MetadataPathPlanning, IModuleEventsPathPlanning, PathPlanningCommands>(this);


        AddCommandLogic<int>(PathPlanningCommands::NEWLY_LOADED_OCCUPANCY_MAP, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(vehicleID);
            UNUSED(sender);
            NewlyLoadedOccupancyMap();
        });

        AddCommandLogic(PathPlanningCommands::NEWLY_UPDATED_OCCUPANCY_MAP, [this](const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedOccupancyMap();
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(PathPlanningCommands::UPDATE_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });

        AddCommandLogic<MissionItem::MissionList>(PathPlanningCommands::NEWLY_AVAILABLE_MISSION, [this](const MissionItem::MissionList &mission, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableMission(mission);
        });

        AddCommandLogic<mace::state_space::GoalState>(PathPlanningCommands::PROCESS_TARGET_STATE, [this](const mace::state_space::GoalState &goal, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableGoalState(goal);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:


    //!
    //! \brief NewlyLoadedOccupancyMap New occupancy map loaded subscriber
    //!
    virtual void NewlyLoadedOccupancyMap() = 0;

    //!
    //! \brief NewlyUpdatedOccupancyMap New updated occupancy map subscriber
    //!
    virtual void NewlyUpdatedOccupancyMap() = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin New global origin available
    //! \param position New global origin
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;

    //!
    //! \brief NewlyAvailableMission
    //! \param mission
    //!
    virtual void NewlyAvailableMission(const MissionItem::MissionList &mission) = 0;

    //!
    //! \brief NewlyAvailableGoalState
    //! \param mission
    //!
    virtual void NewlyAvailableGoalState(const mace::state_space::GoalState &goal) = 0;


};

} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_PATH_PLANNING_H
