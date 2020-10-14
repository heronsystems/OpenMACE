#ifndef I_GROUND_STATION_H
#define I_GROUND_STATION_H

#include "abstract_module_event_listeners.h"
#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_ground_station.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_item/data_generic_item_components.h"

#include "base/pose/cartesian_position_3D.h"


#include "i_module_command_generic_boundaries.h"
#include "i_module_command_generic_vehicle_listener.h"

namespace MaceCore
{

enum class GroundStationCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    GENERIC_MODULE_VEHICLE_LISTENER_ENUMS,
    NEWLY_AVAILABLE_CURRENT_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION,
    NEWLY_AVAILABLE_VEHICLE_PARAMETERS
};

class MACE_CORESHARED_EXPORT IModuleCommandGroundStation :
        public AbstractModule_EventListeners<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>,
        public IModuleGenericBoundaries,
        public IModuleGenericVehicleListener
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandGroundStation():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>(this);
        IModuleGenericVehicleListener::SetUp<Metadata_GroundStation, IModuleEventsGroundStation, GroundStationCommands>(this);

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableCurrentMission(missionKey);
        });

        AddCommandLogic<MissionItem::MissionKey>(GroundStationCommands::NEW_MISSION_EXE_STATE, [this](const MissionItem::MissionKey &missionKey, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableMissionExeState(missionKey);
        });

        AddCommandLogic<command_item::SpatialHome>(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const command_item::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableHomePosition(home, sender);
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(GroundStationCommands::UPDATE_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });

        AddCommandLogic<std::map<std::string, DataGenericItem::DataGenericItem_ParamValue>>(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE_PARAMETERS, [this](const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &paramValues, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableParameterList(paramValues, sender);
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:

    //!
    //! \brief NewlyAvailableCurrentMission New current mission available
    //! \param missionKey Mission key of new mission
    //!
    virtual void NewlyAvailableCurrentMission(const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief NewlyAvailableMissionExeState New mission EXE state available subscriber
    //! \param missionKey Mission key for new exe state
    //!
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief NewlyAvailableHomePosition New home position available subscriber
    //! \param home Home position
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableHomePosition(const command_item::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin New global origin subscriber
    //! \param position New global origin position
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;

    //!
    //! \brief NewlyAvailableParameterList
    //! \param params
    //!
    virtual void NewlyAvailableParameterList(const std::map<std::string, DataGenericItem::DataGenericItem_ParamValue> &params, const OptionalParameter<ModuleCharacteristic> &sender) = 0;

    //!
    //! \brief StartTCPServer Start module TCP server for GUI communications
    //! \return
    //!
    virtual bool StartTCPServer() = 0;


};


} //End MaceCore Namespace

#endif // I_GROUND_STATION_H
