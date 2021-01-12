#ifndef I_MODULE_COMMAND_EXTERNAL_LINK_H
#define I_MODULE_COMMAND_EXTERNAL_LINK_H

#include <string>
#include <map>

#include "abstract_module_base.h"
#include "abstract_module_event_listeners.h"
#include "abstract_module_base_vehicle_listener.h"

#include "metadata_ground_station.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"
#include "i_module_events_external_link.h"

#include "command_marshler.h"
#include "module_characteristics.h"

#include "base/geometry/polygon_cartesian.h"

#include "i_module_command_generic_boundaries.h"
#include "i_module_command_ai_support.h"

namespace MaceCore
{

enum class ExternalLinkCommands
{
    BASE_MODULE_LISTENER_ENUMS,
    BASE_MODULE_VEHICLE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    AI_SUPPORT_GENERAL_COMMAND_ENUMS,
    AI_SUPPORT_VEHICLE_COMMAND_ENUMS,
    REQUEST_REMOTE_BOUNDARY,
    NEWLY_AVAILABLE_ONBOARD_MISSION,
    NEW_MISSION_EXE_STATE,
    NEWLY_AVAILABLE_HOME_POSITION,
    NEWLY_AVAILABLE_MODULE,
    RECEIVED_MISSION_ACK
};

class MACE_CORESHARED_EXPORT IModuleCommandExternalLink :
        public AbstractModule_VehicleListener<Metadata_GroundStation, IModuleEventsExternalLink, ExternalLinkCommands>,
        public IModuleGenericBoundaries, public IModuleCommand_VehicleAISupport
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandExternalLink():
        AbstractModule_VehicleListener(), IModuleCommand_VehicleAISupport()
    {
        IModuleGenericBoundaries::SetUp<Metadata_GroundStation, IModuleEventsExternalLink, ExternalLinkCommands>(this);
        IModuleCommand_VehicleAISupport::SetUp<Metadata_GroundStation, IModuleEventsExternalLink, ExternalLinkCommands>(this);


        AddCommandLogic<MissionItem::MissionKey>(ExternalLinkCommands::NEWLY_AVAILABLE_ONBOARD_MISSION, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableOnboardMission(key, sender);
        });

        AddCommandLogic<MissionItem::MissionKey>(ExternalLinkCommands::NEW_MISSION_EXE_STATE, [this](const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender){            
            UNUSED(sender);
            NewlyAvailableMissionExeState(key);
        });

        AddCommandLogic<command_item::SpatialHome>(ExternalLinkCommands::NEWLY_AVAILABLE_HOME_POSITION, [this](const command_item::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender){
            NewlyAvailableHomePosition(home, sender);
        });

        AddCommandLogic<ModuleCharacteristic, ModuleClasses>(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, [this](const ModuleCharacteristic &module, const ModuleClasses type, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableModule(module, type);
        });

        AddCommandLogic<MissionItem::MissionACK>(ExternalLinkCommands::RECEIVED_MISSION_ACK, [this](const MissionItem::MissionACK &ack, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            ReceivedMissionACK(ack);
        });

        AddCommandLogic<std::tuple<ModuleCharacteristic, uint8_t>>(ExternalLinkCommands::REQUEST_REMOTE_BOUNDARY, [this](const std::tuple<ModuleCharacteristic, uint8_t> &remote, const OptionalParameter<ModuleCharacteristic> &sender){
            Command_RequestBoundaryDownload(remote, sender);
        });

    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:

    //!
    //! \brief NewlyAvailableBoundary New boundary available subscriber
    //! \param boundary Key for new boundary
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableBoundary(const uint8_t &boundary, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    //!
    //! \brief NewlyAvailableOnboardMission New onboard mission available subscriber
    //! \param key Key for new mission
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<ModuleCharacteristic> &sender = OptionalParameter<ModuleCharacteristic>()) = 0;

    //!
    //! \brief NewlyAvailableMissionExeState New mission EXE state subscriber
    //! \param missionKey Mission key for new EXE state
    //!
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) = 0;

    //!
    //! \brief NewlyAvailableHomePosition New home position subscriber
    //! \param home New home position
    //! \param sender Sender module
    //!
    virtual void NewlyAvailableHomePosition(const command_item::SpatialHome &home, const OptionalParameter<ModuleCharacteristic> &sender) = 0;


    //!
    //! \brief NewlyAvailableModule New module available subscriber
    //! \param module New module characteristic
    //!
    virtual void NewlyAvailableModule(const ModuleCharacteristic &module, const ModuleClasses &type) = 0;


    //!
    //! \brief ReceivedMissionACK Mission ACK subscriber
    //! \param ack New mission ack
    //!
    virtual void ReceivedMissionACK(const MissionItem::MissionACK &ack) = 0;

    virtual void Command_RequestBoundaryDownload(const std::tuple<ModuleCharacteristic, uint8_t> &remote, const OptionalParameter<ModuleCharacteristic> &sender) = 0;


};


} //End MaceCore Namespace

#endif // I_MODULE_COMMAND_EXTERNAL_LINK_H
