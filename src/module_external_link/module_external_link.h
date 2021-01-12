#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include "module_external_link_global.h"

#include <sstream>
#include <iostream>
#include <stdint.h>
#include <chrono>
#include <functional>
#include <chrono>

#include <mavlink.h>

#include "common/common.h"

#include "commsMACEHelper/comms_mace_helper.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_external_link.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "controllers/heartbeat_controller_externallink.h"

#include "controllers/generic_controller.h"


#include "controllers/commands/command_arm.h"
#include "controllers/commands/command_land.h"
#include "controllers/commands/command_mission_item.h"
#include "controllers/commands/command_rtl.h"
#include "controllers/commands/command_takeoff.h"
#include "controllers/commands/command_execute_spatial_action.h"
#include "controllers/commands/command_system_mode.h"
#include "controllers/controller_home.h"
#include "controllers/controller_mission.h"
#include "controllers/controller_boundary.h"


#include "mace_core/module_characteristics.h"

#include "data/topic_components/altitude.h"
#include "data/topic_components/position_global.h"
#include "data/topic_components/position_local.h"
#include "data/topic_components/topic_component_void.h"
#include "data/topic_components/topic_component_string.h"

#include "base_topic/base_topic_components.h"

class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public CommsMACEHelper,
        public ExternalLink::HeartbeatController_Interface,
        public Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic>
{


private:

    /**
     * @brief BroadcastLogicToAllVehicles
     * @param vehicleID VehicleID to send function to. If set to 0 then send to all local vehicles
     * @param func Function to call
     */
    void BroadcastLogicToAllVehicles(int vehicleID, const std::function<void(int)> &func)
    {
        if(vehicleID == 0)
        {
            std::vector<unsigned int> vehicleIDs;
            this->getDataObject()->GetLocalVehicles(vehicleIDs);
            for(auto it = vehicleIDs.begin() ; it != vehicleIDs.end() ; ++it)
            {
                func(*it);
            }
        }
        else
        {
            func(vehicleID);
        }
    }


    enum class ContainedControllers
    {
        MISSION_DOWNLOAD,
        MISSION_UPLOAD
    };

public:

    ModuleExternalLink();

    ~ModuleExternalLink();

    virtual std::vector<MaceCore::TopicCharacteristic> GetEmittedTopics() override;

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) override;

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const override;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params) override;


    //!
    //! \brief Event to fire when an external module has been added
    //! \param resourceName Name of resource (module) added
    //! \param ID ID of module
    //!
    void ExternalModuleAdded(const CommsMACE::Resource &resource);

    void ExternalModuleRemoved(const CommsMACE::Resource &resource);

    std::string createLog(const unsigned int &systemID);

    virtual void TransmitMessage(const mavlink_message_t &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) const override;

    virtual MaceCore::ModuleCharacteristic GetKeyFromSecondaryID(int ID) const override;

    virtual MaceCore::ModuleCharacteristic GetHostKey() const override;

    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from the Heartbeat Controller
    /// Interface via callback functionality.
    ///////////////////////////////////////////////////////////////////////////////////////
    void cbiHeartbeatController_transmitCommand(const mavlink_mace_heartbeat_t &heartbeat) override;


    void ReceivedMission(const MissionItem::MissionList &list);
    Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn FetchMissionFromKey(const OptionalParameter<MissionKey> &key);
    Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn FetchAllMissionFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module);


    void ReceivedHome(const MaceCore::ModuleCharacteristic &moduleAppliedTo, const SpatialHome &home);
    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchKeyReturn FetchHomeFromKey(const OptionalParameter<MaceCore::ModuleCharacteristic> &key);
    Controllers::DataItem<MaceCore::ModuleCharacteristic, command_item::SpatialHome>::FetchModuleReturn FetchAllHomeFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module);

    void ReceivedCommand(const MaceCore::ModuleCharacteristic &sender, const AbstractCommandItem &command);
    void ReceivedGoToCommand(const MaceCore::ModuleCharacteristic &moduleFor, const AbstractCommandItem &command);



    bool isExternalLinkAirborne() const
    {
        return airborneInstance;
    }

    void ParseForData(const mavlink_message_t* message);


    void PublishVehicleData(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<Data::ITopicComponentDataObject> &component);

    void PublishMissionData(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<Data::ITopicComponentDataObject> &component);



    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MACEMessage(const std::string &linkName, const mavlink_message_t &msg) override;

    //!
    //! \brief VehicleHeartbeatInfo
    //! \param linkName
    //! \param systemID
    //! \param heartbeatMSG
    //!
    void HeartbeatInfo(const MaceCore::ModuleCharacteristic &sender, const mavlink_mace_heartbeat_t &heartbeatMSG);


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) override;


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;



    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandExternalLink
    /// via the base listener class.
    ///////////////////////////////////////////////////////////////////////////////////////

public:
    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
    /// command and action sequence that accompanies the vheicle. Expect an
    /// acknowledgement or an event to take place when calling these items.
    ////////////////////////////////////////////////////////////////////////////

    void Command_SetGlobalOrigin(const Action_SetGlobalOrigin &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_GoTo
    //! \param command
    //! \param sender
    //!
    void Command_ExecuteSpatialItem(const Action_ExecuteSpatialItem &goTo, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Request_FullDataSync
    //! \param targetSystem
    //!
    void Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    //!
    //! \brief Command_ChangeVehicleArm
    //! \param vehicleArm
    //!
    void Command_SystemArm(const command_item::ActionArm &systemArm, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_ChangeVehicleOperationalMode
    //! \param vehicleMode
    //!
    void Command_ChangeSystemMode(const command_item::ActionChangeMode &vehicleMode, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_RequestVehicleTakeoff
    //! \param vehicleTakeoff
    //!
    void Command_VehicleTakeoff(const command_item::SpatialTakeoff &vehicleTakeoff, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_Land
    //! \param command
    //!
    void Command_Land(const command_item::SpatialLand &vehicleLand, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_ReturnToLaunch
    //! \param command
    //!
    void Command_ReturnToLaunch(const command_item::SpatialRTL &vehicleRTL, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;


    //!
    //! \brief Command_MissionState
    //! \param command
    //!
    void Command_MissionState(const command_item::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_IssueGeneralCommand
    //! \param command
    //!
    void Command_IssueGeneralCommand(const std::shared_ptr<command_item::AbstractCommandItem> &command) override;

    //!
    //! \brief Command_ExecuteDynamicTarget
    //! \param command
    //! \param sender
    //!
    void Command_ExecuteDynamicTarget(const command_item::Action_DynamicTarget &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;


    //!
    //! \brief Command_EmitHeartbeat
    //! \param heartbeat
    //!
    virtual void Command_EmitHeartbeat(const command_item::SpatialTakeoff &heartbeat);

    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality may be pertinent for vehicles not containing a
    /// direct MACE hardware module.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_UploadMission This function allows for a MACE instance to set
    //! a mission queue of a remote MACE instance. This is the only time this should be
    //! called. Missions at this point should merely be in a state of proposed as
    //! the it will be up to the remote instance to confirm receipt and action. No changes
    //! should be made with this associated list state until such event takes place.
    //! \param missionList The mission desired to be transmitted to the remote instance.
    //!
    virtual void Command_UploadMission(const MissionItem::MissionList &missionList) override;

    virtual void Command_GetCurrentMission(const int &targetSystem) override;

    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key) override;
    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;
    virtual void Command_ClearCurrentMission(const int &targetSystem) override;


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL AUTO COMMANDS: This is implying for auto mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentGuidedQueue
    //! \param missionList
    //!
    virtual void Command_GetOnboardAuto(const int &targetSystem) override;

    //!
    //! \brief RequestCurrentGuidedQueue
    //! \param vehicleID
    //!
    virtual void Command_ClearOnboardAuto (const int &targetSystem) override;


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED COMMANDS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentGuidedQueue
    //! \param missionList
    //!
    virtual void Command_GetOnboardGuided(const int &targetSystem) override;

    //!
    //! \brief RequestCurrentGuidedQueue
    //! \param vehicleID
    //!
    virtual void Command_ClearOnboardGuided (const int &targetSystem) override;


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition
    //! \param vehicleID
    //!
    virtual void Command_GetHomePosition (const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    //!
    //! \brief Command_SetHomePosition
    //! \param vehicleHome
    //!
    virtual void Command_SetHomePosition(const command_item::SpatialHome &systemHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    /////////////////////////////////////////////////////////////////////////
    /// EXPLICIT CONTROL EVENTS: These events are explicit overrides of the
    /// vehicle, often developed in tight coordination with the vehicle itself.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_SetSurfaceDeflection
    //! \param action
    //!
    virtual void Command_SetSurfaceDeflection(const command_item::Action_SetSurfaceDeflection &action, const OptionalParameter<MaceCore::ModuleCharacteristic>&sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override
    {
        UNUSED(action);
        UNUSED(sender);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandExternalLink.
    ///////////////////////////////////////////////////////////////////////////////////////
    void NewlyAvailableBoundary(const uint8_t &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;
    virtual void NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;
    virtual void NewlyAvailableHomePosition(const command_item::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey) override;
    virtual void NewlyAvailableModule(const MaceCore::ModuleCharacteristic &module, const MaceCore::ModuleClasses &type) override;
    virtual void ReceivedMissionACK(const MissionItem::MissionACK &ack) override;
    virtual void Command_RequestBoundaryDownload(const std::tuple<MaceCore::ModuleCharacteristic, uint8_t> &remote, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    virtual void NewAICommand_WriteToLogs(const command_item::Action_EventTag &logEvent, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {

    }

    virtual void NewAICommand_ExecuteProcedural(const command_item::Action_ProceduralCommand &procedural, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {

    }

    virtual void NewAICommand_HWInitializationCriteria(const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>())
    {

    }


private:



    //! A list of all known remote boundaries and their characterstic
    std::unordered_map<ModuleBoundaryIdentifier, BoundaryItem::BoundaryCharacterisic> m_RemoteBoundaries;

    //!
    //! \brief Function to be called when a new remote boundary characteristic has been recevied by the controller
    //! \param sender Module that contains the boundary
    //! \param data Data about boundary
    //!
    void ReceivedRemoteBoundaryCharacterstic(const MaceCore::ModuleCharacteristic &remoteModule, const uint8_t remoteBoundaryID, const BoundaryItem::BoundaryCharacterisic &characteristic);

    Controllers::DataItem<ModuleBoundaryIdentifier, BoundaryItem::BoundaryList>::FetchKeyReturn FetchBoundaryFromKey(const OptionalParameter<ModuleBoundaryIdentifier> &key);

    void ReceivedRemoteBoundary(const MaceCore::ModuleCharacteristic &remoteModule, uint8_t remoteBoundaryID, const BoundaryItem::BoundaryList &list);

    void ReceivedRemoteMissionNotification(const MaceCore::ModuleCharacteristic &remoteModule, const MissionItem::MissionKey &key);


    //!
    //! \brief Procedure to perform when a new MACE instance is added.
    //!
    //! This method will consult with data object and send out any relevant data to the remote MACE instance
    //!
    //! \param MaceInstanceID ID of new mace instance.
    //!
    void NewExternalMaceInstance(uint8_t MaceInstanceID);


    //!
    //! \brief Check if the given systemID is known. If unknown then do steps to add the vehicle to this MACE instance
    //! \param sender
    //! \param systemID
    //!
    void CheckAndAddVehicle(const MaceCore::ModuleCharacteristic &sender, unsigned int systemID);

    void RequestRemoteResources()
    {
        //make request
        this->m_LinkMarshaler->RequestRemoteResources(this->m_LinkName);
    }

private:

    ExternalLink::HeartbeatController_ExternalLink *m_HeartbeatController;

    PointerCollection<
    ExternalLink::CommandARM,
    ExternalLink::CommandLand,
    ExternalLink::CommandMissionItem,
    ExternalLink::CommandRTL,
    ExternalLink::CommandTakeoff,
    ExternalLink::Controller_GoTo,
    ExternalLink::ControllerCommand_SystemMode,
    ExternalLink::ControllerHome,
    ExternalLink::ControllerMission,
    ExternalLink::ControllerBoundary
    > m_Controllers;


private:
    bool airborneInstance;
    //!
    //! \brief associatedSystemID This is the identifier that is transmitting the data as a representative of.
    //! In the case of an airborne instance it will be assigned to the value of the heartbeat message recieved
    //! over the internal mace network. Thus, outbound transmissions will be relevant to the request of the
    //! system to which to it attached. It will be defaulted to match the GCS ID.
    //!
    unsigned int associatedSystemID;
    std::map<unsigned int,unsigned int> systemIDMap;

    MaceCore::SpooledTopic<BASE_POSE_TOPICS, DATA_GENERIC_VEHICLE_ITEM_TOPICS> m_VehicleDataTopic;
    MaceCore::SpooledTopic<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;


protected:
    TransmitQueue *m_queue;

    std::unordered_map<std::string, Controllers::IController<mavlink_message_t, MaceCore::ModuleCharacteristic>*> m_TopicToControllers;

private:

    std::unordered_map<MaceCore::ModuleCharacteristic, std::vector<std::function<void()>>> m_TasksToDoWhenModuleComesOnline;
};

#endif // MODULE_EXTERNAL_LINK_H
