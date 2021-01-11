#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>
#include <condition_variable>

#include "data/mission_command.h"

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_command_item/command_item_components.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "flight_states/ardupilot_state_components.h"
#include "vehicle_object/vehicle_object_ardupilot.h"

#include "mace_core/abstract_module_base.h"

using namespace std::placeholders;

class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<>
{
public:
    ModuleVehicleArdupilot();


    virtual ~ModuleVehicleArdupilot() override;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params) override;

    //!
    //! \brief createLog Create a log file for this vehicle
    //! \param systemID Vehicle ID generating the log
    //!
    void createLog(const int &systemID);

public:
    void UpdateDynamicMissionQueue(const command_target::DynamicMissionQueue &queue) override;

    //!
    //! \brief TransmitVisionPoseEstimate
    //! \param pose
    //!
    void TransmitVisionPoseEstimate(const mace::pose::Pose &pose) override;

public:
    //!
    //! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
    //! \param linkName Comms link name
    //! \param systemID Vehicle ID generating heartbeat
    //! \param heartbeatMSG Heartbeat message
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG) override;

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual bool MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg) override;


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
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated,
                                 const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) override;


    //!
    //! \brief PublishVehicleData Parent publisher for vehicle data
    //! \param systemID Vehicle ID generating vehicle data
    //! \param components Data components to publish
    //!
    void PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandVehicle via AbstractModuleBaseVehicleListener.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
    /// command and action sequence that accompanies the vheicle. Expect an
    /// acknowledgement or an event to take place when calling these items.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_SetGlobalOrigin
    //! \param command
    //! \param sender
    //!
    void Command_SetGlobalOrigin(const command_item::Action_SetGlobalOrigin &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_SystemArm Command a systm arm
    //! \param command Arm/Disarm command
    //! \param sender Sender module
    //!
    virtual void Command_ExecuteSpatialItem(const command_item::Action_ExecuteSpatialItem &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Request_FullDataSync Request all data from all systems
    //! \param targetSystem Destination of the data dump receiver
    //!
    virtual void Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    //!
    //! \brief Command_SystemArm Command an ARM/DISARM action
    //! \param command ARM/DISARM command
    //! \param sender Generating system
    //!
    virtual void Command_SystemArm(const command_item::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_VehicleTakeoff Command a takeoff action
    //! \param command Takeoff altitude and location
    //! \param sender Generating system
    //!
    virtual void Command_VehicleTakeoff(const command_item::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_Land Command a LAND action
    //! \param command Land command
    //! \param sender Generating system
    //!
    virtual void Command_Land(const command_item::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_ReturnToLaunch command a return to launch action
    //! \param command RTL command
    //! \param sender Generating system
    //!
    virtual void Command_ReturnToLaunch(const command_item::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_MissionState Command a mission state request
    //! \param command Mission state request command
    //! \param sender Generating system
    //!
    virtual void Command_MissionState(const command_item::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_ChangeSystemMode Command a system mode change
    //! \param command Change mode command
    //! \param sender Generating system
    //!
    virtual void Command_ChangeSystemMode(const command_item::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;

    //!
    //! \brief Command_IssueGeneralCommand Command a general command
    //! \param command General command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<command_item::AbstractCommandItem> &command) override;

    //!
    //! \brief Command_SetDynamicTarget
    //! \param command
    //! \param sender
    //!
    void Command_ExecuteDynamicTarget(const command_item::Action_DynamicTarget &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) override;


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_UploadMission Command a mission upload
    //! \param missionList Mission list to upload
    //!
    virtual void Command_UploadMission(const MissionItem::MissionList &missionList) override;

    //!
    //! \brief Command_SetCurrentMission Issue a set current mission command
    //! \param key Mission key to set as current mission
    //!
    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key) override;

    //!
    //! \brief Command_GetCurrentMission Issue a get current mission command
    //! \param targetSystem System asking for the current mission
    //!
    virtual void Command_GetCurrentMission(const int &targetSystem) override;

    //!
    //! \brief Command_GetMission Request a mission by mission key
    //! \param key Mission key
    //! \param sender System asking for the mission
    //!
    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    //!
    //! \brief Command_ClearCurrentMission Clear the current mission
    //! \param targetSystem System asking for mission cleared
    //!
    virtual void Command_ClearCurrentMission(const int &targetSystem) override;

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetOnboardAuto Request the current onboard auto mission and state
    //! \param targetSystem System asking for auto info
    //!
    virtual void Command_GetOnboardAuto(const int &targetSystem) override;

    //!
    //! \brief Command_ClearOnboardAuto Clear the current onboard auto mission and state
    //! \param targetSystem System to clear auto info
    //!
    virtual void Command_ClearOnboardAuto(const int &targetSystem) override;

    /////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetOnboardGuided Request the current onboard guided state
    //! \param targetSystem System asking for guided info
    //!
    virtual void Command_GetOnboardGuided(const int &targetSystem) override;

    //!
    //! \brief Command_ClearOnboardGuided Clear the current onboard guided state
    //! \param targetSystem System to clear guided state
    //!
    virtual void Command_ClearOnboardGuided(const int &targetSystem) override;


    //THE OLD ONES AND THEN WE COMPARE
    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality may be pertinent for vehicles not containing a
    /// direct MACE hardware module.
    /////////////////////////////////////////////////////////////////////////


    //!
    //! \brief UpdateMissionKey Update the current mission's key
    //! \param key New mission key
    //!
    void UpdateMissionKey(const MissionItem::MissionKeyChange &key) override;


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition Request a vehicle's home position
    //! \param vehicleID Vehicle ID corresponding to the home position
    //!
    virtual void Command_GetHomePosition (const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic>&sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;

    //!
    //! \brief Command_SetHomePosition Set a vehicle's home position
    //! \param vehicleHome Vehicle home data
    //!
    virtual void Command_SetHomePosition(const command_item::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic>&sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override;


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


    /////////////////////////////////////////////////////////////////////////
    /// AI SUPPORT EVENTS: These events are explicit overrides in support of
    /// AI based autonomy.
    /////////////////////////////////////////////////////////////////////////

    virtual void NewAICommand_WriteToLogs(const command_item::Action_EventTag &logEvent, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override
    {
        UNUSED(sender);
        UNUSED(logEvent);
    }

    virtual void NewAICommand_ExecuteProcedural(const command_item::Action_ProceduralCommand &procedural, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override
    {
        UNUSED(sender);
        UNUSED(procedural);
    }

    virtual void NewAICommand_HWInitializationCriteria(const command_item::Action_InitializeTestSetup &initialization, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) override
    {
        UNUSED(initialization);
        UNUSED(sender);
    }

    //!
    //! \brief RequestDummyFunction
    //! \param vehicleID
    //!
    virtual void RequestDummyFunction(const int &vehicleID) override
    {
        UNUSED(vehicleID);
    }


protected:
    static void staticCallbackFunction_VehicleTarget(void *p, MissionTopic::VehicleTargetTopic &target)
    {
        ((ModuleVehicleArdupilot *)p)->callbackFunction_VehicleTarget(target);
    }

    void callbackFunction_VehicleTarget(const MissionTopic::VehicleTargetTopic &target)
    {
        std::shared_ptr<MissionTopic::VehicleTargetTopic> ptrTarget = std::make_shared<MissionTopic::VehicleTargetTopic>(target);
        ModuleVehicleMAVLINK::cbi_VehicleMissionData(target.getVehicleID(),ptrTarget);
    }

protected:

    //!
    //! \brief ProgressStateMachineStates Cause the state machine to update its states
    //!
    void ProgressStateMachineStates();

protected:
    std::mutex m_Mutex_VehicleData;

protected:
    std::mutex m_Mutex_StateMachine;
    hsm::StateMachine* stateMachine; /**< Member variable containing a pointer to the state
 machine. This state machine evolves the state per event updates and/or external commands. */


};

#endif // MODULE_VEHICLE_ARDUPILOT_H
