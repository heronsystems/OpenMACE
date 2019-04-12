#ifndef EXAMPLE_VEHICLE_MODULE_H
#define EXAMPLE_VEHICLE_MODULE_H

#include "example_vehicle_module_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "module_vehicle_generic/module_vehicle_generic.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "mace_core/module_factory.h"
#include "mace_core/i_module_events_general.h"

#include "data_interface_MAVLINK/components/data_interface_mavlink_components.h"

#include "commsExample/comms_example.h"

#include "controllers/generic_controller.h"

#include "module_vehicle_MAVLINK/controllers/commands/command_land.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_takeoff.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_arm.h"
#include "module_vehicle_MAVLINK/controllers/commands/command_rtl.h"
#include "module_vehicle_MAVLINK/controllers/controller_system_mode.h"

#include "base_topic/vehicle_topics.h"
#include "data_generic_state_item_topic/state_topic_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"



/*
 *
 * USAGE:
 *
 * Insert the nessessary code to do Vehicle communications with MultiWii
 *
 * Look at i_module_events_vehicle.h in mace_core for the events that can be triggered.
 * Feel free to add any nessessary events (if an event is added, its handler must also be added in mace_core.h
 *
 * When it comes time to signal an event to MaceCore do so by calling the following code structure:
 *      NotifyListeners([&](IModuleEventsVehicle *obj){obj->NewPositionDynamics(this, arg1, arg2, ... , argN);});
 * Replacing "NewPositionDynamics" with the event of your choice, and replacing arguments with what is required for that event
 *
 * The start method is the entry point for the thread that the module is to run on.
 * The start() method should contain an event loop of some sort that responds to commands made.
 *
 * Each module will implement commands as defined by it's interface.
 * These commands will NOT be invoked on the thread the module is operating on.
 * If the command is to kick off some action on the module's thread, it will have to marshaled onto the event loop in some way.
 *
 * */

template <typename ...VehicleTopicAdditionalComponents>

class EXAMPLE_VEHICLE_MODULESHARED_EXPORT ExampleVehicleModule :
        public ModuleVehicleGeneric<VehicleTopicAdditionalComponents...>, // EXAMPLE: DATA_VEHICLE_EXAMPLE_TYPES would be replaced with whatever types are developed
        public CommsExample // EXAMPLE: This would be replaced with whatever comms protocol is developed
//        public CallbackInterface_ExampleVehicleObject // EXAMPLE: This would be replaced with whatever callback interface is developed
{

protected:
    typedef ModuleVehicleGeneric<VehicleTopicAdditionalComponents...> ModuleVehicleExampleBase;


public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///             CONFIGURE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ExampleVehicleModule() :
        ModuleVehicleGeneric<VehicleTopicAdditionalComponents...>()
    {

    }

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const
    {

        MaceCore::ModuleParameterStructure structure;
        ConfigureExampleStructure(structure);

        // EXAMPLE: To see an example of how to configure a module structure, see /src/module_vehicle_MAVLINK/module_vehicle_mavlink.h

        return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
    }

    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
    {
        ConfigureComms(params);
        // EXAMPLE: To see an example of how to configure the module, see /src/module_vehicle_MAVLINK/module_vehicle_mavlink.h
    }

    virtual void start()
    {
        ModuleVehicleExampleBase::start();
    }

    virtual void shutdown()
    {
        CommsExample::Shutdown();
        ModuleVehicleExampleBase::shutdown();
    }

public:

public:
    void UpdateDynamicMissionQueue(const TargetItem::DynamicMissionQueue &queue) override {
        UNUSED(queue);
    }

public:
    //!
    //! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
    //! \param linkName Comms link name
    //! \param systemID Vehicle ID generating heartbeat
    //! \param heartbeatMSG Heartbeat message
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG) {
        UNUSED(linkName);
        UNUSED(systemID);
        UNUSED(heartbeatMSG);
    }

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual bool MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg) {
        UNUSED(linkName);
        UNUSED(msg);
    }


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target) {
        UNUSED(topicName);
        UNUSED(sender);
        UNUSED(data);
        UNUSED(target);
    }


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
    virtual void NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) {
        UNUSED(topicName);
        UNUSED(sender);
        UNUSED(componentsUpdated);
        UNUSED(target);
    }


    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr) {
        UNUSED(ptr);
    }


    //!
    //! \brief PublishVehicleData Parent publisher for vehicle data
    //! \param systemID Vehicle ID generating vehicle data
    //! \param components Data components to publish
    //!
    void PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components) {
        UNUSED(systemID);
        UNUSED(components);
    }

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
    //! \brief Command_SystemArm Command a systm arm
    //! \param command Arm/Disarm command
    //! \param sender Sender module
    //!
    virtual void Command_GoTo(const CommandItem::CommandGoTo &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Request_FullDataSync Request all data from all systems
    //! \param targetSystem Destination of the data dump receiver
    //!
    virtual void Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>()) {
        UNUSED(targetSystem);
    }

    //!
    //! \brief Command_SystemArm Command an ARM/DISARM action
    //! \param command ARM/DISARM command
    //! \param sender Generating system
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_VehicleTakeoff Command a takeoff action
    //! \param command Takeoff altitude and location
    //! \param sender Generating system
    //!
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_Land Command a LAND action
    //! \param command Land command
    //! \param sender Generating system
    //!
    virtual void Command_Land(const CommandItem::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_ReturnToLaunch command a return to launch action
    //! \param command RTL command
    //! \param sender Generating system
    //!
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_MissionState Command a mission state request
    //! \param command Mission state request command
    //! \param sender Generating system
    //!
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_ChangeSystemMode Command a system mode change
    //! \param command Change mode command
    //! \param sender Generating system
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender) {
        UNUSED(command);
        UNUSED(sender);
    }

    //!
    //! \brief Command_IssueGeneralCommand Command a general command
    //! \param command General command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command) {
        UNUSED(command);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS:
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_UploadMission Command a mission upload
    //! \param missionList Mission list to upload
    //!
    virtual void Command_UploadMission(const MissionItem::MissionList &missionList) {
        UNUSED(missionList);
    }

    //!
    //! \brief Command_SetCurrentMission Issue a set current mission command
    //! \param key Mission key to set as current mission
    //!
    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key) {
        UNUSED(key);
    }

    //!
    //! \brief Command_GetCurrentMission Issue a get current mission command
    //! \param targetSystem System asking for the current mission
    //!
    virtual void Command_GetCurrentMission(const int &targetSystem) {
        UNUSED(targetSystem);
    }

    //!
    //! \brief Command_GetMission Request a mission by mission key
    //! \param key Mission key
    //! \param sender System asking for the mission
    //!
    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>()) {
        UNUSED(key);
        UNUSED(sender);
    }

    //!
    //! \brief Command_ClearCurrentMission Clear the current mission
    //! \param targetSystem System asking for mission cleared
    //!
    virtual void Command_ClearCurrentMission(const int &targetSystem) {
        UNUSED(targetSystem);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetOnboardAuto Request the current onboard auto mission and state
    //! \param targetSystem System asking for auto info
    //!
    virtual void Command_GetOnboardAuto(const int &targetSystem) {
        UNUSED(targetSystem);
    }

    //!
    //! \brief Command_ClearOnboardAuto Clear the current onboard auto mission and state
    //! \param targetSystem System to clear auto info
    //!
    virtual void Command_ClearOnboardAuto(const int &targetSystem) {
        UNUSED(targetSystem);
    }

    /////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetOnboardGuided Request the current onboard guided state
    //! \param targetSystem System asking for guided info
    //!
    virtual void Command_GetOnboardGuided(const int &targetSystem) {
        UNUSED(targetSystem);
    }

    //!
    //! \brief Command_ClearOnboardGuided Clear the current onboard guided state
    //! \param targetSystem System to clear guided state
    //!
    virtual void Command_ClearOnboardGuided(const int &targetSystem) {
        UNUSED(targetSystem);
    }


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
    void UpdateMissionKey(const MissionItem::MissionKeyChange &key) override {
        UNUSED(key);
    }


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition Request a vehicle's home position
    //! \param vehicleID Vehicle ID corresponding to the home position
    //!
    virtual void Command_GetHomePosition (const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>()) {
        UNUSED(vehicleID);
    }

    //!
    //! \brief Command_SetHomePosition Set a vehicle's home position
    //! \param vehicleHome Vehicle home data
    //!
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic>& = OptionalParameter<MaceCore::ModuleCharacteristic>()) {
        UNUSED(vehicleHome);
    }

    //!
    //! \brief RequestDummyFunction
    //! \param vehicleID
    //!
    virtual void RequestDummyFunction(const int &vehicleID)
    {
        UNUSED(vehicleID);
    }
};

#endif // EXAMPLE_VEHICLE_MODULE_H
