#include "module_vehicle_arducopter.h"
#include <functional>

#include "mace_core/i_module_events_general.h"

template <typename T>
T CopyCommandAndInsertTarget(const command_item::AbstractCommandItem &item, int targetSystem)
{
    T cpy((T&)item);
    cpy.setTargetSystem(static_cast<unsigned int>(targetSystem));
    return cpy;
}


//ModuleVehicleArducopter::ModuleVehicleArducopter() :
//    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUCOPTER_TYPES>(),
//    m_VehicleMissionTopic("vehicleMission"), m_AircraftController(nullptr), vehicleData(nullptr)
ModuleVehicleArducopter::ModuleVehicleArducopter() :
    ModuleVehicleArdupilot()
{

    m_TransmissionQueue = new TransmitQueue(2000, 3);
}

ModuleVehicleArducopter::~ModuleVehicleArducopter()
{
    if(stateMachine)
    {
        delete stateMachine;
    }

    m_ControllersCollection.ForAll([this](Controllers::IController<mavlink_message_t, int>* controller){
        controller->RemoveHost(this);
    });

    delete m_TransmissionQueue;
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleArducopter::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ModuleVehicleMAVLINK::ConfigureModule(params);
}

//!
//! \brief createLog Create log file
//! \param systemID ID of vehicle being logged
//!
void ModuleVehicleArducopter::createLog(const int &systemID)
{
//    std::string logname = this->loggingPath + "/VehicleModule_" + std::to_string(systemID) + ".txt";

    // Initiate logs:
    std::string loggerName = "vehicle_arducopter_" + std::to_string(systemID);
    std::string loggerPath = this->loggingPath + "/vehicle_logs/arducopter_" + std::to_string(systemID) + ".txt";
    this->initiateLogs(loggerName, loggerPath);
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArducopter::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMissionTopic.Name());
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArducopter::Command_SetGlobalOrigin(const Action_SetGlobalOrigin &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    mace::pose::GeodeticPosition_3D* originPosition = command.getGlobalOrigin()->positionAs<mace::pose::GeodeticPosition_3D>();
    m_SystemData->environment->swarmGlobalOrigin.set(*originPosition);

//    handleGlobalOriginController(command);
}

void ModuleVehicleArducopter::Command_ExecuteSpatialItem(const Action_ExecuteSpatialItem &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    Action_ExecuteSpatialItem commandWithTarget = CopyCommandAndInsertTarget<Action_ExecuteSpatialItem>(command, this->GetAttachedMavlinkEntity());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(commandWithTarget.getClone());
    ProgressStateMachineStates();
}

//!
//! \brief Request_FullDataSync Request all data from all systems
//! \param targetSystem Destination of the data dump receiver
//!
void ModuleVehicleArducopter::Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> objectData = m_SystemData->state->GetTopicData();
    this->PublishVehicleData(targetSystem,objectData);
    //vehicleData->m_MissionController->requestMission();
}

//!
//! \brief Command_SystemArm Command an ARM/DISARM action
//! \param command ARM/DISARM command
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_SystemArm(const command_item::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::ActionArm commandWithTarget = CopyCommandAndInsertTarget<command_item::ActionArm>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

//    mLogs->debug("Receieved a command system arm.");
//    mLogs->info(buffer.str());
    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(command.getClone());

    ProgressStateMachineStates();
}

//!
//! \brief Command_VehicleTakeoff Command a takeoff action
//! \param command Takeoff altitude and location
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_VehicleTakeoff(const command_item::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::SpatialTakeoff commandWithTarget = CopyCommandAndInsertTarget<command_item::SpatialTakeoff>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

//    mLogs->debug("Receieved a command takeoff.");
//    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(commandWithTarget.getClone());
    ProgressStateMachineStates();
}

//!
//! \brief Command_Land Command a LAND action
//! \param command Land command
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_Land(const command_item::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::SpatialLand commandWithTarget = CopyCommandAndInsertTarget<command_item::SpatialLand>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

//    mLogs->debug("Receieved a command to land.");
//    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(commandWithTarget.getClone());
    ProgressStateMachineStates();
}

//!
//! \brief Command_ReturnToLaunch command a return to launch action
//! \param command RTL command
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_ReturnToLaunch(const command_item::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::SpatialRTL commandWithTarget = CopyCommandAndInsertTarget<command_item::SpatialRTL>(command, this->GetAttachedMavlinkEntity());

//    mLogs->debug("Receieved a command RTL.");

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(commandWithTarget.getClone());
    ProgressStateMachineStates();
}

//!
//! \brief Command_MissionState Command a mission state request
//! \param command Mission state request command
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_MissionState(const command_item::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::ActionMissionCommand commandWithTarget = CopyCommandAndInsertTarget<command_item::ActionMissionCommand>(command, this->GetAttachedMavlinkEntity());

    //mLogs->debug("Receieved a command to change mission state.");

    int systemID = commandWithTarget.getTargetSystem();

    if((m_SystemData != nullptr) && (m_SystemData->getMAVLINKID() == systemID))
    {
        ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
        currentOuterState->handleCommand(commandWithTarget.getClone());
        ProgressStateMachineStates();
    }
}

//!
//! \brief Command_ChangeSystemMode Command a system mode change
//! \param command Change mode command
//! \param sender Generating system
//!
void ModuleVehicleArducopter::Command_ChangeSystemMode(const command_item::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    //Temporary solution to solve boadcasting until rework of commands can be done
    command_item::ActionChangeMode commandWithTarget = CopyCommandAndInsertTarget<command_item::ActionChangeMode>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

//    mLogs->debug("Receieved a command to change the mode.");
//    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* outerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    outerState->handleCommand(commandWithTarget.getClone());
    ProgressStateMachineStates();
}

void ModuleVehicleArducopter::Command_ExecuteDynamicTarget(const command_item::Action_DynamicTarget &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    ardupilot::state::AbstractStateArdupilot* outerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    AbstractCommandItemPtr currentCommand = std::make_shared<command_item::Action_DynamicTarget>(command);
    outerState->handleCommand(currentCommand);
    ProgressStateMachineStates();
}

//!
//! \brief Command_IssueGeneralCommand Command a general command
//! \param command General command
//!
void ModuleVehicleArducopter::Command_IssueGeneralCommand(const std::shared_ptr<command_item::AbstractCommandItem> &command)
{
    UNUSED(command);
    std::cout<<"Ken: we have seen the issuing of a general command!!!! Pay attention to this."<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way arducopter works.
/////////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetHomePosition Request a vehicle's home position
//! \param vehicleID Vehicle ID corresponding to the home position
//!
void ModuleVehicleArducopter::Command_GetHomePosition(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(vehicleID); UNUSED(sender);
    //    if((vehicleData) && (vehicleData->getSystemID() == vehicleID))
    //        vehicleData->command->getSystemHome();
}

//!
//! \brief Command_SetHomePosition Set a vehicle's home position
//! \param vehicleHome Vehicle home data
//!
void ModuleVehicleArducopter::Command_SetHomePosition(const command_item::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(sender);
    /*
    std::stringstream buffer;
    buffer << vehicleHome;

    mLogs->debug("Receieved a command to home position.");
    mLogs->info(buffer.str());
    */
    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(vehicleHome.getClone());
    ProgressStateMachineStates();
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

//!
//! \brief UpdateMissionKey Update the current mission's key
//! \param key New mission key
//!
void ModuleVehicleArducopter::UpdateMissionKey(const MissionItem::MissionKeyChange &key)
{
    UNUSED(key);
    //    std::shared_ptr<DataARDUCOPTER::VehicleObject_ARDUCOPTER> tmpData = getArducopterData(key.oldKey.m_systemID);
    //    MissionItem::MissionList missionList = tmpData->data->Command_GetCurrentMission(key.oldKey.m_missionType);
    //    if(missionList.getMissionKey() == key.oldKey)
    //    {
    //        missionList.setMissionKey(key.newKey);
    //        tmpData->data->setCurrentMission(missionList);
    //    }
}

//!
//! \brief Command_UploadMission Command a mission upload
//! \param missionList Mission list to upload
//!
void ModuleVehicleArducopter::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    std::stringstream buffer;
    buffer << missionList;

//    mLogs->info("Vehicle module has been told to upload a mission.");
//    mLogs->info(buffer.str());

    switch(missionList.getMissionType())
    {
    case(MissionItem::MISSIONTYPE::AUTO): //This case should push the mission directly to the aircraft
    {
        //request the current mission on the vehicle
        this->prepareMissionController();
        MAVLINKUXVControllers::ControllerMission* missionController = static_cast<MAVLINKUXVControllers::ControllerMission*>(m_ControllersCollection.At("missionController"));

        missionController->AddLambda_Finished(this, [this, missionController, missionList](const bool completed, const uint8_t code){
            UNUSED(this);
            UNUSED(code);
            if (completed)
                printf("Mission Upload Completed\n");
            else
                printf("Mission Upload Failed\n");
            //Ken Fix: We should handle stuff related to the completion and the code
            //update the data contained for the vehicle mission
            //vehicleData->mission->currentAutoMission.set(missionList);
            //we should notify everyone that this the current auto mission
            missionController->Shutdown();
        });

        //            MavlinkEntityKey target = missionList.getVehicleID();
        //            command_item::SpatialHome home;
        //            home.setTargetSystem(2);
        //            home.setOriginatingSystem(255);
        //            home.setPosition(Base3DPosition(-35.3625523,149.165201,15));
        //            missionController->UploadMission(missionList,home,target);
        //        if(vehicleData)
        //            vehicleData->m_MissionController->transmitMission(missionList);
        break;
    }
    case(MissionItem::MISSIONTYPE::GUIDED):
    {
        //In these two cases the mission should be carefully considered if we are a module
        //aboard MACE hardware companion package or communicating via ground link
        if(airborneInstance)
        {
            //If we are an airborne instance we can acknowledge this right away
            //This implies we are aboard the aircraft directly communicating with the autocopter
            //Thus higher rate capabilities and request for state are available
        }else{
            //since we are not airborne and someone has requested a guided mission, we will assume that
            //we
        }
        break;
    }
    default:
        break;
    }
}

//!
//! \brief Command_SetCurrentMission Issue a set current mission command
//! \param key Mission key to set as current mission
//!
void ModuleVehicleArducopter::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);
}

//!
//! \brief Command_GetCurrentMission Issue a get current mission command
//! \param targetSystem System asking for the current mission
//!
void ModuleVehicleArducopter::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
    //    mavlink_message_t msg;
    //    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
    //    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

//!
//! \brief Command_GetMission Request a mission by mission key
//! \param key Mission key
//! \param sender System asking for the mission
//!
void ModuleVehicleArducopter::Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key);
    UNUSED(sender);
}

//!
//! \brief Command_ClearCurrentMission Clear the current mission
//! \param targetSystem System asking for mission cleared
//!
void ModuleVehicleArducopter::Command_ClearCurrentMission(const int &targetSystem)
{
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
void ModuleVehicleArducopter::Command_GetOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
    //    mavlink_message_t msg;
    //    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
    //    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

//!
//! \brief Command_ClearOnboardAuto Clear the current onboard auto mission and state
//! \param targetSystem System to clear auto info
//!
void ModuleVehicleArducopter::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
    //    mavlink_message_t msg;
    //    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
    //    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
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
void ModuleVehicleArducopter::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief Command_ClearOnboardGuided Clear the current onboard guided state
//! \param targetSystem System to clear guided state
//!
void ModuleVehicleArducopter::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
bool ModuleVehicleArducopter::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    bool consumed = false;

    //this is necessary because if we have yet to have received a vehicle heartbeat,
    //we have yet to form the vehicle object and accompanying state machine
    if(m_SystemData)
    {
//        MavlinkEntityKey sender = message.sysid;
        //consumed = m_MissionController->ReceiveMessage(&message, sender);

        if(!consumed)
            consumed = ModuleVehicleMAVLINK::MavlinkMessage(linkName, message);

        if(!consumed)
            consumed = m_SystemData->handleMAVLINKMessage(message);

        if(!consumed)
            consumed = m_SystemData->parseMessage(&message);

        ProgressStateMachineStates();
    }

    return consumed;
}

//!
//! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
//! \param linkName Comms link name
//! \param systemID Vehicle ID generating heartbeat
//! \param heartbeatMSG Heartbeat message
//!
void ModuleVehicleArducopter::VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    //If module hasn't started yet, then ignore this message
    if(this->ModuleStarted() == false)
    {
        return;
    }

    UNUSED(linkName);
    if(m_SystemData == nullptr)
    {
        MavlinkEntityKey key = systemID;
        SetAttachedMavlinkEntity(key);

        createLog(systemID);
        //this is the first time we have seen this heartbeat or the data was destroyed for some reason
        m_SystemData = new VehicleObject_Arducopter(this, this->GetCharacteristic(), systemID);

        m_SystemData->connectCallback(this);
        dynamic_cast<VehicleObject_Ardupilot*>(m_SystemData)->connectTargetCallback(ModuleVehicleArducopter::staticCallbackFunction_VehicleTarget, this);
        m_SystemData->environment->set_ShouldTransformLocalAltitude(transformToSwarmAltitude);

        //setup the vision_position_estimate controller
        MAVLINKUXVControllers::Controller_VisionPositionEstimate* visionEstimateController = new MAVLINKUXVControllers::Controller_VisionPositionEstimate(m_SystemData, m_TransmissionQueue, m_LinkChan);
        visionEstimateController->setLambda_Shutdown([this]() mutable
        {
            UNUSED(this);
            auto ptr = m_ControllersCollection.Remove("visionEstimateController");
            delete ptr;
        });
        m_ControllersCollection.Insert("visionEstimateController", visionEstimateController);


        MaceCore::ModuleCharacteristic moduleCharacterstic = this->GetCharacteristic();
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->Events_NewVehicle(this, static_cast<uint8_t>(systemID), moduleCharacterstic);
        });

        if(stateMachine)
        {
            delete stateMachine;
            stateMachine = nullptr;
        }

        stateMachine = new hsm::StateMachine();
        stateMachine->Initialize<ardupilot::state::State_Unknown>(m_SystemData);

        handleFirstConnectionSetup();
    } //end of if statement verifying that the current vehicle data is null

    std::string currentFlightMode = dynamic_cast<VehicleObject_Ardupilot*>(m_SystemData)->m_ArdupilotMode->parseMAVLINK(heartbeatMSG);
    DataGenericItem::DataGenericItem_FlightMode flightMode;
    flightMode.setFlightMode(currentFlightMode);
    if(m_SystemData->status->vehicleMode.set(flightMode))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrFlightMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(flightMode);
        this->cbi_VehicleStateData(systemID, ptrFlightMode);
    }

    DataGenericItem::DataGenericItem_SystemArm arm;
    arm.setSystemArm(heartbeatMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
    if(m_SystemData->status->vehicleArm.set(arm))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(arm);
        ModuleVehicleMAVLINK::cbi_VehicleStateData(systemID, ptrArm);
    }

    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(Data::AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA);
    heartbeat.setCompanion(this->airborneInstance);
    heartbeat.setProtocol(Data::CommsProtocol::COMMS_MAVLINK);
    heartbeat.setMavlinkID(systemID);

    switch(heartbeatMSG.type)
    {
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_QUADROTOR);
        break;
    case MAV_TYPE_FIXED_WING:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_FIXED_WING);
        break;
    default:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_GENERIC);
    }

    // Set MACE HSM state:
    if(this->stateMachine->getCurrentState()) {
        ardupilot::state::AbstractStateArdupilot* outerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentState());
        heartbeat.setHSMState(outerState->getCurrentStateEnum());
    }

    m_SystemData->status->vehicleHeartbeat.set(heartbeat);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    ModuleVehicleMAVLINK::cbi_VehicleStateData(systemID,ptrHeartbeat);

    ProgressStateMachineStates();
}

//!
//! \brief PublishVehicleData Parent publisher for vehicle data
//! \param systemID Vehicle ID generating vehicle data
//! \param components Data components to publish
//!
void ModuleVehicleArducopter::PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components)
{
    if(components.size() > 0)
    {
        //construct datagram
        MaceCore::TopicDatagram topicDatagram;
        for(size_t i = 0 ; i < components.size() ; i++)
        {
            m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
            //notify listeners of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            });

        }
    } //if there is information available
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
void ModuleVehicleArducopter::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(sender);
    UNUSED(target);
    if(this->m_TopicToControllers.find(topicName) == m_TopicToControllers.cend())
    {
        throw std::runtime_error("Attempting to send a topic that the vehicle module link has no knowledge of");
    }

    Controllers::IController<mavlink_message_t, int> *controller = m_TopicToControllers.at(topicName);

    if(controller->ContainsAction(Controllers::Actions::SEND) == false)
    {
        throw std::runtime_error("Attempting to send a topic to a controller that has no send action");
    }
    Controllers::IActionSend<MaceCore::TopicDatagram, int> *sendAction = dynamic_cast<Controllers::IActionSend<MaceCore::TopicDatagram, int>*>(controller);
    sendAction->Send(data, 255, this->GetAttachedMavlinkEntity());
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
void ModuleVehicleArducopter::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    UNUSED(target);
    if(topicName == m_VehicleMissionTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMissionTopic.Name(), sender);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }
}

void ModuleVehicleArducopter::TransmitVisionPoseEstimate(const mace::pose::Pose &pose)
{
    if(m_ControllersCollection.Exist("visionEstimateController"))
    {
        MAVLINKUXVControllers::Controller_VisionPositionEstimate* visionPositionEstimateController = static_cast<MAVLINKUXVControllers::Controller_VisionPositionEstimate*>(m_ControllersCollection.At("visionEstimateController"));
        if(visionPositionEstimateController != nullptr)
        {
            MavlinkEntityKey sender = 255;
            visionPositionEstimateController->Broadcast(pose,sender);
        }
    }
}


void ModuleVehicleArducopter::UpdateDynamicMissionQueue(const command_target::DynamicMissionQueue &queue)
{
    UNUSED(queue);
}
