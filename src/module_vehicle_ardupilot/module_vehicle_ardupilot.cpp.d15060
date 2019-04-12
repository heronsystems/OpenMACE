#include "module_vehicle_ardupilot.h"
#include <functional>

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<>(),
    m_VehicleMissionTopic("vehicleMission"), m_AircraftController(NULL), vehicleData(NULL)
{

}

void ModuleVehicleArdupilot::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ModuleVehicleMAVLINK::ConfigureModule(params);
}

void ModuleVehicleArdupilot::createLog(const int &systemID)
{
    std::string logname = this->loggingPath + "/VehicleModule_" + std::to_string(systemID) + ".txt";
    std::string loggerName = "VehicleModule_" + std::to_string(systemID);
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    this->mLogs = spdlog::basic_logger_mt(logNameArray, logname);
    mLogs->set_level(spdlog::level::debug);
}

////////////////////////////////////////////////////////////////////////////
/// DataInterface_MAVLINK: These functions are required for the connection
/// of the virtual interface via the vehicleData object
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::cbi_VehicleCommandACK(const int &systemID, const mavlink_command_ack_t &cmdACK)
{
    UNUSED(systemID);
    if(checkControllerState())
        m_AircraftController->updateCommandACK(cmdACK);
}

void ModuleVehicleArdupilot::cbi_VehicleMissionACK(const MissionItem::MissionACK &ack)
{
    std::cout<<"The module has now seen a mission ack."<<std::endl;
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
         ptr->EventVehicle_MissionACK(this, ack);
     });

    //This function shall update the local MACE core of the new mission
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->NewCurrentVehicleMission(this, ack.getUpdatedMissionKey());
    });
}

void ModuleVehicleArdupilot::cbi_VehicleMissionData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data)
{
    MaceCore::TopicDatagram topicDatagram;
    m_VehicleMissionTopic.SetComponent(data, topicDatagram);
    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleMissionTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    });
}

void ModuleVehicleArdupilot::cbi_VehicleMissionItemCurrent(const MissionItem::MissionItemCurrent &current)
{
    std::cout<<"The vehicle module has seen a current mission item"<<std::endl;
    //This function shall update the local MACE core of the new mission
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->GVEvents_MissionItemCurrent(this, current);
    });

    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemCurrentTopic>(current);
    cbi_VehicleMissionData(current.getMissionKey().m_systemID,ptrMissionTopic);
}

void ModuleVehicleArdupilot::cbi_VehicleStateData(const int &systemID, std::shared_ptr<Data::ITopicComponentDataObject> data)
{
    MaceCore::TopicDatagram topicDatagram;
    m_VehicleDataTopic.SetComponent(data, topicDatagram);
    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    });
}

void ModuleVehicleArdupilot::cbi_VehicleHome(const int &systemID, const CommandItem::SpatialHome &home)
{
    std::stringstream buffer;
    buffer << home;

    mLogs->debug("Receieved a new vehicle home position.");
    mLogs->info(buffer.str());

    //notify the core of the change
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->GVEvents_NewHomePosition(this, home);
    });

    std::shared_ptr<CommandItem::SpatialHome> ptrHome = std::make_shared<CommandItem::SpatialHome>(home);
    std::shared_ptr<MissionTopic::MissionHomeTopic> homeTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
    homeTopic->setHome(ptrHome);

    this->cbi_VehicleMissionData(systemID,homeTopic);
}

void ModuleVehicleArdupilot::cbi_VehicleMission(const int &systemID, const MissionItem::MissionList &missionList)
{
    mLogs->debug("Receieved a new vehicle mission.");

    //This function shall update the local MACE CORE instance of the mission
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->EventVehicle_NewOnboardVehicleMission(this, missionList);
    });
    //We should update all listeners
    std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>(missionList);

    //This function shall update the local vehicle object with the current vehicle mission
    vehicleData->mission->setCurrentMission(missionList);
    this->cbi_VehicleMissionData(systemID,missionTopic);
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMissionTopic.Name());
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Request_FullDataSync(const int &targetSystem)
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> objectData = vehicleData->state->GetTopicData();
    this->PublishVehicleData(targetSystem,objectData);
    vehicleData->m_MissionController->requestMission();
}

void ModuleVehicleArdupilot::Command_SystemArm(const CommandItem::ActionArm &command)
{
    std::stringstream buffer;
    buffer << command;

    mLogs->debug("Receieved a command system arm.");
    mLogs->info(buffer.str());

    vehicleData->m_CommandController->setSystemArm(command);
}

void ModuleVehicleArdupilot::Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command)
{
    std::stringstream buffer;
    buffer << command;

    mLogs->debug("Receieved a command takeoff.");
    mLogs->info(buffer.str());

    if(vehicleData)
    {
        if(command.getTargetSystem() == vehicleData->getSystemID())
        {
            Ardupilot_TakeoffController* newController = new Ardupilot_TakeoffController(vehicleData);
            if(command.position.has3DPositionSet())
                newController->initializeTakeoffSequence(command);
            else{
                CommandItem::SpatialTakeoff defaultTakeoff = command;
                newController->initializeTakeoffSequence(defaultTakeoff);
            }
            newController->connectTargetCallback(ModuleVehicleArdupilot::staticCallbackFunction_VehicleTarget, this);

            this->SpinUpController(newController);
        }
    }

}

void ModuleVehicleArdupilot::Command_Land(const CommandItem::SpatialLand &command)
{
    std::stringstream buffer;
    buffer << command;

    mLogs->debug("Receieved a command to land.");
    mLogs->info(buffer.str());

    if(vehicleData)
        vehicleData->m_CommandController->setSystemLand(command);
}

void ModuleVehicleArdupilot::Command_ReturnToLaunch(const CommandItem::SpatialRTL &command)
{
    mLogs->debug("Receieved a command RTL.");

    if(vehicleData)
        vehicleData->m_CommandController->setSystemRTL(command);
}

void ModuleVehicleArdupilot::Command_MissionState(const CommandItem::ActionMissionCommand &command)
{
    mLogs->debug("Receieved a command to change mission state.");

    int systemID = command.getTargetSystem();
    if((vehicleData) && (vehicleData->getSystemID() == systemID))
    {
        if(command.getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_PAUSE)
        {
            DataGenericItem::DataGenericItem_Heartbeat heartbeat = vehicleData->state->vehicleHeartbeat.get();
            if(Data::isSystemTypeRotary(heartbeat.getType()))
            {
                DataARDUPILOT::ARDUPILOTComponent_FlightMode tmp = vehicleData->state->vehicleFlightMode.get();
                int mode = tmp.getFlightModeFromString("BRAKE");
                vehicleData->m_CommandController->setNewMode(mode);
            }
            else{
                DataARDUPILOT::ARDUPILOTComponent_FlightMode tmp = vehicleData->state->vehicleFlightMode.get();
                int mode = tmp.getFlightModeFromString("LOITER");
                vehicleData->m_CommandController->setNewMode(mode);
            }

//            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
//                ptr->GVEvents_MissionExeStateUpdated(this, key, Data::MissionExecutionState::MESTATE_PAUSED);
//            });
        }else if(command.getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_START)
        {
            DataARDUPILOT::ARDUPILOTComponent_FlightMode tmp = vehicleData->state->vehicleFlightMode.get();
            int mode = tmp.getFlightModeFromString("AUTO");
            vehicleData->m_CommandController->setNewMode(mode);

//            MissionItem::MissionKey key = tmpData->data->currentAutoMission.get().getMissionKey();
//            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
//                ptr->GVEvents_MissionExeStateUpdated(this, key, Data::MissionExecutionState::MESTATE_EXECUTING);
//            });
        }
    }
}

void ModuleVehicleArdupilot::Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command)
{
    std::stringstream buffer;
    buffer << command;

    mLogs->debug("Receieved a command to change the mode.");
    mLogs->info(buffer.str());

    DataARDUPILOT::ARDUPILOTComponent_FlightMode tmp = vehicleData->state->vehicleFlightMode.get();
    int mode = tmp.getFlightModeFromString(command.getRequestMode());
    vehicleData->command->setNewMode(mode,255,m_LinkChan);
}

void ModuleVehicleArdupilot::Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{

}

void ModuleVehicleArdupilot::SpinUpController(Ardupilot_GeneralController *newController) {

    m_AircraftController = newController;
    m_AircraftController->start();
}

void ModuleVehicleArdupilot::SpinDownController() {
    m_AircraftController->terminateObject();
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetHomePosition(const int &vehicleID)
{
    if((vehicleData) && (vehicleData->getSystemID() == vehicleID))
        vehicleData->command->getSystemHome();
}

void ModuleVehicleArdupilot::Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome)
{
    std::stringstream buffer;
    buffer << vehicleHome;

    mLogs->debug("Receieved a command to home position.");
    mLogs->info(buffer.str());

    if((vehicleData) && (vehicleData->getSystemID() == vehicleHome.getTargetSystem()))
        vehicleData->m_CommandController->setHomePosition(vehicleHome);
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::UpdateMissionKey(const MissionItem::MissionKeyChange &key)
{
//    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(key.oldKey.m_systemID);
//    MissionItem::MissionList missionList = tmpData->data->Command_GetCurrentMission(key.oldKey.m_missionType);
//    if(missionList.getMissionKey() == key.oldKey)
//    {
//        missionList.setMissionKey(key.newKey);
//        tmpData->data->setCurrentMission(missionList);
//    }
}

void ModuleVehicleArdupilot::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    switch(missionList.getMissionType())
    {
    case(MissionItem::MISSIONTYPE::AUTO): //This case should push the mission directly to the aircraft
    {
        if(vehicleData)
            vehicleData->m_MissionController->transmitMission(missionList);
        break;
    }
    case(MissionItem::MISSIONTYPE::GUIDED):
    {
        //In these two cases the mission should be carefully considered if we are a module
        //aboard MACE hardware companion package or communicating via ground link
        if(airborneInstance)
        {
            //If we are an airborne instance we can acknowledge this right away
            //This implies we are aboard the aircraft directly communicating with the autopilot
            //Thus higher rate capabilities and request for state are available
        }else{

        }
        break;
    }
    default:
        break;
    }
}


void ModuleVehicleArdupilot::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);
}

void ModuleVehicleArdupilot::Command_GetCurrentMission(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_GetMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);
}

void ModuleVehicleArdupilot::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetOnboardAuto(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_ClearOnboardAuto(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleVehicleArdupilot::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    if(vehicleData)
        vehicleData->parseMessage(&message);
}


void ModuleVehicleArdupilot::VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    UNUSED(linkName);
    if(vehicleData == NULL)
    {
        createLog(systemID);
        //this is the first time we have seen this heartbeat or the data was destroyed for some reason
        vehicleData = std::make_shared<DataInterface_MAVLINK::VehicleObject_MAVLINK>(systemID,255);
        vehicleData->updateCommsInfo(m_LinkMarshaler,m_LinkName,m_LinkChan);
        vehicleData->connectCallback(this);
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->EventVehicle_NewConstructedVehicle(this, systemID);
        });

        vehicleData->m_MissionController->requestMission();
    }


    DataARDUPILOT::ARDUPILOTComponent_FlightMode flightMode = vehicleData->state->vehicleFlightMode.get();
    flightMode.parseMAVLINK(heartbeatMSG);
    if(vehicleData->state->vehicleFlightMode.set(flightMode))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrFlightMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(flightMode);
        this->cbi_VehicleStateData(systemID, ptrFlightMode);
    }

    DataGenericItem::DataGenericItem_SystemArm arm;
    arm.setSystemArm(heartbeatMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
    if(vehicleData->state->vehicleArm.set(arm))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(arm);
        this->cbi_VehicleStateData(systemID, ptrArm);
    }

    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(Data::AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA);
    heartbeat.setCompanion(this->airborneInstance);
    heartbeat.setProtocol(Data::CommsProtocol::COMMS_MAVLINK);

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
    vehicleData->state->vehicleHeartbeat.set(heartbeat);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    this->cbi_VehicleStateData(systemID,ptrHeartbeat);

}

void ModuleVehicleArdupilot::PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components)
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

void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    if(topicName == m_VehicleMissionTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMissionTopic.Name(), senderID);
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

