#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {


void VehicleObject_MAVLINK::async_example(const std::string &loggingPath)
{
    std::string baseLogName = loggingPath + "/VehicleData_" + std::to_string(this->systemID);

    std::string dataLogName = baseLogName + ".txt";

    std::string loggerName = "Log_Vehicle" + std::to_string(this->systemID);
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    std::shared_ptr<spdlog::logger> mLog = spdlog::basic_logger_mt(logNameArray, dataLogName);
    mLog->set_level(spdlog::level::debug);
}

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const std::string &loggingPath, const int &vehicleID, const int &transmittingID):
    m_CB(NULL), m_CommandController(NULL), m_MissionController(NULL),
    command(NULL), mission(NULL), state(NULL),
    systemID(vehicleID), commandID(transmittingID),
    m_LinkMarshaler(NULL), m_LinkName(""), m_LinkChan(0)
{
    async_example(loggingPath);

    command = new CommandInterface_MAVLINK(systemID, 0);
    command->connectCallback_CommandLong(VehicleObject_MAVLINK::staticCallbackCMDLongFunction, this);

    m_CommandController = new CommandController_MAVLINK(systemID,0);
    m_CommandController->connectCallback(this);

    m_GuidedController = new GuidedController_MAVLINK(systemID,0);
    m_GuidedController->connectCallback(this);

    m_MissionController = new MissionController_MAVLINK(systemID,0);
    m_MissionController->connectCallback(this);

    mission = new MissionData_MAVLINK();
    state = new StateData_MAVLINK();
    state->connectCallback_State(VehicleObject_MAVLINK::staticCallbackState, this);
}

VehicleObject_MAVLINK::~VehicleObject_MAVLINK()
{
    delete command;
    command = NULL;

    delete mission;
    mission = NULL;

    delete state;
    state = NULL;

    delete m_CommandController;
    m_CommandController = NULL;

    delete m_GuidedController;
    m_GuidedController = NULL;

    delete m_MissionController;
    m_MissionController = NULL;

    //do not delete this object as we did not create it
    m_LinkMarshaler = NULL;
}

void VehicleObject_MAVLINK::updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan)
{
    m_LinkMarshaler = commsMarshaler;
    m_LinkName = linkName;
    m_LinkChan = linkChan;

    mavlink_message_t msg;
    mavlink_request_data_stream_t request;

    request.target_system = 0;
    request.target_component = 0;
    request.start_stop = 1;

    request.req_stream_id = 2;
    request.req_message_rate = 1;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 6;
    request.req_message_rate = 3;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 10;
    request.req_message_rate = 5;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 11;
    request.req_message_rate = 2;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::transmitMessage(const mavlink_message_t &msg)
{
    if(m_LinkMarshaler)
    {
        m_LinkMarshaler->SendMAVMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

////////////////////////////////////////////////////////////////////////////
/// Callback Interface Guided Controller: These functions are required per
/// the interface of the guided controller.
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
/// Callback Interface Command Controller: These functions are required per
/// the interface of the command controller.
////////////////////////////////////////////////////////////////////////////

void VehicleObject_MAVLINK::cbiCommandController_transmitCommand(const mavlink_command_int_t &cmd)
{
    mavlink_message_t msg;
    mavlink_msg_command_int_encode_chan(commandID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiCommandController_transmitCommand(const mavlink_command_long_t &cmd)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_encode_chan(commandID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiCommandController_transmitNewMode(const mavlink_set_mode_t &mode)
{
    mavlink_message_t msg;
    mavlink_msg_set_mode_encode_chan(commandID,0,m_LinkChan,&msg,&mode);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiCommandController_CommandACK(const mavlink_command_ack_t &ack)
{
    std::cout<<"The command has been acknowledged."<<std::endl;
}


////////////////////////////////////////////////////////////////////////////
/// Callback Interface Guided Controller: These functions are required per
/// the interface of the guided controller.
////////////////////////////////////////////////////////////////////////////

void VehicleObject_MAVLINK::cbiGuidedController_TransmitMissionItem(const mavlink_mission_item_t &item)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_encode_chan(commandID,0,m_LinkChan,&msg,&item);
    transmitMessage(msg);
}

////////////////////////////////////////////////////////////////////////////
/// Callback Interface Mission Controller: These functions are required per
/// the interface of the mission controller.
////////////////////////////////////////////////////////////////////////////

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_encode_chan(commandID,190,m_LinkChan,&msg,&count);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_encode_chan(commandID,190,m_LinkChan,&msg,&item);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionReqList(const mavlink_mission_request_list_t &request)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &request)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_ReceviedHome(const command_item::SpatialHome &home)
{
    mission->home.set(home);
    if(m_CB)
        m_CB->cbi_VehicleHome(this->systemID,home);
}

void VehicleObject_MAVLINK::cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList)
{
    mission->setCurrentMission(missionList);
    if(m_CB)
        m_CB->cbi_VehicleMission(this->systemID,missionList);
}

void VehicleObject_MAVLINK::cbiMissionController_MissionACK(const mavlink_mission_ack_t &missionACK, const MissionItem::MissionList &missionList)
{
    //first we need the original key as a reference when performing the acknowledgement
    MissionItem::MissionKey originalKey = missionList.getMissionKey();
    if(missionACK.type == MAV_MISSION_ACCEPTED)
    {
        //we need to update the current state of the mission....since it has been accepted and transmitted
        //to the autopilot it is current
        MissionItem::MissionList missionCopy(missionList);
        missionCopy.setMissionTXState(MissionItem::MISSIONSTATE::CURRENT);
        //next lets update the local data instance of the appropriate mission
        mission->setCurrentMission(missionList);

        MissionItem::MissionACK ack(this->systemID, MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_ACCEPTED, originalKey, MissionItem::MISSIONSTATE::CURRENT);

        if(m_CB)
            m_CB->cbi_VehicleMissionACK(ack);
    }
}
} //end of namespace DataInterface_MAVLINK

