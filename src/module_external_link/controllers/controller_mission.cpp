#include "controller_mission.h"

namespace ExternalLink {


//!
//! \brief Called when building mavlink packet initial request to a mission
//! \param data
//! \param cmd
//!
bool ControllerMission::Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj)
{
    MaceLog::Green("In Construct_Send (mission key), (mission request list)");

    UNUSED(target);

    queueObj = data;

    cmd.target_system = static_cast<uint8_t>(data.m_systemID);
    cmd.target_component = 0;
    cmd.mission_type = static_cast<uint8_t>(data.m_missionType);

    if(m_MissionsBeingFetching.find(data) != m_MissionsBeingFetching.cend())
    {
        return false;
    }

    MissionItem::MissionList newList;
    newList.setMissionKey(data);
    newList.clearQueue();
    MissionRequestStruct newItem;
    newItem.mission = newList;
    newItem.requester = sender;

    m_MissionsBeingFetching.insert({data, newItem});

    std::cout << "Mission Controller: Sending Mission Request List. " << "S_ID: " << (int)data.m_systemID << " M_ID: " << (int)data.m_missionID << std::endl;

    return true;
}

/*
    bool ControllerMission::BuildData_Send(const mace_mission_request_list_t &cmd, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &rtn, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(cmd.mission_system, cmd.mission_creator, cmd.mission_id, static_cast<MissionItem::MISSIONTYPE>(cmd.mission_type), static_cast<MissionItem::MISSIONSTATE>(cmd.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;

        vehicleObj = this->GetKeyFromSecondaryID(cmd.mission_system);



        //if we don't know the mission being upload fetch it.
        //These code is generally executed, the exception being when the receiver doesn't receive mace_mission_count_t message and resends mace_mission_request_list_t
        if(m_MissionsUploading.find(sender) == m_MissionsUploading.cend())
        {
            m_MissionsUploading.insert({sender, {}});
        }
        if(m_MissionsUploading.at(sender).find(key) == m_MissionsUploading.at(sender).cend())
        {
            std::vector<std::tuple<MissionKey, MissionList>> missions;
            Controllers::DataItem<MissionItem::MissionKey, MissionItem::MissionList>::FetchDataFromKey(key, missions);

            if(missions.size() == 0)
            {
                return false;
            }
            if(missions.size() > 1)
            {
                throw std::runtime_error("Multiple missions assigned to the same key returned, This is a non-op");
            }
            if(std::get<0>(missions.at(0)) != key)
            {
                throw std::runtime_error("Requesting a specific missionkey did not return the same key, This is a non-op");
            }

            MissionList mission = std::get<1>(missions.at(0));
            m_MissionsUploading.at(sender).insert({key, mission});
        }


        rtn.count = m_MissionsUploading.at(sender).at(key).getQueueSize();
        rtn.target_system = m_MissionsUploading.at(sender).at(key).getVehicleID();
        rtn.mission_system =  static_cast<uint8_t>(key.m_systemID);
        rtn.mission_creator =  static_cast<uint8_t>(key.m_creatorID);
        rtn.mission_id =  static_cast<uint8_t>(key.m_missionID);
        rtn.mission_type = static_cast<uint8_t>(key.m_missionType);
        rtn.mission_state = static_cast<uint8_t>(key.m_missionState);

        std::cout << "Mission Controller: Sending Mission Count. " << "S_ID: " << (int)cmd.mission_system << " M_ID: " << (int)cmd.mission_id << std::endl;

        return true;
    }

*/


bool ControllerMission::BuildData_Send(const mavlink_mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_request_int_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
{
    MaceLog::Green("In BuildData_Send (mission count)");

    UNUSED(sender);
    MissionItem::MissionKey key(mission.target_system, mission.mission_creator, mission.mission_id, static_cast<MissionItem::MISSIONTYPE>(mission.mission_type));
    receiveQueueObj = key;
    respondQueueObj = key;


    if(m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        return false;
    }

    moduleFor = this->GetHostKey();

    m_MissionsBeingFetching.at(key).mission.initializeQueue(mission.count);

    request.mission_creator = mission.mission_creator;
    request.mission_id = mission.mission_id;
    request.target_system = mission.target_system;
    request.mission_type = mission.mission_type;
    request.target_system = mission.target_system;
    request.seq = 0;

    std::cout << "Mission Controller: Requesting Item " << 0 << " S_ID: " << (int)mission.target_system << " M_ID: " << (int)mission.mission_id << std::endl;

    return true;
}


bool ControllerMission::BuildData_Send(const mavlink_mace_mission_request_int_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_item_int_t &missionItem, MaceCore::ModuleCharacteristic &moduleFrom, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
{

    MaceLog::Green("In BuildData_Send (mission request int)");

    UNUSED(sender);
    MissionItem::MissionKey key(missionRequest.target_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type));
    receiveQueueObj = key;
    respondQueueObj = key;

    moduleFrom = this->GetKeyFromSecondaryID(missionRequest.target_system);

    if(m_MissionsUploading.find(sender) == m_MissionsUploading.cend())
    {
        //            if(CONTROLLER_MISSION_TYPE::mLog)
        //                CONTROLLER_MISSION_TYPE::mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
        return false;
    }
    if(m_MissionsUploading.at(sender).find(key) == m_MissionsUploading.at(sender).cend())
    {
        //            if(CONTROLLER_MISSION_TYPE::mLog)
        //                CONTROLLER_MISSION_TYPE::mLog->error("MissionController_ExternalLink has been told to transmit a mission item from a mission which keys dont match the contained.");
        return false;
    }

    unsigned int index = static_cast<unsigned int>(missionRequest.seq);
    if(index >= m_MissionsUploading.at(sender)[key].getQueueSize())
    {
        //this indicates that RX system requested something OOR
        //            if(CONTROLLER_MISSION_TYPE::mLog)
        //                CONTROLLER_MISSION_TYPE::mLog->error("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + " which is greater than the size of the list contained.");
        return false;
    }

    //        if(CONTROLLER_MISSION_TYPE::mLog)
    //            CONTROLLER_MISSION_TYPE::mLog->info("MissionController_ExternalLink has been told to transmit a mission item with index " + std::to_string(index) + ".");

    std::shared_ptr<command_item::AbstractCommandItem> ptrItem = this->m_MissionsUploading.at(sender)[key].getMissionItem(index);

    MissionItem::MissionItemFactory::generateMACEMissionItem(ptrItem,index, missionItem);
    MissionItem::MissionItemFactory::updateMissionKey(key, missionItem);

    std::cout << "Mission Controller: Sending Item " << index << " S_ID: " << (int)missionRequest.target_system << " M_ID: " << (int)missionRequest.mission_id << std::endl;

    return true;
}


bool ControllerMission::BuildData_Send(const mavlink_mace_mission_item_int_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_request_int_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
{
    MaceLog::Green("In BuildData_Send (mission item int)");

    UNUSED(sender);
    //MTB this isn't quite right, but I think it only effects more or less data fields that are not used.
    MaceCore::ModuleCharacteristic target;
    target.ModuleID = missionItem.target_system;
    target.MaceInstance = 0;

    MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type));
    receiveQueueObj = key;
    respondQueueObj = key;


    //check if mission item received is part of a mission we are activly downloading
    if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        //            if(CONTROLLER_MISSION_TYPE::mLog)
        //                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
        return false;
    }

    unsigned int seqReceived = missionItem.seq;
    if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
    {
        std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
        //            if(CONTROLLER_MISSION_TYPE::mLog)
        //                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
        return false;
    }
    //execution will only continue if not last item
    if(seqReceived == (m_MissionsBeingFetching[key].mission.getQueueSize() - 1))
    {
        return false;
    }

    moduleFor = this->GetHostKey();

    command_item::AbstractCommandItemPtr newMissionItem = MissionItem::MissionItemFactory::generateAbstractCommandItem(missionItem, target.ModuleID, target.ModuleID);
    m_MissionsBeingFetching[key].mission.replaceMissionItemAtIndex(newMissionItem, seqReceived);

    MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].mission.getMissionListStatus();
    if(status.state == MissionItem::MissionList::COMPLETE)
    {
        throw std::runtime_error("Still have more items to request, but mission is full");
    }


    int indexRequest = status.remainingItems.at(0);

    request.target_system = static_cast<uint8_t>(target.ModuleID);
    request.mission_creator =  static_cast<uint8_t>(key.m_creatorID);
    request.mission_id =  static_cast<uint8_t>(key.m_missionID);
    request.mission_type =  static_cast<uint8_t>(key.m_missionType);
    request.seq =  static_cast<uint16_t>(indexRequest);

    std::cout << "Mission Controller: Requesting Item " << indexRequest << " S_ID: " << (int)missionItem.target_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

    return true;
}

bool ControllerMission::Construct_FinalObjectAndResponse(const mavlink_mace_mission_item_int_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &ackMission, MissionItem::MissionKey &finalKey, MissionItem::MissionList &finalList, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &queueObj)
{

    MaceLog::Green("In Construct_FinalObjectAndResponse");

    UNUSED(sender);
    //MTB this isn't quite right, but I think it only effects more or less data fields that are not used.
    MaceCore::ModuleCharacteristic target;
    target.ModuleID = missionItem.target_system;
    target.MaceInstance = 0;

    MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type));
    queueObj = key;

    moduleFor = this->GetHostKey();

    //check if mission item received is part of a mission we are activly downloading
    if(this->m_MissionsBeingFetching.find(key) == m_MissionsBeingFetching.cend())
    {
        //            if(CONTROLLER_MISSION_TYPE::mLog) {
        //                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with a key that is not equal to the one we were originally told.");
        //            }
        return false;
    }

    uint seqReceived = missionItem.seq;
    if(seqReceived > (m_MissionsBeingFetching[key].mission.getQueueSize() - 1)) //this should never happen
    {
        std::cout << "Mission download Error: received a mission item with an index greater than available in the queue" << std::endl;
        //            if(CONTROLLER_MISSION_TYPE::mLog) {
        //                CONTROLLER_MISSION_TYPE::mLog->error("Mission controller received a mission item with an index greater than available in the queue.");
        //            }
        return false;
    }

    //execution will only continue if last item
    if(seqReceived < (m_MissionsBeingFetching[key].mission.getQueueSize() - 1))
    {
        return false;
    }

    command_item::AbstractCommandItemPtr newMissionItem = MissionItem::MissionItemFactory::generateAbstractCommandItem(missionItem, target.ModuleID, target.ModuleID);
    m_MissionsBeingFetching[key].mission.replaceMissionItemAtIndex(newMissionItem, seqReceived);

    MissionItem::MissionList::MissionListStatus status = m_MissionsBeingFetching[key].mission.getMissionListStatus();
    if(status.state == MissionItem::MissionList::INCOMPLETE)
    {
        throw std::runtime_error("Reached end of request but missions are not completed");
    }

    //        if(CONTROLLER_MISSION_TYPE::mLog)
    //        {
    //            std::stringstream buffer;
    //            buffer << key;
    //            CONTROLLER_MISSION_TYPE::mLog->info("Mission Controller has received the entire mission of " + std::to_string(m_MissionsBeingFetching[key].mission.getQueueSize()) + " for mission " + buffer.str() + ".");
    //        }

    ackMission.target_system = static_cast<uint8_t>(key.m_systemID);
    ackMission.mission_creator = static_cast<uint8_t>(key.m_creatorID);
    ackMission.mission_id = static_cast<uint8_t>(key.m_missionID);
    ackMission.mission_type = static_cast<uint8_t>(key.m_missionType);

    //KEN This is a hack but for now
    if(key.m_missionState == MissionItem::MISSIONSTATE::PROPOSED)
    {
        m_MissionsBeingFetching[key].mission.setMissionTXState(MissionItem::MISSIONSTATE::RECEIVED);
    }
    else
    {

    }

    finalKey = key;
    finalList = m_MissionsBeingFetching[key].mission;
    m_MissionsBeingFetching.erase(key);

    std::cout << "Mission Controller: Sending Final ACK" << " S_ID: " << (int)missionItem.target_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

    return true;
}


bool ControllerMission::Finish_Receive(const mavlink_mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MissionItem::MissionKey &queueObj)
{
    MaceLog::Green("In Finish_Receive (mission key)");

    MissionItem::MissionKey key(missionItem.target_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type));

    //if we aren't uploading a mission then there is nothing to ack.
    if(m_MissionsUploading.find(sender) == m_MissionsUploading.cend())
    {
        return false;
    }
    if(m_MissionsUploading.at(sender).find(key) == m_MissionsUploading.at(sender).cend())
    {
        return false;
    }

    queueObj = key;

    ack = missionItem.type;

    std::cout << "Mission Controller: Received Final ACK" << " S_ID: " << (int)missionItem.target_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

    return true;
}


void ControllerMission::Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_mission_request_list_t &msg, MaceCore::ModuleCharacteristic &queueObj)
{
    MaceLog::Green("In Request_Construct");

    throw std::runtime_error("No Longer supported, need to pull the correct mission_system");
    UNUSED(sender);
    msg.target_system = static_cast<uint8_t>(target.ModuleID);
    msg.mission_type = static_cast<uint8_t>(MissionItem::MISSIONSTATE::CURRENT);

    m_GenericRequester = sender;

    queueObj = target;

    std::cout << "Mission Controller: Sending mission request" << " S_ID: " << (int)target.ModuleID << std::endl;
}


bool ControllerMission::BuildData_Send(const mavlink_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_count_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &responseQueueObj)
{

    MaceLog::Green("In BuildData_Send (mission request list), (mission count)");

    MissionItem::MISSIONSTATE state = MissionItem::MISSIONSTATE::CURRENT;
    if(state == MissionItem::MISSIONSTATE::CURRENT)
    {
        DataItem<MissionKey, MissionList>::FetchModuleReturn items;

        vehicleObj  = this->GetKeyFromSecondaryID(msg.target_system);

        Controllers::DataItem<MissionItem::MissionKey, MissionItem::MissionList>::FetchFromModule(vehicleObj, items);

        //no modules reported back!
        if(items.size() == 0)
        {
            throw std::runtime_error("No modules reported back");
        }

        //too many modules reported back!
        if(items.size() > 1)
        {
            throw std::runtime_error("More than one module reported");
        }

        std::vector<std::tuple<MissionKey, MissionList>> vec = std::get<1>(items.at(0));
        if(vec.size() == 0)
        {
            return false;
        }
        if(vec.size() == 1)
        {
            MissionItem::MissionKey key = std::get<0>(vec.at(0));
            receiveQueueObj = key;
            responseQueueObj = key;

            // TODO-PAT: Added this if statement to insert a blank mission over EL. Is this valid?
            if(m_MissionsUploading.find(sender) == m_MissionsUploading.cend())
            {
                m_MissionsUploading.insert({sender, {}});
            }
            else if(m_MissionsUploading.find(sender) != m_MissionsUploading.cend())
            {
                if(m_MissionsUploading.at(sender).find(key) != m_MissionsUploading.at(sender).cend())
                {
                    std::cout << "Mission Upload Progress: The mission that was requested to be transmitted is already being transmitted" << std::endl;
                    return false;
                }
            }
            MissionList mission = std::get<1>(vec.at(0));
            m_MissionsUploading.at(sender).insert({key, mission});


            response.count = m_MissionsUploading.at(sender).at(key).getQueueSize();
            response.target_system = m_MissionsUploading.at(sender).at(key).getVehicleID();
            response.mission_creator = static_cast<uint8_t>(key.m_creatorID);
            response.mission_id = static_cast<uint8_t>(key.m_missionID);
            response.mission_type = static_cast<uint8_t>(key.m_missionType);

            std::cout << "Mission Controller: Sending Mission Count of " << response.count << " for S_ID: " << (int)key.m_systemID << " M_ID: " << (int)key.m_missionID << std::endl;

            return true;
        }
        if(vec.size() > 1)
        {
            throw std::runtime_error("Multiple missions reported back, this is a non-op");
        }
    }
    return false;
}


bool ControllerMission::BuildData_Send(const mavlink_mission_request_list_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
{
    MaceLog::Green("In BuildData_Send (mission request list), (mission ack)");

    UNUSED(sender);
    UNUSED(receiveQueueObj);
    UNUSED(respondQueueObj);

    MissionItem::MISSIONSTATE state = MissionItem::MISSIONSTATE::CURRENT;
    if(state == MissionItem::MISSIONSTATE::CURRENT)
    {
        DataItem<MissionKey, MissionList>::FetchModuleReturn items;

        vehicleObj = this->GetKeyFromSecondaryID(msg.target_system);

        Controllers::DataItem<MissionItem::MissionKey, MissionItem::MissionList>::FetchFromModule(vehicleObj, items);

        //no modules reported back!
        if(items.size() == 0)
        {
            throw std::runtime_error("No modules reported back");
        }

        //too many modules reported back!
        if(items.size() > 1)
        {
            throw std::runtime_error("More than one module reported");
        }

        std::vector<std::tuple<MissionKey, MissionList>> vec = std::get<1>(items.at(0));
        if(vec.size() == 0)
        {
            response.target_system = msg.target_system;
            response.type = MAV_MISSION_RESULT::MAV_MISSION_INVALID;

            std::cout << "Mission Controller: Received request list, no missions so sending ack" << std::endl;

            return true;
        }
        if(vec.size() == 1)
        {
            return false;
        }
        if(vec.size() > 1)
        {
            throw std::runtime_error("Multiple missions reported back, this is a non-op");
        }
    }
    return false;
}


bool ControllerMission::Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_new_onboard_mission_t &msg, MaceCore::ModuleCharacteristic &queue)
{
    MaceLog::Green("In Construct_Send (mission key), (new onboard mission)");

    UNUSED(sender);

    queue = target;

    msg.mission_creator = static_cast<uint8_t>(data.m_creatorID);
    msg.mission_id = static_cast<uint8_t>(data.m_missionID);
    msg.mission_type = static_cast<uint8_t>(data.m_missionType);
    msg.mission_system = static_cast<uint8_t>(data.m_systemID);
    msg.mission_state = static_cast<uint8_t>(data.m_missionState);

    std::cout << "Mission Controller: Sending New Mission Notification" << " S_ID: " << (int)data.m_systemID << " M_ID: " << (int)data.m_missionID << std::endl;

    return true;
}


bool ControllerMission::Construct_FinalObjectAndResponse(const mavlink_new_onboard_mission_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_mace_mission_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &dataKey, MissionKey &data)
{
    MaceLog::Green("In Construct_FinalObjectAndResponse (new onboard mission)");

    MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));

    module_from = this->GetHostKey();

    dataKey = sender;
    data = key;


    ack.target_system = static_cast<uint8_t>(key.m_systemID);
    ack.mission_creator = static_cast<uint8_t>(key.m_creatorID);
    ack.mission_id = static_cast<uint8_t>(key.m_missionID);
    ack.mission_type = static_cast<uint8_t>(key.m_missionType);

    std::cout << "Mission Controller: Received New Mission Notification" << " S_ID: " << (int)key.m_systemID << " M_ID: " << (int)key.m_missionID << std::endl;

    return true;
}


bool ControllerMission::Finish_Receive(const mavlink_mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
{
    MaceLog::Green("In Finish_Receive (module characteristic)");
    MissionItem::MissionKey key(missionItem.target_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type));

    //The final mission upload, and notificaiton of received mission notification use the same message
    //So we must determine what we are doing
    //if we are uploading a mission then there is nothing to ack. (This code is to only return true for an ack on notificaiton, not download)
    if(m_MissionsUploading.find(sender) != m_MissionsUploading.cend())
    {
        if(m_MissionsUploading.at(sender).find(key) != m_MissionsUploading.at(sender).cend())
        {
            return false;
        }
    }

    queueObj = sender;

    ack = 0;

    std::cout << "Mission Controller: Received Notification ACK" << " S_ID: " << (int)missionItem.target_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

    return true;
}

ControllerMission::ControllerMission(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
    CONTROLLER_MISSION_TYPE(cb, queue, linkChan, "Mission"),
    SendHelper_RequestMissionDownload(this, ModuleToSysIDCompIDConverter<mavlink_mission_request_list_t>(mavlink_msg_mission_request_list_encode_chan)),
    SendHelper_RequestList(this, mavlink_msg_mission_request_list_decode, ModuleToSysIDCompIDConverter<mavlink_mace_mission_count_t>(mavlink_msg_mace_mission_count_encode_chan)),
    SendHelper_ReceiveCountRespondItemRequest(this, mavlink_msg_mace_mission_count_decode, ModuleToSysIDCompIDConverter<mavlink_mace_mission_request_int_t>(mavlink_msg_mace_mission_request_int_encode_chan)),
    SendHelper_RequestItem(this, mavlink_msg_mace_mission_request_int_decode, ModuleToSysIDCompIDConverter<mavlink_mace_mission_item_int_t>(mavlink_msg_mace_mission_item_int_encode_chan)),
    SendHelper_ReceiveItem(this,
                           [this](const mavlink_mace_mission_request_int_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_ReceiveCountRespondItemRequest::NextTransmission(A,B,C,D);},
mavlink_msg_mace_mission_item_int_decode),
    SendHelper_Final(this, mavlink_msg_mace_mission_item_int_decode, ModuleToSysIDCompIDConverter<mavlink_mace_mission_ack_t>(mavlink_msg_mace_mission_ack_encode_chan)),
    SendHelper_FinalFinal(this, mavlink_msg_mace_mission_ack_decode),
    Action_RequestCurrentMission_Initiate(this, ModuleToSysIDCompIDConverter<mavlink_mission_request_list_t>(mavlink_msg_mission_request_list_encode_chan)),
    Action_RequestCurrentMission_Response(this,
                                          [this](const mavlink_mace_mission_count_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RequestList::NextTransmission(A,B,C,D);},
mavlink_msg_mission_request_list_decode),
    Action_RequestCurrentMission_NoMissionResponse(this,
                                                   [this](const mavlink_mace_mission_ack_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_Final::FinalResponse(A,B,C,D);},
mavlink_msg_mission_request_list_decode),

    NotifyRemoteOfMission(this,
                          ModuleToSysIDCompIDConverter<mavlink_new_onboard_mission_t>(mavlink_msg_new_onboard_mission_encode_chan)),

    UsolicitedReceiveMissionNotification(this,
                                         mavlink_msg_new_onboard_mission_decode,
                                         ModuleToSysIDCompIDConverter<mavlink_mace_mission_ack_t>(mavlink_msg_mace_mission_ack_encode_chan)),
    NotifyRemoteOfMissionFinish(this,
                                mavlink_msg_mace_mission_ack_decode)

{

}


void ControllerMission::RequestMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
{
    SendHelper_RequestMissionDownload::Send(key, sender, target);
}

void ControllerMission::RequestCurrentMission(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
{
    Action_RequestCurrentMission_Initiate::Request(sender, target);
}

void ControllerMission::NotifyOfMission(const MissionItem::MissionKey &key, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target)
{
    NotifyRemoteOfMission::Send(key, sender, target);
}

}
