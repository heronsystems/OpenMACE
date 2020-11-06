#include "controller_mission.h"

namespace ExternalLink {


    //!
    //! \brief Called when building mavlink packet initial request to a mission
    //! \param data
    //! \param cmd
    //!
    bool ControllerMission::Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_list_t &cmd, MissionItem::MissionKey &queueObj)
    {
        UNUSED(target);

        queueObj = data;

        cmd.mission_creator = static_cast<uint8_t>(data.m_creatorID);
        cmd.mission_id = static_cast<uint8_t>(data.m_missionID);
        cmd.mission_system = static_cast<uint8_t>(data.m_systemID);
        cmd.mission_type = static_cast<uint8_t>(data.m_missionType);
        cmd.mission_state = static_cast<uint8_t>(data.m_missionState);

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








    bool ControllerMission::BuildData_Send(const mace_mission_count_t &mission, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(mission.mission_system,mission.mission_creator,mission.mission_id,static_cast<MissionItem::MISSIONTYPE>(mission.mission_type),static_cast<MissionItem::MISSIONSTATE>(mission.mission_state));
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
        request.mission_system = mission.mission_system;
        request.mission_type = mission.mission_type;
        request.mission_state = mission.mission_state;
        request.target_system = mission.target_system;
        request.seq = 0;

        std::cout << "Mission Controller: Requesting Item " << 0 << " S_ID: " << (int)mission.mission_system << " M_ID: " << (int)mission.mission_id << std::endl;

        return true;
    }



    bool ControllerMission::BuildData_Send(const mace_mission_request_item_t &missionRequest, const MaceCore::ModuleCharacteristic &sender, mace_mission_item_t &missionItem, MaceCore::ModuleCharacteristic &moduleFrom, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        MissionItem::MissionKey key(missionRequest.mission_system,missionRequest.mission_creator,missionRequest.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionRequest.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionRequest.mission_state));
        receiveQueueObj = key;
        respondQueueObj = key;

        moduleFrom = this->GetKeyFromSecondaryID(missionRequest.mission_system);

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

        MissionItem::MissionItemFactory::generateMACEMissionItem(ptrItem,index,missionItem);
        MissionItem::MissionItemFactory::updateMissionKey(key, missionItem);

        std::cout << "Mission Controller: Sending Item " << index << " S_ID: " << (int)missionRequest.mission_system << " M_ID: " << (int)missionRequest.mission_id << std::endl;

        return true;
    }








    bool ControllerMission::BuildData_Send(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_request_item_t &request, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        //MTB this isn't quite right, but I think it only effects more or less data fields that are not used.
        MaceCore::ModuleCharacteristic target;
        target.ModuleID = missionItem.target_system;
        target.MaceInstance = 0;

        MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));
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
        request.mission_system =  static_cast<uint8_t>(key.m_systemID);
        request.mission_type =  static_cast<uint8_t>(key.m_missionType);
        request.mission_state =  static_cast<uint8_t>(key.m_missionState);
        request.seq =  static_cast<uint16_t>(indexRequest);

        std::cout << "Mission Controller: Requesting Item " << indexRequest << " S_ID: " << (int)missionItem.mission_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

        return true;
    }





    bool ControllerMission::Construct_FinalObjectAndResponse(const mace_mission_item_t &missionItem, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &ackMission, MissionItem::MissionKey &finalKey, MissionItem::MissionList &finalList, MaceCore::ModuleCharacteristic &moduleFor, MissionItem::MissionKey &queueObj)
    {
        UNUSED(sender);
        //MTB this isn't quite right, but I think it only effects more or less data fields that are not used.
        MaceCore::ModuleCharacteristic target;
        target.ModuleID = missionItem.target_system;
        target.MaceInstance = 0;

        MissionItem::MissionKey key(missionItem.target_system,missionItem.mission_creator,missionItem.mission_id,static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type),static_cast<MissionItem::MISSIONSTATE>(missionItem.mission_state));
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

        ackMission.mission_system = static_cast<uint8_t>(key.m_systemID);
        ackMission.mission_creator = static_cast<uint8_t>(key.m_creatorID);
        ackMission.mission_id = static_cast<uint8_t>(key.m_missionID);
        ackMission.mission_type = static_cast<uint8_t>(key.m_missionType);
        ackMission.prev_mission_state = static_cast<uint8_t>(key.m_missionState);
        ackMission.mission_result = MAV_MISSION_ACCEPTED;

        //KEN This is a hack but for now
        if(key.m_missionState == MissionItem::MISSIONSTATE::PROPOSED)
        {
            ackMission.cur_mission_state = (uint8_t)MissionItem::MISSIONSTATE::RECEIVED;
            m_MissionsBeingFetching[key].mission.setMissionTXState(MissionItem::MISSIONSTATE::RECEIVED);
        }
        else
        {
            ackMission.cur_mission_state = (uint8_t)key.m_missionState;
        }

        finalKey = key;
        finalList = m_MissionsBeingFetching[key].mission;
        m_MissionsBeingFetching.erase(key);

        std::cout << "Mission Controller: Sending Final ACK" << " S_ID: " << (int)missionItem.mission_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

        return true;
    }

    bool ControllerMission::Finish_Receive(const mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MissionItem::MissionKey &queueObj)
    {
        MissionItem::MissionKey key(missionItem.mission_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type), static_cast<MissionItem::MISSIONSTATE>(missionItem.cur_mission_state));

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

        ack = missionItem.mission_result;

        std::cout << "Mission Controller: Received Final ACK" << " S_ID: " << (int)missionItem.mission_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

        return true;
    }


    void ControllerMission::Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_list_generic_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        throw std::runtime_error("No Longer supported, need to pull the correct mission_system");
        UNUSED(sender);
        msg.mission_system = static_cast<uint8_t>(target.ModuleID);
        msg.mission_type = static_cast<uint8_t>(MissionItem::MISSIONSTATE::CURRENT);
        msg.mission_state = 0;

        m_GenericRequester = sender;

        queueObj = target;

        std::cout << "Mission Controller: Sending mission request" << " S_ID: " << (int)target.ModuleID << std::endl;
    }

    bool ControllerMission::BuildData_Send(const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_count_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &responseQueueObj)
    {
        MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
        if(state == MissionItem::MISSIONSTATE::CURRENT)
        {
            DataItem<MissionKey, MissionList>::FetchModuleReturn items;

            vehicleObj  = this->GetKeyFromSecondaryID(msg.mission_system);

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

                if(m_MissionsUploading.find(sender) != m_MissionsUploading.cend())
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
                response.mission_system = static_cast<uint8_t>(key.m_systemID);
                response.mission_creator = static_cast<uint8_t>(key.m_creatorID);
                response.mission_id = static_cast<uint8_t>(key.m_missionID);
                response.mission_type = static_cast<uint8_t>(key.m_missionType);
                response.mission_state = static_cast<uint8_t>(key.m_missionState);

                std::cout << "Mission Controller: Sending Mission Count" << " S_ID: " << (int)key.m_systemID << " M_ID: " << (int)key.m_missionID << std::endl;

                return true;
            }
            if(vec.size() > 1)
            {
                throw std::runtime_error("Multiple missions reported back, this is a non-op");
            }
        }
        return false;
    }


    bool ControllerMission::BuildData_Send(const mace_mission_request_list_generic_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &response, MaceCore::ModuleCharacteristic &vehicleObj, MissionItem::MissionKey &receiveQueueObj, MissionItem::MissionKey &respondQueueObj)
    {
        UNUSED(sender);
        UNUSED(receiveQueueObj);
        UNUSED(respondQueueObj);

        MissionItem::MISSIONSTATE state = static_cast<MissionItem::MISSIONSTATE>(msg.mission_state);
        if(state == MissionItem::MISSIONSTATE::CURRENT)
        {
            DataItem<MissionKey, MissionList>::FetchModuleReturn items;

            vehicleObj = this->GetKeyFromSecondaryID(msg.mission_system);

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
                response.mission_system = msg.mission_system;
                response.cur_mission_state = msg.mission_state;
                response.mission_result = static_cast<uint8_t>(MissionItem::MissionACK::MISSION_RESULT::MISSION_RESULT_DOES_NOT_EXIST);

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


    bool ControllerMission::Construct_Send(const MissionItem::MissionKey &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_new_onboard_mission_t &msg, MaceCore::ModuleCharacteristic &queue)
    {
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


    bool ControllerMission::Construct_FinalObjectAndResponse(const mace_new_onboard_mission_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_mission_ack_t &ack, MaceCore::ModuleCharacteristic &module_from, MaceCore::ModuleCharacteristic &dataKey, MissionKey &data)
    {        
        MissionItem::MissionKey key(msg.mission_system, msg.mission_creator, msg.mission_id, static_cast<MissionItem::MISSIONTYPE>(msg.mission_type), static_cast<MissionItem::MISSIONSTATE>(msg.mission_state));

        module_from = this->GetHostKey();

        dataKey = sender;
        data = key;


        ack.mission_system = static_cast<uint8_t>(key.m_systemID);
        ack.mission_creator = static_cast<uint8_t>(key.m_creatorID);
        ack.mission_id = static_cast<uint8_t>(key.m_missionID);
        ack.mission_type = static_cast<uint8_t>(key.m_missionType);
        ack.prev_mission_state = static_cast<uint8_t>(key.m_missionState);
        ack.mission_result = MAV_MISSION_ACCEPTED;

        std::cout << "Mission Controller: Received New Mission Notification" << " S_ID: " << (int)key.m_systemID << " M_ID: " << (int)key.m_missionID << std::endl;

        return true;
    }


    bool ControllerMission::Finish_Receive(const mace_mission_ack_t &missionItem, const MaceCore::ModuleCharacteristic &sender, uint8_t & ack, MaceCore::ModuleCharacteristic &queueObj)
    {
        MissionItem::MissionKey key(missionItem.mission_system, missionItem.mission_creator, missionItem.mission_id, static_cast<MissionItem::MISSIONTYPE>(missionItem.mission_type), static_cast<MissionItem::MISSIONSTATE>(missionItem.cur_mission_state));

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

        std::cout << "Mission Controller: Received Notification ACK" << " S_ID: " << (int)missionItem.mission_system << " M_ID: " << (int)missionItem.mission_id << std::endl;

        return true;
    }

    ControllerMission::ControllerMission(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        CONTROLLER_MISSION_TYPE(cb, queue, linkChan),
        SendHelper_RequestMissionDownload(this, ModuleToSysIDCompIDConverter<mace_mission_request_list_t>(mace_msg_mission_request_list_encode_chan)),
        SendHelper_RequestList(this, mace_msg_mission_request_list_decode, ModuleToSysIDCompIDConverter<mace_mission_count_t>(mace_msg_mission_count_encode_chan)),
        SendHelper_ReceiveCountRespondItemRequest(this, mace_msg_mission_count_decode, ModuleToSysIDCompIDConverter<mace_mission_request_item_t>(mace_msg_mission_request_item_encode_chan)),
        SendHelper_RequestItem(this, mace_msg_mission_request_item_decode, ModuleToSysIDCompIDConverter<mace_mission_item_t>(mace_msg_mission_item_encode_chan)),
        SendHelper_ReceiveItem(this,
                               [this](const mace_mission_request_item_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_ReceiveCountRespondItemRequest::NextTransmission(A,B,C,D);},
                               mace_msg_mission_item_decode),
        SendHelper_Final(this, mace_msg_mission_item_decode, ModuleToSysIDCompIDConverter<mace_mission_ack_t>(mace_msg_mission_ack_encode_chan)),
        SendHelper_FinalFinal(this, mace_msg_mission_ack_decode),
        Action_RequestCurrentMission_Initiate(this, ModuleToSysIDCompIDConverter<mace_mission_request_list_generic_t>(mace_msg_mission_request_list_generic_encode_chan)),
        Action_RequestCurrentMission_Response(this,
                                [this](const mace_mission_count_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_RequestList::NextTransmission(A,B,C,D);},
                                mace_msg_mission_request_list_generic_decode),
        Action_RequestCurrentMission_NoMissionResponse(this,
                                [this](const mace_mission_ack_t &A, const MaceCore::ModuleCharacteristic &B, const MissionItem::MissionKey &C, const MaceCore::ModuleCharacteristic &D){SendHelper_Final::FinalResponse(A,B,C,D);},
                                mace_msg_mission_request_list_generic_decode),

        NotifyRemoteOfMission(this,
                                ModuleToSysIDCompIDConverter<mace_new_onboard_mission_t>(mace_msg_new_onboard_mission_encode_chan)),

        UsolicitedReceiveMissionNotification(this,
                                             mace_msg_new_onboard_mission_decode,
                                             ModuleToSysIDCompIDConverter<mace_mission_ack_t>(mace_msg_mission_ack_encode_chan)),
        NotifyRemoteOfMissionFinish(this,
                                    mace_msg_mission_ack_decode)

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
