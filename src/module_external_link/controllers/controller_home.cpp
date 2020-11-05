#include "controller_home.h"

namespace ExternalLink {



    void ControllerHome::Construct_Broadcast(const command_item::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &msg)
    {
        UNUSED(sender);
        UNUSED(data);
        UNUSED(msg);
//        msg.latitude = data.position->getX() * pow(10,7);
//        msg.longitude = data.position->getY()* pow(10,7);
//        msg.altitude = data.position->getZ() * 1000.0;
//        msg.x = 0;
//        msg.y = 0;
//        msg.z = 0;
//        msg.q[0] = 0;
//        msg.q[1] = 0;
//        msg.q[2] = 0;
//        msg.q[3] = 0;
//        msg.approach_x = 0;
//        msg.approach_y = 0;
//        msg.approach_z = 0;
//        msg.validity = 0;

        std::cout << "Home Controller: Broadcasting Home" << std::endl;
    }


    /**
     * @brief Contruct a SpatialHome object from broadcasted home position
     * @param vehicleObj
     * @return
     */
    bool ControllerHome::Construct_FinalObject(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data)
    {
        UNUSED(msg);
        UNUSED(data);

        //If we have requested home position received module then don't do anything.
        // (Because this handles broadcast, let other method handle this case)
        if(m_ModulesRequestedFrom.find(sender) != m_ModulesRequestedFrom.cend())
        {
            return false;
        }

        key = sender;
//        data.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
//        data.position->setX(msg.latitude / pow(10,7));
//        data.position->setY(msg.longitude / pow(10,7));
//        data.position->setZ(msg.altitude / pow(10,3));

        std::cout << "Home Controller: Received broadcasted home" << std::endl;

        return true;
    }


    void ControllerHome::Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_mission_request_home_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        msg.target_system = static_cast<uint8_t>(target.ModuleID);

        queueObj = target;

        m_ModulesRequestedFrom.insert({target, sender});

        std::cout << "Home Controller: Sending home request" << std::endl;
    }


    bool ControllerHome::BuildData_Send(const mace_mission_request_home_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_t &rsp, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, MaceCore::ModuleCharacteristic &respondQueueObj)
    {
        UNUSED(rsp);

        receiveQueueObj = sender;
        respondQueueObj = receiveQueueObj;

        vehicleObj = GetKeyFromSecondaryID(msg.target_system);


        std::vector<std::tuple<MaceCore::ModuleCharacteristic, command_item::SpatialHome>> homes;
        this->FetchDataFromKey(vehicleObj, homes);

        command_item::SpatialHome homeToSend = std::get<1>(homes.at(0));
//        rsp.latitude = homeToSend.position->getX() * pow(10,7);
//        rsp.longitude = homeToSend.position->getY() * pow(10,7);
//        rsp.altitude = homeToSend.position->getZ() * pow(10,3);

        std::cout << "Home Controller: Receive home request, sending home position" << std::endl;

        return true;
    }

    bool ControllerHome::Construct_FinalObjectAndResponse(const mace_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &response, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(msg);

        //Only continue if we have requested a home posiiton from this module.
        if(m_ModulesRequestedFrom.find(sender) == m_ModulesRequestedFrom.cend())
        {
            return false;
        }
        vehicleObj = m_ModulesRequestedFrom.at(sender);
        m_ModulesRequestedFrom[sender] = sender;

        queueObj = sender;

        key = sender;
//        data.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
//        data.position->setX(msg.latitude / pow(10,7));
//        data.position->setY(msg.longitude / pow(10,7));
//        data.position->setZ(msg.altitude / pow(10,3));


        data.setTargetSystem(sender.ModuleID);
        data.setOriginatingSystem(sender.ModuleID);

        response.target_system = sender.ModuleID;

        std::cout << "Home Controller: Receive home position, sending ack" << std::endl;

        return true;
    }


    bool ControllerHome::Finish_Receive(const mace_home_position_ack_t &ack, const MaceCore::ModuleCharacteristic &sender, uint8_t &ack_code, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(ack);
        queueObj = sender;
        ack_code = ack.ack;

        std::cout << "Home Controller: Receive ACK, done" << std::endl;

        return true;
    }


    bool ControllerHome::Construct_Send(const command_item::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mace_set_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj)
    {
        UNUSED(sender);
        UNUSED(target);
        UNUSED(msg);

        //std::cout << "DEBUG: Sending SetHomePosition. Raw XYZ Values: " << data.position->getX() << " " << data.position->getY() << " " << data.position->getZ() << std::endl;

//        msg.target_system = data.getTargetSystem();
//        msg.latitude = data.position->getX() * pow(10,7);
//        msg.longitude = data.position->getY()* pow(10,7);
//        msg.altitude = data.position->getZ() * 1000.0;

        queueObj = GetKeyFromSecondaryID(data.getTargetSystem());

        std::cout << "Home Controller: Sending set home position" << std::endl;

        return true;
    }

    bool ControllerHome::Construct_FinalObjectAndResponse(const mace_set_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mace_home_position_ack_t &ack, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
    {
        vehicleObj = GetKeyFromSecondaryID(msg.target_system);

        queueObj = sender;

        key = sender;
//        data.position->setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
//        data.position->setX(msg.latitude / pow(10,7));
//        data.position->setY(msg.longitude / pow(10,7));
//        data.position->setZ(msg.altitude / pow(10,3));
        data.setTargetSystem(msg.target_system);
        data.setOriginatingSystem(msg.target_system);

        //std::cout << "DEBUG: Received SetHomePosition. Final XYZ Values: " << data.position->getX() << " " << data.position->getY() << " " << data.position->getZ() << std::endl;

        ack.target_system = msg.target_system;

        std::cout << "Home Controller: Receive new set home, sending ack" << std::endl;

        return true;
    }

    ControllerHome::ControllerHome(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue * queue, int linkChan) :
        CONTROLLER_HOME_TYPE(cb, queue, linkChan),
        ControllerHome_Step_BroadcastHome(this, ModuleToSysIDCompIDConverter<mace_home_position_t>(mace_msg_home_position_encode_chan)),
        ControllerHome_Step_ReceiveBroadcastedHome(this, mace_msg_home_position_decode),
        ControllerHome_Step_RequestHome(this, ModuleToSysIDCompIDConverter<mace_mission_request_home_t>(mace_msg_mission_request_home_encode_chan)),
        ControllerHome_Step_ReceiveHomeRequest(this, mace_msg_mission_request_home_decode, ModuleToSysIDCompIDConverter<mace_home_position_t>(mace_msg_home_position_encode_chan)),
        ControllerHome_Step_ReceiveHomePositionSendAck(this, mace_msg_home_position_decode, ModuleToSysIDCompIDConverter<mace_home_position_ack_t>(mace_msg_home_position_ack_encode_chan)),
        ControllerHome_Step_ReceiveFinishingAck(this, mace_msg_home_position_ack_decode),
        ControllerHome_Step_SendHomePosition(this, ModuleToSysIDCompIDConverter<mace_set_home_position_t>(mace_msg_set_home_position_encode_chan)),
        ControllerHome_Step_ReceiveSetHomeSendACK(this, mace_msg_set_home_position_decode, ModuleToSysIDCompIDConverter<mace_home_position_ack_t>(mace_msg_home_position_ack_encode_chan))
    {

    }

}

