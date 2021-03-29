#include "controller_home.h"

namespace ExternalLink {

void ControllerHome::Construct_Broadcast(const command_item::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, mavlink_home_position_t &msg)
{
    UNUSED(sender);
    UNUSED(data);
    UNUSED(msg);
    const mace::pose::Position* homePosition = data.getPosition();

    switch (homePosition->getCoordinateSystemType()) {
    case CoordinateSystemTypes::CARTESIAN:
    {
        std::cout<<"We are asking to transmit a local home position, this is not currently supported."<<std::endl;
        break;
    }
    case CoordinateSystemTypes::GEODETIC:
    {
        const mace::pose::GeodeticPosition_3D* currentPosition = homePosition->positionAs<mace::pose::GeodeticPosition_3D>();
        msg.latitude = currentPosition->getLatitude() * pow(10,7);
        msg.longitude = currentPosition->getLongitude() * pow(10,7);
        msg.altitude = currentPosition->getAltitude() * pow(10,3);
        break;
    }
    default:
        break;
    }

    std::cout << "Home Controller: Broadcasting Home" << std::endl;
}


/**
     * @brief Contruct a SpatialHome object from broadcasted home position
     * @param vehicleObj
     * @return
     */
bool ControllerHome::Construct_FinalObject(const mavlink_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data)
{
    UNUSED(msg);

    //If we have requested home position received module then don't do anything.
    // (Because this handles broadcast, let other method handle this case)
    if(m_ModulesRequestedFrom.find(sender) != m_ModulesRequestedFrom.cend())
    {
        return false;
    }

    key = sender;
    mace::pose::GeodeticPosition_3D currentPosition(msg.latitude/pow(10,7), msg.longitude/pow(10,7), msg.altitude/pow(10,3));
    data.setPosition(&currentPosition);

    std::cout << "Home Controller: Received broadcasted home" << std::endl;

    return true;
}

void ControllerHome::Request_Construct(const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_request_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj)
{
    UNUSED(sender);
    msg.target_system = static_cast<uint8_t>(target.ModuleID);

    queueObj = target;

    m_ModulesRequestedFrom.insert({target, sender});

    std::cout << "Home Controller: Sending home request" << std::endl;
}


bool ControllerHome::BuildData_Send(const mavlink_request_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_home_position_t &rsp, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &receiveQueueObj, MaceCore::ModuleCharacteristic &respondQueueObj)
{
    receiveQueueObj = sender;
    respondQueueObj = receiveQueueObj;

    vehicleObj = GetKeyFromSecondaryID(msg.target_system);


    std::vector<std::tuple<MaceCore::ModuleCharacteristic, command_item::SpatialHome>> homes;
    this->FetchDataFromKey(vehicleObj, homes);

    command_item::SpatialHome homeToSend = std::get<1>(homes.at(0));


    const mace::pose::Position* homePosition = homeToSend.getPosition();

    switch (homePosition->getCoordinateSystemType()) {
    case CoordinateSystemTypes::CARTESIAN:
    {
        std::cout<<"We are asking to build the data send a local position which is not currently supported."<<std::endl;
        break;
    }
    case CoordinateSystemTypes::GEODETIC:
    {
        const mace::pose::GeodeticPosition_3D* currentPosition = homePosition->positionAs<mace::pose::GeodeticPosition_3D>();
        rsp.latitude = currentPosition->getLatitude() * pow(10,7);
        rsp.longitude = currentPosition->getLongitude() * pow(10,7);
        rsp.altitude = currentPosition->getAltitude() * pow(10,3);

        break;
    }
    default:
        break;
    }

    std::cout << "Home Controller: Receive home request, sending home position" << std::endl;

    return true;
}

bool ControllerHome::Construct_FinalObjectAndResponse(const mavlink_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_home_position_ack_t &response, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
{
    //Only continue if we have requested a home posiiton from this module.
    if(m_ModulesRequestedFrom.find(sender) == m_ModulesRequestedFrom.cend())
    {
        return false;
    }
    vehicleObj = m_ModulesRequestedFrom.at(sender);
    m_ModulesRequestedFrom[sender] = sender;

    queueObj = sender;

    key = sender;

    mace::pose::GeodeticPosition_3D currentPosition(msg.latitude/pow(10,7), msg.longitude/pow(10,7), msg.altitude/pow(10,3));
    data.setPosition(&currentPosition);

    data.setTargetSystem(sender.ModuleID);
    data.setOriginatingSystem(sender.ModuleID);

    response.target_system = sender.ModuleID;

    std::cout << "Home Controller: Receive home position, sending ack" << std::endl;

    return true;
}


bool ControllerHome::Finish_Receive(const mavlink_home_position_ack_t &ack, const MaceCore::ModuleCharacteristic &sender, uint8_t &ack_code, MaceCore::ModuleCharacteristic &queueObj)
{
    queueObj = sender;
    ack_code = ack.ack;

    std::cout << "Home Controller: Receive ACK, done" << std::endl;

    return true;
}


bool ControllerHome::Construct_Send(const command_item::SpatialHome &data, const MaceCore::ModuleCharacteristic &sender, const MaceCore::ModuleCharacteristic &target, mavlink_set_home_position_t &msg, MaceCore::ModuleCharacteristic &queueObj)
{
    UNUSED(sender);
    UNUSED(target);

    const mace::pose::Position* homePosition = data.getPosition();
    msg.target_system = data.getTargetSystem();
    if(homePosition != nullptr)
    {
        const mace::pose::GeodeticPosition_3D* currentPosition = homePosition->positionAs<mace::pose::GeodeticPosition_3D>();
        msg.latitude = currentPosition->getLatitude() * pow(10,7);
        msg.longitude = currentPosition->getLongitude() * pow(10,7);
        msg.altitude = currentPosition->getAltitude() * pow(10,3);
    }

    queueObj = GetKeyFromSecondaryID(data.getTargetSystem());

    std::cout << "Home Controller: Sending set home position" << std::endl;

    return true;
}

bool ControllerHome::Construct_FinalObjectAndResponse(const mavlink_set_home_position_t &msg, const MaceCore::ModuleCharacteristic &sender, mavlink_home_position_ack_t &ack, MaceCore::ModuleCharacteristic &key, command_item::SpatialHome &data, MaceCore::ModuleCharacteristic &vehicleObj, MaceCore::ModuleCharacteristic &queueObj)
{
    vehicleObj = GetKeyFromSecondaryID(msg.target_system);

    queueObj = sender;

    key = sender;
    data.setOriginatingSystem(sender.ModuleID);

    mace::pose::GeodeticPosition_3D currentPosition(msg.latitude/pow(10,7), msg.longitude/pow(10,7), msg.altitude/pow(10,3));
    data.setPosition(&currentPosition);

    data.setTargetSystem(msg.target_system);
    data.setOriginatingSystem(msg.target_system);

    //std::cout << "DEBUG: Received SetHomePosition. Final XYZ Values: " << data.position->getX() << " " << data.position->getY() << " " << data.position->getZ() << std::endl;

    ack.target_system = msg.target_system;

    std::cout << "Home Controller: Receive new set home, sending ack" << std::endl;

    return true;
}

ControllerHome::ControllerHome(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue * queue, int linkChan) :
    CONTROLLER_HOME_TYPE(cb, queue, linkChan, "Home", false),
    ControllerHome_Step_BroadcastHome(this, ModuleToSysIDCompIDConverter<mavlink_home_position_t>(mavlink_msg_home_position_encode_chan)),
    ControllerHome_Step_ReceiveBroadcastedHome(this, mavlink_msg_home_position_decode),
    ControllerHome_Step_RequestHome(this, ModuleToSysIDCompIDConverter<mavlink_request_home_position_t>(mavlink_msg_request_home_position_encode_chan)),
    ControllerHome_Step_ReceiveHomeRequest(this, mavlink_msg_request_home_position_decode, ModuleToSysIDCompIDConverter<mavlink_home_position_t>(mavlink_msg_home_position_encode_chan)),
    ControllerHome_Step_ReceiveHomePositionSendAck(this, mavlink_msg_home_position_decode, ModuleToSysIDCompIDConverter<mavlink_home_position_ack_t>(mavlink_msg_home_position_ack_encode_chan)),
    ControllerHome_Step_ReceiveFinishingAck(this, mavlink_msg_home_position_ack_decode),
    ControllerHome_Step_SendHomePosition(this, ModuleToSysIDCompIDConverter<mavlink_set_home_position_t>(mavlink_msg_set_home_position_encode_chan)),
    ControllerHome_Step_ReceiveSetHomeSendACK(this, mavlink_msg_set_home_position_decode, ModuleToSysIDCompIDConverter<mavlink_home_position_ack_t>(mavlink_msg_home_position_ack_encode_chan))
{

}

}
