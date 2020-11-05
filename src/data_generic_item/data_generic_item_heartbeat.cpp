#include "data_generic_item_heartbeat.h"

namespace DataGenericItem {

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat() :
    autopilot(Data::AutopilotType::AUTOPILOT_TYPE_GENERIC), protocol(Data::CommsProtocol::COMMS_MACE), type(Data::SystemType::SYSTEM_TYPE_GENERIC),
    missionState(Data::MissionExecutionState::MESTATE_UNEXECUTED), maceCompanion(true), currentHSMState(Data::MACEHSMState::STATE_UNKNOWN)
{

}

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat(const DataGenericItem_Heartbeat &copyObj)
{
    this->protocol = copyObj.getProtocol();
    this->type = copyObj.getType();
    this->autopilot = copyObj.getAutopilot();
    this->missionState = copyObj.getMissionState();
    this->maceCompanion = copyObj.getCompanion();
    this->mavlinkID = copyObj.getMavlinkID();
    this->currentHSMState = copyObj.getHSMState();
}


mace_heartbeat_t DataGenericItem_Heartbeat::getMACECommsObject() const
{
    mace_heartbeat_t rtnObj;

    rtnObj.autopilot = (uint8_t)this->autopilot;
    rtnObj.mace_companion = this->maceCompanion;
    rtnObj.protocol = (uint8_t)this->protocol;
    rtnObj.type = (uint8_t)this->type;
    rtnObj.mavlinkID = this->mavlinkID;
//    rtnObj.currentHSMState = (uint8_t)this->currentHSMState; // TODO: Edit Heartbeat mavlink message to include HSM state

    return rtnObj;
}

mace_message_t DataGenericItem_Heartbeat::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_heartbeat_t heartbeat = getMACECommsObject();
    mace_msg_heartbeat_encode_chan(systemID,compID,chan,&msg,&heartbeat);
    return msg;
}

QJsonObject DataGenericItem_Heartbeat::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["autopilot"] = QString::fromStdString(Data::AutopilotTypeToString(getAutopilot()));
    json["vehicle_type"] = QString::fromStdString(Data::SystemTypeToString(getType()));
    json["companion"] = getCompanion();
    json["protocol"] = QString::fromStdString(Data::CommsProtocolToString(getProtocol()));
    json["mission_state"] = (uint8_t)getMissionState();
    json["mavlink_id"] = getMavlinkID();

    // TODO: Populate:
    json["behavior_state"] = ""; // Meant to be the behavior execution state?
    json["vehicle_state"] = QString::fromStdString(Data::MACEHSMStateToString(getHSMState())); // This can be where we store its state machine state. Can be used for button state machine on GUI

    return json;
}

} //end of namespace DataGenericItem
