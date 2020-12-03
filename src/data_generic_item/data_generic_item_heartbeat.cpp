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
    json["mission_state"] = QString::fromStdString(Data::MissionExecutionStateToString(getMissionState()));
    json["mavlink_id"] = getMavlinkID();

    // TODO: Populate:
    json["behavior_state"] = ""; // Meant to be the behavior execution state?
    json["vehicle_state"] = QString::fromStdString(Data::MACEHSMStateToString(getHSMState())); // This can be where we store its state machine state. Can be used for button state machine on GUI

    return json;
}

void DataGenericItem_Heartbeat::fromJSON(const std::string &inputJSON)
{
    //Pull Values from JSON string
    size_t s = inputJSON.find("autopilot")+12;
    std::string autopilot = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    s = inputJSON.find("vehicle_type")+15;
    std::string vehicletype = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    s = inputJSON.find("companion")+11;
    std::string companion = inputJSON.substr(s, inputJSON.find(",", s) - s );
    s = inputJSON.find("protocol")+11;
    std::string protocol = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    s = inputJSON.find("mission_state")+16;
    std::string missionstate = inputJSON.substr(s, inputJSON.find("\"", s) - s );
    s = inputJSON.find("mavlink_id")+12;
    std::string mavlinkid = inputJSON.substr(s, inputJSON.find(",", s) - s );
    s = inputJSON.find("vehicle_state")+16;
    std::string vehiclestate = inputJSON.substr(s, inputJSON.find("\"", s) - s );

    //Set Values
    this->setAutopilot(Data::AutopilotTypeFromString(autopilot));
    this->setType(Data::SystemTypeFromString(vehicletype));
    if (companion == "true")
        this->setCompanion(true);
    if (companion == "false")
        this->setCompanion(false);
    this->setProtocol(Data::CommsProtocolFromString(protocol));
    this->setExecutionState(Data::MissionExecutionStateFromString(missionstate));
    this->setMavlinkID(std::stoi(mavlinkid));
    this->setHSMState(Data::MACEHSMStateFromString(vehiclestate));
}

std::string DataGenericItem_Heartbeat::toCSV() const
{
    std::string newline = Data::AutopilotTypeToString(getAutopilot()) + "; " + Data::SystemTypeToString(getType())+ "; " + (getCompanion() ? "true" : "false")  + "; " + Data::CommsProtocolToString(getProtocol()) + "; " + Data::MissionExecutionStateToString(getMissionState()) + "; " + std::to_string(getMavlinkID()) + "; " + Data::MACEHSMStateToString(getHSMState()) + "; ";
    return newline;
}
} //end of namespace DataGenericItem
