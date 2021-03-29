#include "data_generic_item_heartbeat.h"

namespace DataGenericItem {

DataGenericItem_Heartbeat::DataGenericItem_Heartbeat() :
    autopilot(MAV_AUTOPILOT_GENERIC), protocol(Data::CommsProtocol::COMMS_MACE), type(MAV_TYPE::MAV_TYPE_GENERIC),
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
    this->flightMode = copyObj.getFlightMode();
    this->armed = copyObj.getArmed();
}


mavlink_mace_heartbeat_t DataGenericItem_Heartbeat::getMACECommsObject() const
{
    mavlink_mace_heartbeat_t rtnObj;

    rtnObj.autopilot = (uint8_t)this->autopilot;
    rtnObj.flight_mode = (uint8_t)this->flightMode;
//    rtnObj.mavlink_version = ;
    rtnObj.vehicle_hsm = (uint8_t)this->currentHSMState;
    rtnObj.type = (uint8_t)this->type;
    rtnObj.vehicle_id = this->mavlinkID;
    rtnObj.armed = this->armed;

    return rtnObj;
}

mavlink_message_t DataGenericItem_Heartbeat::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mavlink_message_t msg;
    mavlink_mace_heartbeat_t heartbeat = getMACECommsObject();
    mavlink_msg_mace_heartbeat_encode_chan(systemID,compID,chan,&msg,&heartbeat);
    return msg;
}

QJsonObject DataGenericItem_Heartbeat::toJSON(const int &vehicleID, const std::string &dataType) const
{
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["autopilot"] = QString::fromStdString(Data::AutopilotTypeToString(getAutopilot()));
    json["vehicle_type"] = QString::fromStdString(Data::MAVTypeToString(getType()));
    json["companion"] = getCompanion();
    json["protocol"] = QString::fromStdString(Data::CommsProtocolToString(getProtocol()));
    json["mission_state"] = QString::fromStdString(Data::MissionExecutionStateToString(getMissionState()));
    json["mavlink_id"] = getMavlinkID();

    // TODO: Populate:
    json["behavior_state"] = ""; // Meant to be the behavior execution state?
    json["vehicle_state"] = QString::fromStdString(Data::MACEHSMStateToString(getHSMState())); // This can be where we store its state machine state. Can be used for button state machine on GUI
    json["flight_mode"] = getFlightMode();
    json["armed"] = getArmed();

    return json;
}

void DataGenericItem_Heartbeat::fromJSON(const QJsonDocument &inputJSON)
{
    this->setAutopilot(Data::AutopilotTypeFromString(inputJSON.object().value("autopilot").toString().toStdString()));
    this->setType(Data::SystemTypeFromString(inputJSON.object().value("vehicle_type").toString().toStdString()));
    this->setCompanion(inputJSON.object().value("companion").toBool());
    this->setProtocol(Data::CommsProtocolFromString(inputJSON.object().value("protocol").toString().toStdString()));
    this->setExecutionState(Data::MissionExecutionStateFromString(inputJSON.object().value("mission_state").toString().toStdString()));
    this->setMavlinkID(inputJSON.object().value("mavlink_id").toInt());
    this->setHSMState(Data::MACEHSMStateFromString(inputJSON.object().value("vehicle_state").toString().toStdString()));
    this->setFlightMode(inputJSON.object().value("flight_mode").toInt());
    this->setArmed(inputJSON.object().value("armed").toBool());
}

std::string DataGenericItem_Heartbeat::toCSV(const std::string &delimiter) const
{
    std::string newline = Data::AutopilotTypeToString(getAutopilot()) + delimiter + Data::MAVTypeToString(getType())+ delimiter + (getCompanion() ? "true" : "false")  + delimiter + Data::CommsProtocolToString(getProtocol()) + delimiter + Data::MissionExecutionStateToString(getMissionState()) + delimiter + std::to_string(getMavlinkID()) + delimiter + Data::MACEHSMStateToString(getHSMState());
    return newline;
}
} //end of namespace DataGenericItem
