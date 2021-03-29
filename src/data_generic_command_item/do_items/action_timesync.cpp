#include "action_timesync.h"

namespace command_item {

MAV_CMD Action_Timesync::getCommandType() const
{
//    return MAV_CMD::MAV_CMD_TIMESYNC;
    return MAV_CMD::MAV_CMD_ENUM_END;
}

std::string Action_Timesync::getDescription() const
{
    return "This command will send timesync data to the vehicle";
}

bool Action_Timesync::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<command_item::AbstractCommandItem> Action_Timesync::getClone() const
{
    return std::make_shared<Action_Timesync>(*this);
}

void Action_Timesync::getClone(std::shared_ptr<command_item::AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_Timesync>(*this);
}

QJsonObject Action_Timesync::toJSON(const int &vehicleID, const std::string &dataType) const{
    UNUSED(dataType);
    QJsonObject json = toJSON_base(vehicleID, dataType);
    json["tc1"] = std::to_string(_tc1).c_str();
    json["ts1"] = std::to_string(_ts1).c_str();
    return json;
}

void Action_Timesync::fromJSON(const QJsonDocument &inputJSON){
    _tc1 = inputJSON.object().value("type").toInt();
    _ts1 = inputJSON.object().value("type").toInt();
}

std::string Action_Timesync::toCSV(const std::string &delimiter) const{
    std::string newline = std::to_string(_tc1) + delimiter + std::to_string(_ts1) + delimiter;
    return newline;
}

Action_Timesync::Action_Timesync()
{

}

Action_Timesync::Action_Timesync(const Action_Timesync &obj):
    command_item::AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

Action_Timesync::Action_Timesync(const unsigned int &systemOrigin, const unsigned int &systemTarget) :
    command_item::AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string Action_Timesync::printCommandInfo() const
{
    return "";
}

void Action_Timesync::populateMACECOMMS_Timesync(mavlink_timesync_t &obj) const
{
    obj.tc1 = _tc1;
    obj.ts1 = _ts1;
}

void Action_Timesync::fromMACECOMMS_Timesync(const mavlink_timesync_t &obj)
{
    _tc1 = obj.tc1;
    _ts1 = obj.ts1;
}

/** Interface imposed via AbstractCommandItem */

void Action_Timesync::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    UNUSED(cmd);
}

void Action_Timesync::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    UNUSED(cmd);
}

void Action_Timesync::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
}

void Action_Timesync::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item


