#include "action_set_surface_deflection.h"

namespace command_item {

MAV_CMD Action_SetSurfaceDeflection::getCommandType() const
{
    return MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED;
}

std::string Action_SetSurfaceDeflection::getDescription() const
{
    return "This will command the deflection of the flight surfaces.";
}

bool Action_SetSurfaceDeflection::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> Action_SetSurfaceDeflection::getClone() const
{
    return std::make_shared<Action_SetSurfaceDeflection>(*this);
}

void Action_SetSurfaceDeflection::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_SetSurfaceDeflection>(*this);
}

Action_SetSurfaceDeflection::Action_SetSurfaceDeflection()
{
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, _time);
}

Action_SetSurfaceDeflection::Action_SetSurfaceDeflection(const Action_SetSurfaceDeflection &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

Action_SetSurfaceDeflection::Action_SetSurfaceDeflection(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string Action_SetSurfaceDeflection::printCommandInfo() const
{
    return "";
}

void  Action_SetSurfaceDeflection::setTime(){
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, _time);
}

QJsonObject Action_SetSurfaceDeflection::toJSON(const int &vehicleID, const std::string &dataType) const{
    QJsonObject json = toJSON_base(vehicleID,dataType);
    json["timestamp"] = _time.ToQTDateTime().toString(Qt::ISODateWithMs);
    json["aileron"] = this->_surfaceDeflection._aileron;
    json["elevator"] = this->_surfaceDeflection._elevator;
    json["rudder"] = this->_surfaceDeflection._rudder;
    json["throttle"] = this->_surfaceDeflection._throttle;
    return json;
}

void Action_SetSurfaceDeflection::fromJSON(const QJsonDocument &inputJSON){

    Deflection newDeflection(inputJSON.object().value("aileron").toDouble(),inputJSON.object().value("elevator").toDouble(),inputJSON.object().value("rudder").toDouble(),inputJSON.object().value("throttle").toDouble());
    this->_surfaceDeflection = newDeflection;
}

std::string Action_SetSurfaceDeflection::toCSV(const std::string &delimiter) const{
    std::string newline = std::to_string(this->_surfaceDeflection._aileron) + delimiter + std::to_string(this->_surfaceDeflection._elevator) + delimiter + std::to_string(this->_surfaceDeflection._rudder) + delimiter + std::to_string(this->_surfaceDeflection._throttle) + delimiter;
    return newline;
}

/** Interface imposed via AbstractCommandItem */

void Action_SetSurfaceDeflection::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    UNUSED(cmd);
}

void Action_SetSurfaceDeflection::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    UNUSED(cmd);
}

void Action_SetSurfaceDeflection::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
}

void Action_SetSurfaceDeflection::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item

