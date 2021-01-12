#include "action_execute_spatial_item.h"

namespace command_item {

MAV_CMD Action_ExecuteSpatialItem::getCommandType() const
{
    return MAV_CMD::MAV_CMD_USER_1;
}

std::string Action_ExecuteSpatialItem::getDescription() const
{
    return "This command and its underlying contents tell the vehicle to perform an immediate spatial manuever.";
}

bool Action_ExecuteSpatialItem::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> Action_ExecuteSpatialItem::getClone() const
{
    return std::make_shared<Action_ExecuteSpatialItem>(*this);
}

void Action_ExecuteSpatialItem::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_ExecuteSpatialItem>(*this);
}

Action_ExecuteSpatialItem::Action_ExecuteSpatialItem():
    AbstractCommandItem(0,0), m_SpatialAction(nullptr)
{

}

Action_ExecuteSpatialItem::Action_ExecuteSpatialItem(const AbstractSpatialActionPtr cmd):
    AbstractCommandItem(0,0)
{
    this->setSpatialAction(cmd);
}

Action_ExecuteSpatialItem::Action_ExecuteSpatialItem(const Action_ExecuteSpatialItem &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

Action_ExecuteSpatialItem::Action_ExecuteSpatialItem(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), m_SpatialAction(nullptr)
{

}

void Action_ExecuteSpatialItem::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    UNUSED(cmd);
    throw std::runtime_error("");
}

void Action_ExecuteSpatialItem::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    UNUSED(cmd);
    throw std::runtime_error("");
}

void Action_ExecuteSpatialItem::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    throw std::runtime_error("");
}

void Action_ExecuteSpatialItem::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    throw std::runtime_error("");
}

std::string Action_ExecuteSpatialItem::printCommandInfo() const
{
    return "TODO: IMPLEMENT printCommandInfo";
}


} //end of namespace command_item
