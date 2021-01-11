#include "action_set_global_origin.h"

namespace command_item {

MAV_CMD Action_SetGlobalOrigin::getCommandType() const
{
    return MAV_CMD::MAV_CMD_USER_5;
}

std::string Action_SetGlobalOrigin::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool Action_SetGlobalOrigin::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> Action_SetGlobalOrigin::getClone() const
{
    return std::make_shared<Action_SetGlobalOrigin>(*this);
}

void Action_SetGlobalOrigin::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_SetGlobalOrigin>(*this);
}

Action_SetGlobalOrigin::Action_SetGlobalOrigin():
    AbstractCommandItem(0,0), m_Origin(nullptr)
{

}

Action_SetGlobalOrigin::Action_SetGlobalOrigin(const Action_SetGlobalOrigin &obj):
    AbstractCommandItem(0,0), m_Origin(nullptr)
{
    this->operator =(obj);
}

Action_SetGlobalOrigin::Action_SetGlobalOrigin(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), m_Origin(nullptr)
{

}

std::string Action_SetGlobalOrigin::printCommandInfo() const
{
    return "";
}

/** Interface imposed via Interface_CommandItem<mavlink_command_int_t> */
void Action_SetGlobalOrigin::populateCommandItem(mavlink_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void Action_SetGlobalOrigin::fromCommandItem(const mavlink_command_long_t &obj)
{
    UNUSED(obj);
    //this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */

/** Interface imposed via AbstractCommandItem */

void Action_SetGlobalOrigin::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    UNUSED(cmd);
}

void Action_SetGlobalOrigin::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    UNUSED(cmd);
}

void Action_SetGlobalOrigin::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
}

void Action_SetGlobalOrigin::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);

    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mavlink_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
