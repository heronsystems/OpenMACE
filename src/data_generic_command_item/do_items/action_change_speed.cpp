#include "action_change_speed.h"

namespace command_item {

MAV_CMD ActionChangeSpeed::getCommandType() const
{
    return MAV_CMD::MAV_CMD_DO_CHANGE_SPEED;
}

std::string ActionChangeSpeed::getDescription() const
{
    return "This changes the speed of the vehicle";
}

bool ActionChangeSpeed::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionChangeSpeed::getClone() const
{
    return std::make_shared<ActionChangeSpeed>(*this);
}

void ActionChangeSpeed::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionChangeSpeed>(*this);
}

ActionChangeSpeed::ActionChangeSpeed()
{

}

ActionChangeSpeed::ActionChangeSpeed(const ActionChangeSpeed &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionChangeSpeed::ActionChangeSpeed(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionChangeSpeed::printCommandInfo() const
{
    return "";
}

/** Interface imposed via Interface_CommandItem<mavlink_command_long_t> */
void ActionChangeSpeed::populateCommandItem(mavlink_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.param1 = static_cast<float>(this->getDesiredSpeed());
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionChangeSpeed::fromCommandItem(const mavlink_command_long_t &obj)
{
    this->setDesiredSpeed(static_cast<double>(obj.param1));
}
/** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */

/** Interface imposed via AbstractCommandItem */

void ActionChangeSpeed::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mavlink_command_long_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mavlink_command_long_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionChangeSpeed::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    mavlink_command_long_t longCommand;
    Interface_CommandHelper<mavlink_command_long_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionChangeSpeed::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_mace_mission_item_int_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mavlink_msg_mission_item_encode_chan();
}

void ActionChangeSpeed::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
}

/** End of interface imposed via AbstractCommandItem */


} //end of namespace command_item
