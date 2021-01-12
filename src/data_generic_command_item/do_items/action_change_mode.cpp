#include "action_change_mode.h"

namespace command_item {

MAV_CMD ActionChangeMode::getCommandType() const
{
    return MAV_CMD::MAV_CMD_DO_SET_MODE;
}

std::string ActionChangeMode::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool ActionChangeMode::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionChangeMode::getClone() const
{
    return std::make_shared<ActionChangeMode>(*this);
}

void ActionChangeMode::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionChangeMode>(*this);
}

ActionChangeMode::ActionChangeMode()
{

}

ActionChangeMode::ActionChangeMode(const ActionChangeMode &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionChangeMode::ActionChangeMode(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionChangeMode::printCommandInfo() const
{
    return "";
}

/** Interface imposed via Interface_CommandItem<mavlink_command_int_t> */
void ActionChangeMode::populateCommandItem(mavlink_command_int_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionChangeMode::fromCommandItem(const mavlink_command_int_t &obj)
{
    UNUSED(obj);
    //this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */

/** Interface imposed via AbstractCommandItem */

void ActionChangeMode::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mavlink_command_int_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mavlink_command_int_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionChangeMode::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    mavlink_command_int_t shortCommand;
    Interface_CommandHelper<mavlink_command_int_t>::transferFromMissionItem(cmd, shortCommand);
    fromCommandItem(shortCommand);
}

void ActionChangeMode::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_mace_mission_item_int_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mavlink_msg_mission_item_encode_chan();
}

void ActionChangeMode::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_command_int_t shortCommand;
    this->populateCommandItem(shortCommand);
    //mavlink_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item

