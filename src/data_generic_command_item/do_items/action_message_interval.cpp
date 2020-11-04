#include "action_message_interval.h"

namespace command_item {

COMMANDTYPE ActionMessageInterval::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_MSG_INTERVAL;
}

std::string ActionMessageInterval::getDescription() const
{
    return "This command modifies the rate that a message is received.";
}

bool ActionMessageInterval::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionMessageInterval::getClone() const
{
    return std::make_shared<ActionMessageInterval>(*this);
}

void ActionMessageInterval::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionMessageInterval>(*this);
}

ActionMessageInterval::ActionMessageInterval()
{

}

ActionMessageInterval::ActionMessageInterval(const ActionMessageInterval &copy):
    AbstractCommandItem(copy)
{
    this->message_id = copy.message_id;
    this->interval_us = copy.interval_us;
}

ActionMessageInterval::ActionMessageInterval(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionMessageInterval::printCommandInfo() const
{
    return "";
}


/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionMessageInterval::populateCommandItem(mace_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionMessageInterval::fromCommandItem(const mace_command_long_t &obj)
{
    UNUSED(obj);
}

/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionMessageInterval::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    Interface_CommandHelper<mace_command_long_t>::transferToMissionItem(longCommand, cmd);
}

void ActionMessageInterval::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_long_t longCommand;
    Interface_CommandHelper<mace_command_long_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionMessageInterval::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_mission_item_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mace_msg_mission_item_encode_chan();
}

void ActionMessageInterval::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
