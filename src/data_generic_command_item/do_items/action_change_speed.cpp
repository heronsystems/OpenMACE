#include "action_change_speed.h"

namespace command_item {

COMMANDTYPE ActionChangeSpeed::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_CHANGESPEED;
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

/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionChangeSpeed::populateCommandItem(mace_command_short_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.param = static_cast<float>(this->getDesiredSpeed());
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionChangeSpeed::fromCommandItem(const mace_command_short_t &obj)
{
    this->setDesiredSpeed(static_cast<double>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionChangeSpeed::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mace_command_short_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionChangeSpeed::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_short_t shortCommand;
    Interface_CommandHelper<mace_command_short_t>::transferFromMissionItem(cmd, shortCommand);
    fromCommandItem(shortCommand);
}

void ActionChangeSpeed::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_mission_item_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mace_msg_mission_item_encode_chan();
}

void ActionChangeSpeed::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */


} //end of namespace command_item
