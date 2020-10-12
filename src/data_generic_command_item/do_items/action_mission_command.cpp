#include "action_mission_command.h"
#include "interface_command_helper.cpp"

namespace command_item {

COMMANDTYPE ActionMissionCommand::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_MISSIONCMD;
}

std::string ActionMissionCommand::getDescription() const
{
    return "This influences whether or not the vehicle should start or pause the current mission state.";
}

bool ActionMissionCommand::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionMissionCommand::getClone() const
{
    return std::make_shared<ActionMissionCommand>(*this);
}

void ActionMissionCommand::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionMissionCommand>(*this);
}

ActionMissionCommand::ActionMissionCommand()
{

}

ActionMissionCommand::ActionMissionCommand(const ActionMissionCommand &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionMissionCommand::ActionMissionCommand(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionMissionCommand::printCommandInfo() const
{
    return "TODO: IMPLEMENT printCommandInfo";
}

/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionMissionCommand::populateCommandItem(mace_command_short_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.param = static_cast<float>(this->getMissionCommandAction());
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionMissionCommand::fromCommandItem(const mace_command_short_t &obj)
{
    this->setMissionCommandType(static_cast<Data::MissionCommandAction>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionMissionCommand::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mace_command_short_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionMissionCommand::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_short_t shortCommand;
    Interface_CommandHelper<mace_command_short_t>::transferFromMissionItem(cmd, shortCommand);
    fromCommandItem(shortCommand);
}

void ActionMissionCommand::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    throw std::logic_error("Requested a mace message based on a mission item " + CommandItemToString(this->getCommandType()) +" which should logically never occur.");
}

void ActionMissionCommand::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);

    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
