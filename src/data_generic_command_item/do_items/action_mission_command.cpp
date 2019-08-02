#include "action_mission_command.h"
namespace command_item {

COMMANDTYPE ActionMissionCommand::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_MISSIONCOMMAND;
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

ActionMissionCommand::ActionMissionCommand(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionMissionCommand::printCommandInfo() const
{

}

void ActionMissionCommand::toMACEComms_CommandItem(mace_command_short_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    obj.param = static_cast<float>(this->missionCommand);
}

} //end of namespace command_item
