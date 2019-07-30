#include "action_change_mode.h"

namespace CommandItem {

COMMANDITEM ActionChangeMode::getCommandType() const
{
    return COMMANDITEM::CI_ACT_CHANGEMODE;
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

ActionChangeMode::ActionChangeMode(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionChangeMode::printCommandInfo() const
{

}

}

