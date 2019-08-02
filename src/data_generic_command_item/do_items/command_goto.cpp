#include "command_goto.h"

namespace command_item {

COMMANDTYPE CommandGoTo::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_GOTO;
}

std::string CommandGoTo::getDescription() const
{
    return "This command and its underlying contents tell the vehicle to perform an immediate spatial manuever.";
}

bool CommandGoTo::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> CommandGoTo::getClone() const
{
    return std::make_shared<CommandGoTo>(*this);
}

void CommandGoTo::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<CommandGoTo>(*this);
}

CommandGoTo::CommandGoTo():
    AbstractCommandItem(0,0), m_SpatialAction(nullptr)
{

}

CommandGoTo::CommandGoTo(const AbstractSpatialActionPtr cmd):
    AbstractCommandItem(0,0)
{
    this->setSpatialCommand(cmd);
}

CommandGoTo::CommandGoTo(const CommandGoTo &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

CommandGoTo::CommandGoTo(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), m_SpatialAction(nullptr)
{

}

void CommandGoTo::toMACEComms_CommandItem(mace_command_goto_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
}

std::string CommandGoTo::printCommandInfo() const
{

}


} //end of namespace command_item
