#include "action_arm.h"

namespace command_item {

COMMANDTYPE ActionArm::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_ARM;
}

std::string ActionArm::getDescription() const
{
    return "This arms the vehicle";
}

bool ActionArm::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionArm::getClone() const
{
    return std::make_shared<ActionArm>(*this);
}

void ActionArm::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionArm>(*this);
}


ActionArm::ActionArm()
{

}

ActionArm::ActionArm(const ActionArm &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionArm::ActionArm(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionArm::printCommandInfo() const
{
    return "";
}

void ActionArm::toMACEComms_CommandItem(mace_command_short_t &obj) const
{
    Interface_CommandItem::initializeCommandItem(obj);
    obj.param = this->actionArm;
}

} //end of namespace command_item
