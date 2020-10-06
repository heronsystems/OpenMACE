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


/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionArm::populateCommandItem(mace_command_short_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.param = this->getRequestArm();
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionArm::fromCommandItem(const mace_command_short_t &obj)
{
    this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionArm::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mace_command_short_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionArm::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_short_t shortCommand;
    Interface_CommandHelper<mace_command_short_t>::transferFromMissionItem(cmd, shortCommand);
    fromCommandItem(shortCommand);
}

void ActionArm::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_mission_item_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mace_msg_mission_item_encode_chan();
}

void ActionArm::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
