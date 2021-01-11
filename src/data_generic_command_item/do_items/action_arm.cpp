#include "action_arm.h"

namespace command_item {

MAV_CMD ActionArm::getCommandType() const
{
    return MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM;
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

ActionArm::ActionArm(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionArm::printCommandInfo() const
{
    return "";
}


/** Interface imposed via Interface_CommandItem<mavlink_command_long_t> */
void ActionArm::populateCommandItem(mavlink_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.param1 = this->getRequestArm();
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionArm::fromCommandItem(const mavlink_command_long_t &obj)
{
    this->setVehicleArm(static_cast<bool>(obj.param1));
}
/** End of interface imposed via Interface_CommandItem<mavlink_command_long_t> */

/** Interface imposed via AbstractCommandItem */

void ActionArm::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    Interface_CommandHelper<mavlink_command_long_t>::transferToMissionItem(longCommand, cmd);
}

void ActionArm::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    mavlink_command_long_t longCommand;
    Interface_CommandHelper<mavlink_command_long_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionArm::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_mace_mission_item_int_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mavlink_msg_mission_item_encode_chan();
}

void ActionArm::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mavlink_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
