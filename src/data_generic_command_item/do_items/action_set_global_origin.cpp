#include "action_set_global_origin.h"

namespace command_item {

COMMANDTYPE Action_SetGlobalOrigin::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_SET_GLOBAL_ORIGIN;
}

std::string Action_SetGlobalOrigin::getDescription() const
{
    return "This changes the mode of the vehicle";
}

bool Action_SetGlobalOrigin::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> Action_SetGlobalOrigin::getClone() const
{
    return std::make_shared<Action_SetGlobalOrigin>(*this);
}

void Action_SetGlobalOrigin::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_SetGlobalOrigin>(*this);
}

Action_SetGlobalOrigin::Action_SetGlobalOrigin():
    AbstractCommandItem(0,0), m_Origin(nullptr)
{

}

Action_SetGlobalOrigin::Action_SetGlobalOrigin(const Action_SetGlobalOrigin &obj):
    AbstractCommandItem(0,0), m_Origin(nullptr)
{
    this->operator =(obj);
}

Action_SetGlobalOrigin::Action_SetGlobalOrigin(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), m_Origin(nullptr)
{

}

std::string Action_SetGlobalOrigin::printCommandInfo() const
{
    return "";
}

/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void Action_SetGlobalOrigin::populateCommandItem(mace_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void Action_SetGlobalOrigin::fromCommandItem(const mace_command_long_t &obj)
{
    UNUSED(obj);
    //this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void Action_SetGlobalOrigin::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    UNUSED(cmd);
}

void Action_SetGlobalOrigin::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    UNUSED(cmd);
}

void Action_SetGlobalOrigin::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
}

void Action_SetGlobalOrigin::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);

    mace_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
