#include "action_message_request.h"

namespace command_item {

COMMANDTYPE ActionMessageRequest::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_MSG_INTERVAL;
}

std::string ActionMessageRequest::getDescription() const
{
    return "This command requests the data of a specific message one time.";
}

bool ActionMessageRequest::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionMessageRequest::getClone() const
{
    return std::make_shared<ActionMessageRequest>(*this);
}

void ActionMessageRequest::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionMessageRequest>(*this);
}

ActionMessageRequest::ActionMessageRequest()
{

}

ActionMessageRequest::ActionMessageRequest(const ActionMessageRequest &copy):
    AbstractCommandItem(copy)
{
    this->message_id = copy.message_id;
}

ActionMessageRequest::ActionMessageRequest(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionMessageRequest::printCommandInfo() const
{
    return "";
}


/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionMessageRequest::populateCommandItem(mace_command_short_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionMessageRequest::fromCommandItem(const mace_command_short_t &obj)
{
    UNUSED(obj);
}

/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionMessageRequest::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    Interface_CommandHelper<mace_command_short_t>::transferToMissionItem(shortCommand, cmd);
}

void ActionMessageRequest::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_short_t longCommand;
    Interface_CommandHelper<mace_command_short_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionMessageRequest::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_mission_item_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mace_msg_mission_item_encode_chan();
}

void ActionMessageRequest::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_command_short_t shortCommand;
    this->populateCommandItem(shortCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item

