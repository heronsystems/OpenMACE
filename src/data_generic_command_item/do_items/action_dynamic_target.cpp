#include "action_dynamic_target.h"

namespace command_item {

COMMANDTYPE Action_DynamicTarget::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_TARGET;
}

std::string Action_DynamicTarget::getDescription() const
{
    return "This command and its underlying contents tell the vehicle to perform an immediate spatial manuever. The explicit bitmask tells what "
           "dimension are appropriately valid.";
}

bool Action_DynamicTarget::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> Action_DynamicTarget::getClone() const
{
    return std::make_shared<Action_DynamicTarget>(*this);
}

void Action_DynamicTarget::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_DynamicTarget>(*this);
}

Action_DynamicTarget::Action_DynamicTarget():
    AbstractCommandItem(0,0), m_Target(nullptr)
{

}

Action_DynamicTarget::Action_DynamicTarget(const command_target::DynamicTarget* cmd):
    AbstractCommandItem(0,0)
{
    if(cmd != nullptr)
        m_Target = cmd->getDynamicTargetClone();
}

Action_DynamicTarget::Action_DynamicTarget(const Action_DynamicTarget &obj):
    AbstractCommandItem(obj)
{
    if(obj.getDynamicTarget() != nullptr)
        this->m_Target = obj.getDynamicTarget()->getDynamicTargetClone();
}

Action_DynamicTarget::Action_DynamicTarget(const unsigned int &systemOrigin, const unsigned int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

Action_DynamicTarget::~Action_DynamicTarget()
{
    if(m_Target != nullptr) {
        delete m_Target;
        m_Target = nullptr;
    }
}

void Action_DynamicTarget::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    UNUSED(cmd);
    throw std::runtime_error("");
}

void Action_DynamicTarget::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    UNUSED(cmd);
    throw std::runtime_error("");
}

void Action_DynamicTarget::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    throw std::runtime_error("");
}

void Action_DynamicTarget::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    throw std::runtime_error("");
}

std::string Action_DynamicTarget::printCommandInfo() const
{
    return "";
}


} //end of namespace command_item
