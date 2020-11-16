#include "action_initialize_test_setup.h"

namespace command_item {

COMMANDTYPE Action_InitializeTestSetup::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_ARM;
}

std::string Action_InitializeTestSetup::getDescription() const
{
    return "This command will tell the vehicle to get ready for a test evaluation";
}

bool Action_InitializeTestSetup::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> Action_InitializeTestSetup::getClone() const
{
    return std::make_shared<Action_InitializeTestSetup>(*this);
}

void Action_InitializeTestSetup::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<Action_InitializeTestSetup>(*this);
}


Action_InitializeTestSetup::Action_InitializeTestSetup()
{

}

Action_InitializeTestSetup::Action_InitializeTestSetup(const Action_InitializeTestSetup &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

Action_InitializeTestSetup::Action_InitializeTestSetup(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string Action_InitializeTestSetup::printCommandInfo() const
{
    return "";
}


/** Interface imposed via AbstractCommandItem */

void Action_InitializeTestSetup::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{

}

void Action_InitializeTestSetup::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{

}

void Action_InitializeTestSetup::generateMACEMSG_MissionItem(mace_message_t &msg) const
{

}

void Action_InitializeTestSetup::generateMACEMSG_CommandItem(mace_message_t &msg) const
{

}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item

