#include "action_motor_test.h"

namespace command_item {

COMMANDTYPE ActionMotorTest::getCommandType() const
{
    return COMMANDTYPE::CI_ACT_MOTORTEST;
}

std::string ActionMotorTest::getDescription() const
{
    return "This tests the operation of vehicle motors";
}

bool ActionMotorTest::hasSpatialInfluence() const
{
    return false;
}

std::shared_ptr<AbstractCommandItem> ActionMotorTest::getClone() const
{
    return std::make_shared<ActionMotorTest>(*this);
}

void ActionMotorTest::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<ActionMotorTest>(*this);
}

ActionMotorTest::ActionMotorTest()
{

}

ActionMotorTest::ActionMotorTest(const ActionMotorTest &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

ActionMotorTest::ActionMotorTest(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::string ActionMotorTest::printCommandInfo() const
{
    return "";
}

/** Interface imposed via Interface_CommandItem<mace_command_short_t> */
void ActionMotorTest::populateCommandItem(mace_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    //obj.param = this->getRequestArm();
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionMotorTest::fromCommandItem(const mace_command_long_t &obj)
{
    UNUSED(obj);
    //this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mace_command_short_t> */

/** Interface imposed via AbstractCommandItem */

void ActionMotorTest::populateMACECOMMS_MissionItem(mace_mission_item_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mace_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    Interface_CommandHelper<mace_command_long_t>::transferToMissionItem(longCommand, cmd);
}

void ActionMotorTest::fromMACECOMMS_MissionItem(const mace_mission_item_t &cmd)
{
    mace_command_long_t longCommand;
    Interface_CommandHelper<mace_command_long_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionMotorTest::generateMACEMSG_MissionItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_mission_item_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mace_msg_mission_item_encode_chan();
}

void ActionMotorTest::generateMACEMSG_CommandItem(mace_message_t &msg) const
{
    UNUSED(msg);
    mace_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mace_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
