#include "action_motor_test.h"

namespace command_item {

MAV_CMD ActionMotorTest::getCommandType() const
{
    return MAV_CMD::MAV_CMD_DO_MOTOR_TEST;
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

/** Interface imposed via Interface_CommandItem<mavlink_command_int_t> */
void ActionMotorTest::populateCommandItem(mavlink_command_long_t &obj) const
{
    obj.target_system = static_cast<uint8_t>(this->targetSystem);
    obj.target_component = static_cast<uint8_t>(this->targetComponent);
    //obj.param = this->getRequestArm();
    obj.command = static_cast<uint8_t>(this->getCommandType());
}

void ActionMotorTest::fromCommandItem(const mavlink_command_long_t &obj)
{
    UNUSED(obj);
    //this->setVehicleArm(static_cast<bool>(obj.param));
}
/** End of interface imposed via Interface_CommandItem<mavlink_command_int_t> */

/** Interface imposed via AbstractCommandItem */

void ActionMotorTest::populateMACECOMMS_MissionItem(mavlink_mace_mission_item_int_t &cmd) const
{
    AbstractCommandItem::populateMACECOMMS_MissionItem(cmd);
    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    Interface_CommandHelper<mavlink_command_long_t>::transferToMissionItem(longCommand, cmd);
}

void ActionMotorTest::fromMACECOMMS_MissionItem(const mavlink_mace_mission_item_int_t &cmd)
{
    mavlink_command_long_t longCommand;
    Interface_CommandHelper<mavlink_command_long_t>::transferFromMissionItem(cmd, longCommand);
    fromCommandItem(longCommand);
}

void ActionMotorTest::generateMACEMSG_MissionItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_mace_mission_item_int_t missionItem;
    AbstractCommandItem::populateMACECOMMS_MissionItem(missionItem);
    //mavlink_msg_mission_item_encode_chan();
}

void ActionMotorTest::generateMACEMSG_CommandItem(mavlink_message_t &msg) const
{
    UNUSED(msg);
    mavlink_command_long_t longCommand;
    this->populateCommandItem(longCommand);
    //mavlink_msg_command_short_encode_chan();
}
/** End of interface imposed via AbstractCommandItem */

} //end of namespace command_item
