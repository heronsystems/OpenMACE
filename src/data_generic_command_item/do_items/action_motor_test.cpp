#include "action_motor_test.h"

namespace CommandItem {

COMMANDITEM ActionMotorTest::getCommandType() const
{
    return COMMANDITEM::CI_ACT_MOTORTEST;
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

}
