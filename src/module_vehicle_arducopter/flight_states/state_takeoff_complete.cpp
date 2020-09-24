#include "state_takeoff_complete.h"

namespace ardupilot {
namespace state{

State_TakeoffComplete::State_TakeoffComplete():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_COMPLETE"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
    desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
}

AbstractStateArdupilot* State_TakeoffComplete::getClone() const
{
    return (new State_TakeoffComplete(*this));
}

void State_TakeoffComplete::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffComplete(*this);
}

hsm::Transition State_TakeoffComplete::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    /*
     * We do not want to cause an immediate sibling transition from within an
     * inner state. Therefore, this transition is completed by the parent. There
     * is nothing left for us to do once within this state and should never
     * transition this far.
     */

    return rtn;
}

bool State_TakeoffComplete::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return true;
}

void State_TakeoffComplete::Update()
{

}

void State_TakeoffComplete::OnEnter()
{

}

void State_TakeoffComplete::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
}

} //end of namespace ardupilot
} //end of namespace state

