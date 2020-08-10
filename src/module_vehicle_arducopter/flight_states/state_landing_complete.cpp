#include "state_landing_complete.h"

namespace arducopter{
namespace state{

State_LandingComplete::State_LandingComplete():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_LANDING_COMPLETE"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_LANDING_COMPLETE;
    desiredStateEnum = ArducopterFlightState::STATE_LANDING_COMPLETE;
}

AbstractStateArducopter* State_LandingComplete::getClone() const
{
    return (new State_LandingComplete(*this));
}

void State_LandingComplete::getClone(AbstractStateArducopter** state) const
{
    *state = new State_LandingComplete(*this);
}

hsm::Transition State_LandingComplete::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {

        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_TAKEOFF."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingComplete::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_LandingComplete::Update()
{

}

void State_LandingComplete::OnEnter()
{

}

void State_LandingComplete::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {

    }
}

} //end of namespace arducopter
} //end of namespace state
