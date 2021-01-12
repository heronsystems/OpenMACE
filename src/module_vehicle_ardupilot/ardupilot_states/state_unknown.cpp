#include "state_unknown.h"

namespace ardupilot{
namespace state{

State_Unknown::State_Unknown():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_UNKNOWN"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_UNKNOWN;
    desiredStateEnum = ArdupilotFlightState::STATE_UNKNOWN;
}

AbstractStateArdupilot* State_Unknown::getClone() const
{
    return (new State_Unknown(*this));
}

void State_Unknown::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Unknown(*this);
}

hsm::Transition State_Unknown::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_GROUNDED:
        {
            return hsm::SiblingTransition<State_Grounded>();
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF:
        {
            return hsm::SiblingTransition<State_Takeoff>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT:
        {
            return hsm::SiblingTransition<State_Flight>();
            break;
        }
        case ArdupilotFlightState::STATE_LANDING:
        {
            return hsm::SiblingTransition<State_Landing>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_UNKNOWN."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_Unknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_Unknown::Update()
{
    if(!Owner().state->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED; //This is a definite case condition
    else
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
    }
}

void State_Unknown::OnEnter()
{

}

void State_Unknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded.h"
#include "ardupilot_states/state_takeoff.h"
#include "ardupilot_states/state_flight.h"
#include "ardupilot_states/state_landing.h"
