#include "state_unknown.h"

namespace arducopter{
namespace state{

State_Unknown::State_Unknown():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_UNKNOWN"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_UNKNOWN;
    desiredStateEnum = ArducopterFlightState::STATE_UNKNOWN;
}

AbstractStateArducopter* State_Unknown::getClone() const
{
    return (new State_Unknown(*this));
}

void State_Unknown::getClone(AbstractStateArducopter** state) const
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
        case ArducopterFlightState::STATE_GROUNDED:
        {
            return hsm::SiblingTransition<State_Grounded>();
            break;
        }
        case ArducopterFlightState::STATE_TAKEOFF:
        {
            return hsm::SiblingTransition<State_Takeoff>();
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT:
        {
            return hsm::SiblingTransition<State_Flight>();
            break;
        }
        case ArducopterFlightState::STATE_LANDING:
        {
            return hsm::SiblingTransition<State_Landing>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_UNKNOWN."<<std::endl;
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
        desiredStateEnum = ArducopterFlightState::STATE_GROUNDED; //This is a definite case condition
    else
    {
        desiredStateEnum = ArducopterFlightState::STATE_FLIGHT;
    }
}

void State_Unknown::OnEnter()
{

}

void State_Unknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state

#include "flight_states/state_grounded.h"
#include "flight_states/state_takeoff.h"
#include "flight_states/state_flight.h"
#include "flight_states/state_landing.h"
