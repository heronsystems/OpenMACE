#include "state_flight_unknown.h"

namespace ardupilot {
namespace state{

State_FlightUnknown::State_FlightUnknown():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_UNKNOWN)
{

}

AbstractStateArdupilot* State_FlightUnknown::getClone() const
{
    return (new State_FlightUnknown(*this));
}

void State_FlightUnknown::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightUnknown(*this);
}

hsm::Transition State_FlightUnknown::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightUnknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightUnknown::Update()
{

}

void State_FlightUnknown::OnEnter()
{

}

void State_FlightUnknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
