#include "state_flight_brake.h"

namespace ardupilot {
namespace state{

State_FlightBrake::State_FlightBrake():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_BRAKE)
{

}

AbstractStateArdupilot* State_FlightBrake::getClone() const
{
    return (new State_FlightBrake(*this));
}

void State_FlightBrake::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightBrake(*this);
}

hsm::Transition State_FlightBrake::GetTransition()
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

bool State_FlightBrake::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightBrake::Update()
{

}

void State_FlightBrake::OnEnter()
{

}

void State_FlightBrake::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
