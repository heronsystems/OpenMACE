#include "state_flight_rtl.h"

namespace ardupilot {
namespace state{

State_FlightRTL::State_FlightRTL():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_RTL)
{

}

AbstractStateArdupilot* State_FlightRTL::getClone() const
{
    return (new State_FlightRTL(*this));
}

void State_FlightRTL::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightRTL(*this);
}

hsm::Transition State_FlightRTL::GetTransition()
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

bool State_FlightRTL::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightRTL::Update()
{

}

void State_FlightRTL::OnEnter()
{

}

void State_FlightRTL::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

