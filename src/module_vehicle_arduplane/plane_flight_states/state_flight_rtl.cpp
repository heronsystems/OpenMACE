#include "state_flight_rtl.h"

namespace ardupilot {
namespace state{

AP_State_FlightRTL::AP_State_FlightRTL():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_RTL)
{

}

AbstractStateArdupilot* AP_State_FlightRTL::getClone() const
{
    return (new AP_State_FlightRTL(*this));
}

void AP_State_FlightRTL::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightRTL(*this);
}

hsm::Transition AP_State_FlightRTL::GetTransition()
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

bool AP_State_FlightRTL::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return true;
}

void AP_State_FlightRTL::Update()
{

}

void AP_State_FlightRTL::OnEnter()
{

}

void AP_State_FlightRTL::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

