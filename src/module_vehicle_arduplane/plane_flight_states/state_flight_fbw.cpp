#include "state_flight_fbw.h"

namespace ardupilot {
namespace state{

AP_State_FlightFBW::AP_State_FlightFBW():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_FBW)
{

}

AbstractStateArdupilot* AP_State_FlightFBW::getClone() const
{
    return (new AP_State_FlightFBW(*this));
}

void AP_State_FlightFBW::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightFBW(*this);
}

hsm::Transition AP_State_FlightFBW::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_Flight_FBW."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightFBW::handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightFBW::Update()
{

}

void AP_State_FlightFBW::OnEnter()
{

}

void AP_State_FlightFBW::OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
