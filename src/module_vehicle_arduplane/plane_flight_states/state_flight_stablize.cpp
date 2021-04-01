#include "state_flight_stabilize.h"

namespace ardupilot {
namespace state{

AP_State_FlightStabilize::AP_State_FlightStabilize():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_STABILIZE)
{

}

AbstractStateArdupilot* AP_State_FlightStabilize::getClone() const
{
    return (new AP_State_FlightStabilize(*this));
}

void AP_State_FlightStabilize::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightStabilize(*this);
}

hsm::Transition AP_State_FlightStabilize::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_Flight_Stabilize."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightStabilize::handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightStabilize::Update()
{

}

void AP_State_FlightStabilize::OnEnter()
{

}

void AP_State_FlightStabilize::OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
