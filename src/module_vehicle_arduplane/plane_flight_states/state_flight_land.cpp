#include "state_flight_land.h"

namespace ardupilot {
namespace state{

AP_State_FlightLand::AP_State_FlightLand():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_LAND)
{

}

AbstractStateArdupilot* AP_State_FlightLand::getClone() const
{
    return (new AP_State_FlightLand(*this));
}

void AP_State_FlightLand::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightLand(*this);
}

hsm::Transition AP_State_FlightLand::GetTransition()
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

bool AP_State_FlightLand::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightLand::Update()
{

}

void AP_State_FlightLand::OnEnter()
{

}

void AP_State_FlightLand::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
