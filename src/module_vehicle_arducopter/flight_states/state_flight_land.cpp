#include "state_flight_land.h"

namespace ardupilot {
namespace state{

State_FlightLand::State_FlightLand():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_LAND)
{

}

AbstractStateArdupilot* State_FlightLand::getClone() const
{
    return (new State_FlightLand(*this));
}

void State_FlightLand::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightLand(*this);
}

hsm::Transition State_FlightLand::GetTransition()
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

bool State_FlightLand::handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightLand::Update()
{

}

void State_FlightLand::OnEnter()
{

}

void State_FlightLand::OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
