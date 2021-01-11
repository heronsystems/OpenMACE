#include "state_flight_manual.h"

namespace ardupilot {
namespace state{

State_FlightManual::State_FlightManual():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_MANUAL)
{

}

AbstractStateArdupilot* State_FlightManual::getClone() const
{
    return (new State_FlightManual(*this));
}

void State_FlightManual::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightManual(*this);
}

hsm::Transition State_FlightManual::GetTransition()
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

bool State_FlightManual::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightManual::Update()
{

}

void State_FlightManual::OnEnter()
{

}

void State_FlightManual::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
