#include "state_flight_loiter.h"

namespace ardupilot {
namespace state{

AP_State_FlightLoiter::AP_State_FlightLoiter():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_LOITER"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_LOITER;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_LOITER;
}

AbstractStateArdupilot* AP_State_FlightLoiter::getClone() const
{
    return (new AP_State_FlightLoiter(*this));
}

void AP_State_FlightLoiter::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightLoiter(*this);
}

hsm::Transition AP_State_FlightLoiter::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightLoiter::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightLoiter::Update()
{

}

void AP_State_FlightLoiter::OnEnter()
{

}

void AP_State_FlightLoiter::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
