#include "state_flight_unknown.h"

namespace ardupilot {
namespace state{

AP_State_FlightUnknown::AP_State_FlightUnknown():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_UNKNOWN"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_UNKNOWN;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_UNKNOWN;
}

AbstractStateArdupilot* AP_State_FlightUnknown::getClone() const
{
    return (new AP_State_FlightUnknown(*this));
}

void AP_State_FlightUnknown::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightUnknown(*this);
}

hsm::Transition AP_State_FlightUnknown::GetTransition()
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

bool AP_State_FlightUnknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightUnknown::Update()
{

}

void AP_State_FlightUnknown::OnEnter()
{

}

void AP_State_FlightUnknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
