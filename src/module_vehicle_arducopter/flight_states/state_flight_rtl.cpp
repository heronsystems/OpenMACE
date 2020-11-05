#include "state_flight_rtl.h"

namespace ardupilot {
namespace state{

State_FlightRTL::State_FlightRTL():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_RTL"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_RTL;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_RTL;
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

