#include "state_flight_auto.h"

namespace ardupilot {
namespace state{

State_FlightAuto::State_FlightAuto():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AUTO"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AUTO;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AUTO;
}

AbstractStateArdupilot* State_FlightAuto::getClone() const
{
    return (new State_FlightAuto(*this));
}

void State_FlightAuto::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightAuto(*this);
}

hsm::Transition State_FlightAuto::GetTransition()
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

bool State_FlightAuto::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightAuto::Update()
{

}

void State_FlightAuto::OnEnter()
{

}

void State_FlightAuto::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
