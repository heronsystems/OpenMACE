#include "state_flight_brake.h"

namespace ardupilot{
namespace state{

State_FlightBrake::State_FlightBrake():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_BRAKE"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_BRAKE;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_BRAKE;
}

AbstractStateArdupilot* State_FlightBrake::getClone() const
{
    return (new State_FlightBrake(*this));
}

void State_FlightBrake::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightBrake(*this);
}

hsm::Transition State_FlightBrake::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightBrake::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightBrake::Update()
{

}

void State_FlightBrake::OnEnter()
{

}

void State_FlightBrake::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
