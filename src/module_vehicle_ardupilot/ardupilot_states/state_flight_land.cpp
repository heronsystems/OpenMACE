#include "state_flight_land.h"

namespace ardupilot{
namespace state{

State_FlightLand::State_FlightLand():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_LAND"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_LAND;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_LAND;
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

bool State_FlightLand::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightLand::Update()
{

}

void State_FlightLand::OnEnter()
{

}

void State_FlightLand::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state
