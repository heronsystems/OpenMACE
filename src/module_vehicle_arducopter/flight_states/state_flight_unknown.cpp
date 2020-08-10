#include "state_flight_unknown.h"

namespace arducopter{
namespace state{

State_FlightUnknown::State_FlightUnknown():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_UNKNOWN"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_UNKNOWN;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_UNKNOWN;
}

AbstractStateArducopter* State_FlightUnknown::getClone() const
{
    return (new State_FlightUnknown(*this));
}

void State_FlightUnknown::getClone(AbstractStateArducopter** state) const
{
    *state = new State_FlightUnknown(*this);
}

hsm::Transition State_FlightUnknown::GetTransition()
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

bool State_FlightUnknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightUnknown::Update()
{

}

void State_FlightUnknown::OnEnter()
{

}

void State_FlightUnknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state
