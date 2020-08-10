#include "state_flight_loiter.h"

namespace arducopter{
namespace state{

State_FlightLoiter::State_FlightLoiter():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_LOITER"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_LOITER;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_LOITER;
}

AbstractStateArducopter* State_FlightLoiter::getClone() const
{
    return (new State_FlightLoiter(*this));
}

void State_FlightLoiter::getClone(AbstractStateArducopter** state) const
{
    *state = new State_FlightLoiter(*this);
}

hsm::Transition State_FlightLoiter::GetTransition()
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

bool State_FlightLoiter::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightLoiter::Update()
{

}

void State_FlightLoiter::OnEnter()
{

}

void State_FlightLoiter::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state
