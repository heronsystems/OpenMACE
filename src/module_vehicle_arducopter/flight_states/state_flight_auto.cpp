#include "state_flight_auto.h"

namespace arducopter{
namespace state{

State_FlightAuto::State_FlightAuto():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AUTO"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_AUTO;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_AUTO;
}

AbstractStateArducopter* State_FlightAuto::getClone() const
{
    return (new State_FlightAuto(*this));
}

void State_FlightAuto::getClone(AbstractStateArducopter** state) const
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

}

void State_FlightAuto::Update()
{

}

void State_FlightAuto::OnEnter()
{

}

void State_FlightAuto::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state
