#include "state_flight_rtl.h"

namespace arducopter{
namespace state{

State_FlightRTL::State_FlightRTL():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_RTL"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_RTL;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_RTL;
}

AbstractStateArducopter* State_FlightRTL::getClone() const
{
    return (new State_FlightRTL(*this));
}

void State_FlightRTL::getClone(AbstractStateArducopter** state) const
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

}

void State_FlightRTL::Update()
{

}

void State_FlightRTL::OnEnter()
{

}

void State_FlightRTL::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state

