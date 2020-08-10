#include "state_flight_manual.h"

namespace arducopter{
namespace state{

State_FlightManual::State_FlightManual():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_MANUAL"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_MANUAL;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_MANUAL;
}

AbstractStateArducopter* State_FlightManual::getClone() const
{
    return (new State_FlightManual(*this));
}

void State_FlightManual::getClone(AbstractStateArducopter** state) const
{
    *state = new State_FlightManual(*this);
}

hsm::Transition State_FlightManual::GetTransition()
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

bool State_FlightManual::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightManual::Update()
{

}

void State_FlightManual::OnEnter()
{

}

void State_FlightManual::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state
