#include "state_flight_guided_queue.h"

namespace arducopter{
namespace state{

State_FlightGuided_Queue::State_FlightGuided_Queue():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_QUEUE"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_QUEUE;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_QUEUE;
}

void State_FlightGuided_Queue::OnExit()
{

}

AbstractStateArducopter* State_FlightGuided_Queue::getClone() const
{
    return (new State_FlightGuided_Queue(*this));
}

void State_FlightGuided_Queue::getClone(AbstractStateArducopter** state) const
{
    *state = new State_FlightGuided_Queue(*this);
}

hsm::Transition State_FlightGuided_Queue::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_FlightGuided_Queue."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Queue::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{

}

void State_FlightGuided_Queue::Update()
{

}

void State_FlightGuided_Queue::OnEnter()
{

}

void State_FlightGuided_Queue::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state

