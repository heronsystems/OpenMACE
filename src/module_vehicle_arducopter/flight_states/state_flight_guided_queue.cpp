#include "state_flight_guided_queue.h"

namespace ardupilot {
namespace state{

State_FlightGuided_Queue::State_FlightGuided_Queue():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_GUIDED_QUEUE)
{

}

void State_FlightGuided_Queue::OnExit()
{

}

AbstractStateArdupilot* State_FlightGuided_Queue::getClone() const
{
    return (new State_FlightGuided_Queue(*this));
}

void State_FlightGuided_Queue::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_Queue(*this);
}

hsm::Transition State_FlightGuided_Queue::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_FlightGuided_Queue."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Queue::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_FlightGuided_Queue::Update()
{

}

void State_FlightGuided_Queue::OnEnter()
{

}

void State_FlightGuided_Queue::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

