#include "state_flight_guided_queue.h"

namespace ardupilot {
namespace state{

AP_State_FlightGuided_Queue::AP_State_FlightGuided_Queue():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_GUIDED_QUEUE)
{

}

void AP_State_FlightGuided_Queue::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightGuided_Queue::getClone() const
{
    return (new AP_State_FlightGuided_Queue(*this));
}

void AP_State_FlightGuided_Queue::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightGuided_Queue(*this);
}

hsm::Transition AP_State_FlightGuided_Queue::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from AP_State_FlightGuided_Queue."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightGuided_Queue::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_FlightGuided_Queue::Update()
{

}

void AP_State_FlightGuided_Queue::OnEnter()
{

}

void AP_State_FlightGuided_Queue::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

