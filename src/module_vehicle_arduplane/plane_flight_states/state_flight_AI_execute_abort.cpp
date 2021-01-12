#include "state_flight_AI_execute_abort.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_ExecuteAbort::AP_State_FlightAI_ExecuteAbort():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_ABORT)
{

}

void AP_State_FlightAI_ExecuteAbort::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_ExecuteAbort::getClone() const
{
    return (new AP_State_FlightAI_ExecuteAbort(*this));
}

void AP_State_FlightAI_ExecuteAbort::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_ExecuteAbort(*this);
}

hsm::Transition AP_State_FlightAI_ExecuteAbort::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_ExecuteAbort::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_ExecuteAbort::Update()
{

}

void AP_State_FlightAI_ExecuteAbort::OnEnter()
{


}

void AP_State_FlightAI_ExecuteAbort::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{

}



} //end of namespace ardupilot
} //end of namespace state
