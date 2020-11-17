#include "state_flight_AI.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI::AP_State_FlightAI():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI;
}

void AP_State_FlightAI::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI::getClone() const
{
    return (new AP_State_FlightAI(*this));
}

void AP_State_FlightAI::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI(*this);
}

hsm::Transition AP_State_FlightAI::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_EVAL_END:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_EvalEnd>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_TEST_INITIALIZATION:
    {
        if(this->IsInState<AP_State_FlightAI_Initialize>())
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }
        else
        {
            currentCommand = command->getClone();
            desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE;
            success = true;
        }
        break;
    }
    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI::Update()
{

}

void AP_State_FlightAI::OnEnter()
{

}

void AP_State_FlightAI::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr)
    {
        //we need to move to a different state to abort

    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"
#include "plane_flight_states/state_flight_AI_evalend.h"
#include "plane_flight_states/state_flight_AI_initialize.h"
