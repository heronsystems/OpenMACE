#include "state_flight_AI.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI::AP_State_FlightAI():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI)
{

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

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE:
        {
            rtn = hsm::InnerEntryTransition<AP_State_FlightAI_Initialize>(m_InitializationConditions);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE:
        {
            rtn = hsm::InnerEntryTransition<AP_State_FlightAI_Execute>(m_SurfaceDeflection);
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        success = currentInnerState->handleCommand(command);
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
    //This OnEnter state should never happen
    UNUSED(command);
    std::cout<<"We have entered the OnEnter method of AP_State_FlightAI with an AbstractCommand. This should not happen."<<std::endl;
    std::cout<<"The type of the command was: "<<std::endl;

}

void AP_State_FlightAI::OnEnter(const command_item::Action_SetSurfaceDeflection &command)
{
    //since we are going right into the surface deflection, this is clearly us trying to bypass
    m_SurfaceDeflection = command;
    setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE);
}

void AP_State_FlightAI::OnEnter(const command_item::Action_InitializeTestSetup &initialization)
{
    m_InitializationConditions = initialization;
    setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE);
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"
#include "plane_flight_states/state_flight_AI_execute.h"
#include "plane_flight_states/state_flight_AI_execute_end.h"
#include "plane_flight_states/state_flight_AI_initialize.h"
