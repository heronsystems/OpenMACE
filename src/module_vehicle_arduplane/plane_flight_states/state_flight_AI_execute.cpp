#include "state_flight_AI_execute.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Execute::AP_State_FlightAI_Execute():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE)
{

}

void AP_State_FlightAI_Execute::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Execute::getClone() const
{
    return (new AP_State_FlightAI_Execute(*this));
}

void AP_State_FlightAI_Execute::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Execute(*this);
}

hsm::Transition AP_State_FlightAI_Execute::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_ABORT:
        {
            rtn = hsm::SiblingTransition<AP_State_FlightAI_Abort>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION:
        {
            rtn = hsm::InnerEntryTransition<AP_State_FlightAI_ExecuteDeflection>(m_SurfaceDeflection);
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Execute::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED:
    {
        std::cout<<"We are seeing more commands!"<<std::endl;
        if(this->IsInState<AP_State_FlightAI_ExecuteDeflection>())
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }
        else
        {
            std::cout<<"We have gotten a command but are not yet in the ai execute deflection mode"<<std::endl;
            success = true;
        }
        break;
    }
    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Execute::Update()
{
    if(this->IsInState<AP_State_FlightAI_ExecuteAbort>())
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
    }
}

void AP_State_FlightAI_Execute::OnEnter()
{


}

void AP_State_FlightAI_Execute::OnEnter(const command_item::Action_SetSurfaceDeflection &command)
{
    std::cout<<"State flight AI Execute has entered with a new command!"<<std::endl;
    m_SurfaceDeflection = command;
    setupAIMode();
}

void AP_State_FlightAI_Execute::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //the command coming in should be something related to the initialization conditions
    if(command == nullptr)
    {
        //there is no command, we therefore should just abort this as something is wrong
        return;
    }
}

void AP_State_FlightAI_Execute::setupAIMode()
{

//    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();

    if(currentModeString != "AI_DEFL") {

        MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();
        modeController->AddLambda_Finished(this, [this, modeController](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {
                _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION;
            }
            else
            {
                _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
            }
            modeController->Shutdown();
        });

        MavlinkEntityKey sender = 255;
        MavlinkEntityKey target = Owner().getMAVLINKID();
        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
        commandMode.vehicleMode = static_cast<uint8_t>(PLANE_MODE::PLANE_MODE_AI_DEFLECTION);
        modeController->Send(commandMode,sender,target);
    }
    else
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"

#include "plane_flight_states/state_flight_AI_execute_abort.h"
#include "plane_flight_states/state_flight_AI_execute_deflection.h"
#include "plane_flight_states/state_flight_AI_execute_end.h"


