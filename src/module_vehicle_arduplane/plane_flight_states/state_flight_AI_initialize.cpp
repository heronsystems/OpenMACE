#include "state_flight_AI_initialize.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize::AP_State_FlightAI_Initialize():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE)
{

}

void AP_State_FlightAI_Initialize::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Initialize::getClone() const
{
    return (new AP_State_FlightAI_Initialize(*this));
}

void AP_State_FlightAI_Initialize::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize(*this);
}

hsm::Transition AP_State_FlightAI_Initialize::GetTransition()
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
        case Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE:
        {
            rtn = hsm::SiblingTransition<AP_State_FlightAI_Execute>(m_SurfaceDeflection);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE:
        {
            rtn = hsm::InnerEntryTransition<AP_State_FlightAI_Initialize_ROUTE>(m_InitializationConditions);
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Initialize::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    case MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED:
    {
        m_SurfaceDeflection = *command->as<command_item::Action_SetSurfaceDeflection>();
        setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE);
        break;
    }

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize::Update()
{
    if(this->IsInState<AP_State_FlightAI_Initialize_ABORT>())
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
    }
}

void AP_State_FlightAI_Initialize::OnEnter()
{
    //If we have no commands, this is an odd state to have been entered. We are therefore going to transition to the the abort condition.
    setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_ABORT);
}

void AP_State_FlightAI_Initialize::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //the command coming in should be something related to the initialization conditions
    if(command == nullptr)
    {
        //there is no command, we therefore should just abort this as something is wrong
        return;
    }
}

void AP_State_FlightAI_Initialize::OnEnter(const command_item::Action_InitializeTestSetup &initialization)
{
    m_InitializationConditions = initialization;

    setupGuidedMode();
}


void AP_State_FlightAI_Initialize::setupGuidedMode()
{
    _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;

    //This helps us based on the current conditions in the present moment
//    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();

//    if(currentModeString != "LOITER") {
//        MAVLINKUXVControllers::ControllerSystemMode* controllerSystemMode = AbstractStateArdupilot::prepareModeController();
//        controllerSystemMode->AddLambda_Finished(this, [this](const bool completed, const uint8_t finishCode){
//            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
//            else
//                _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
//        });
//        MavlinkEntityKey target = Owner().getMAVLINKID();
//        MavlinkEntityKey sender = 255;

//        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
//        commandMode.targetID = Owner().getMAVLINKID();
//        commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("LOITER");
//        controllerSystemMode->Send(commandMode,sender,target);

//    }
//    else {
//        _desiredState = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;
//    }
}


} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"
#include "plane_flight_states/state_flight_AI_execute.h"

#include "plane_flight_states/state_flight_AI_initialize_ABORT.h"
#include "plane_flight_states/state_flight_AI_initialize_ROUTE.h"

