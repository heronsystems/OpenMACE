#include "state_flight_AI_execute_deflection.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_ExecuteDeflection::AP_State_FlightAI_ExecuteDeflection():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_DEFLECTION), m_ControllerSurfaceDeflection(nullptr)
{
}

void AP_State_FlightAI_ExecuteDeflection::OnExit()
{
    AbstractStateArdupilot::OnExit();

    if(m_ControllerSurfaceDeflection){
        m_ControllerSurfaceDeflection->Shutdown();
    }
}

AbstractStateArdupilot* AP_State_FlightAI_ExecuteDeflection::getClone() const
{
    return (new AP_State_FlightAI_ExecuteDeflection(*this));
}

void AP_State_FlightAI_ExecuteDeflection::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_ExecuteDeflection(*this);
}

hsm::Transition AP_State_FlightAI_ExecuteDeflection::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_ABORT:
        {
            rtn = hsm::SiblingTransition<AP_State_FlightAI_ExecuteAbort>();
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_ExecuteDeflection::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case MAV_CMD::SET_SURFACE_DEFLECTION_NORMALIZED:
    {
        std::cout<<"Received a new deflection command!"<<std::endl;

        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        command_item::Action_SetSurfaceDeflection* cmd = currentCommand->as<command_item::Action_SetSurfaceDeflection>();
        constructAndSendTarget(*cmd);
        break;
    }
    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_ExecuteDeflection::Update()
{

}

void AP_State_FlightAI_ExecuteDeflection::OnEnter()
{
    //This should never happen, if so we are going to transition back to a known state
}

void AP_State_FlightAI_ExecuteDeflection::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
}

void AP_State_FlightAI_ExecuteDeflection::OnEnter(const command_item::Action_SetSurfaceDeflection &command)
{
    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    m_ControllerSurfaceDeflection = new MAVLINKUXVControllers::Controller_SetSurfaceDeflection(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    collection->Insert("AI_SetSurfaceDeflection",m_ControllerSurfaceDeflection);

    m_ControllerSurfaceDeflection->setLambda_Shutdown([this]()
    {
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto ptr = collection->Remove("AI_SetSurfaceDeflection");
        delete ptr; // the ptr here is the same as pointing to m_ControllerSurfaceDeflection
        m_ControllerSurfaceDeflection = nullptr;
    });

    handleCommand(std::make_shared< command_item::Action_SetSurfaceDeflection>(command));
}

void AP_State_FlightAI_ExecuteDeflection::handleTestProcedural(const command_item::Action_ProceduralCommand &command)
{
    std::cout<<"We have received a procedural command while in the execute deflection state!"<<std::endl;
    switch (command.whatIsTheProcedural()) {
    case AI_PROCEDURAL_COMMANDS::STOP:
    case AI_PROCEDURAL_COMMANDS::ABORT:
    {
        setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT_AI_EXECUTE_ABORT);
        break;
    }
    default:
    {
        break;
    }

    } //end of switch statement
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_execute_abort.h"
