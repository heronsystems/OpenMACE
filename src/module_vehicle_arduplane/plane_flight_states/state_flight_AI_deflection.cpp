#include "state_flight_AI_deflection.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Deflection::AP_State_FlightAI_Deflection():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_DEFLECTION"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;

//    m_TimeoutController.connectTargetCallback(AP_State_FlightAI_Deflection::retransmitSurfaceDeflection, this);
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, previousTime);

}

void AP_State_FlightAI_Deflection::OnExit()
{
//    m_TimeoutController.stop();

    AbstractStateArdupilot::OnExit();

    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
    if(Owner().ControllersCollection()->Exist("AI_SetSurfaceDeflection")){
        MAVLINKUXVControllers::Controller_SetSurfaceDeflection* ptr = dynamic_cast<MAVLINKUXVControllers::Controller_SetSurfaceDeflection*>(Owner().ControllersCollection()->Remove("AI_SetSurfaceDeflection"));
        delete ptr;
    }
}

AbstractStateArdupilot* AP_State_FlightAI_Deflection::getClone() const
{
    return (new AP_State_FlightAI_Deflection(*this));
}

void AP_State_FlightAI_Deflection::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Deflection(*this);
}

hsm::Transition AP_State_FlightAI_Deflection::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_ABORT:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Deflection_Idle>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Deflection_Initialize>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Deflection::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_SET_SURFACE_DEFLECTION:
    {
        //stop the current controllers target transmission if it is running
//        m_TimeoutController.clearTarget();

        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        //The command is a target, we therefore have to figure out what type of target it is
        command_item::Action_SetSurfaceDeflection* cmd = currentCommand->as<command_item::Action_SetSurfaceDeflection>();
        constructAndSendTarget(*cmd);

        break;
    }
    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Deflection::Update()
{

}

void AP_State_FlightAI_Deflection::OnEnter()
{
    //This should never happen, if so we are going to transition back to a known state
}

void AP_State_FlightAI_Deflection::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto surfaceDeflectionController = new MAVLINKUXVControllers::Controller_SetSurfaceDeflection(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    collection->Insert("AI_SetSurfaceDeflection",surfaceDeflectionController);
}

} //end of namespace ardupilot
} //end of namespace state


