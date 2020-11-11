#include "state_flight_AI_transition.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Transition::AP_State_FlightAI_Transition():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_DEFLECTION"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;
}

void AP_State_FlightAI_Transition::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Transition::getClone() const
{
    return (new AP_State_FlightAI_Transition(*this));
}

void AP_State_FlightAI_Transition::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Transition(*this);
}

hsm::Transition AP_State_FlightAI_Transition::GetTransition()
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
            rtn = hsm::InnerTransition<AP_State_FlightAI_Abort>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Deflection>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_AI_TRANSITION. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Transition::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Transition::Update()
{

}

void AP_State_FlightAI_Transition::OnEnter()
{
    //This should never happen, if so we are going to transition back to a known state
}

void AP_State_FlightAI_Transition::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr || (command->getCommandType() != command_item::COMMANDTYPE::CI_ACT_SET_SURFACE_DEFLECTION))
    {
        std::cout<<"The wrong type of command caused this transition!!!"<<std::endl;
        desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_LOITER;
        return;
    }

    this->currentCommand = command->getClone();

    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
    if(currentModeString != "AI_DEFL") {
        //check that the vehicle is truely armed and switch us into the guided mode
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            controllerSystemMode->Shutdown();
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;
            else
                desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_LOITER;

        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("AP_State_AIDeflection_modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = Owner().getMAVLINKID();
        commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("AI_DEFL");
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("AP_State_AIDeflection_modeController", controllerSystemMode);
    }
    else {
        desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "state_flight_AI_abort.h"
#include "state_flight_guided_idle.h"

