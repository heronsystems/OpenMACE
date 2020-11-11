#include "state_flight_AI_abort.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Abort::AP_State_FlightAI_Abort():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_ABORT"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
}

void AP_State_FlightAI_Abort::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Abort::getClone() const
{
    return (new AP_State_FlightAI_Abort(*this));
}

void AP_State_FlightAI_Abort::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Abort(*this);
}

hsm::Transition AP_State_FlightAI_Abort::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Abort::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Abort::Update()
{

}

void AP_State_FlightAI_Abort::OnEnter()
{
    //This should never happen, if so we are going to transition back to a known state
}

void AP_State_FlightAI_Abort::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
    if(currentModeString != "AI_DEFL") {
        //check that the vehicle is truely armed and switch us into the guided mode
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            controllerSystemMode->Shutdown();
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
            else

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
    else { //we are already in the current mode, therefore just pass on the command
        //We have no command and therefore are just in the guided mode, we can tranisition to idle
        desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
    }
}

} //end of namespace ardupilot
} //end of namespace state


