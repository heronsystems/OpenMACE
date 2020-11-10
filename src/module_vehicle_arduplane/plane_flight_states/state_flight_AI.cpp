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
        case Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Deflection>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_EVALEND:
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
    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
    if(currentModeString != "GUIDED") {
        //check that the vehicle is truely armed and switch us into the guided mode
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            controllerSystemMode->Shutdown();
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
            else
                desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED;
        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("AP_State_FlightGuided_modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = Owner().getMAVLINKID();
        commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("GUIDED");
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("AP_State_FlightGuided_modeController", controllerSystemMode);
    }
    else {
        //We have no command and therefore are just in the guided mode, we can tranisition to idle
        desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
    }
}

void AP_State_FlightAI::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI.h"
#include "plane_flight_states/state_flight_AI_deflection.h"
#include "plane_flight_states/state_flight_AI_evalend.h"
#include "plane_flight_states/state_flight_AI_initialize.h"

