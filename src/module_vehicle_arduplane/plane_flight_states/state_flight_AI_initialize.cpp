#include "state_flight_AI_initialize.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize::AP_State_FlightAI_Initialize():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_INITIALIZE"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE;
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

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_ABORT:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_ABORT>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Deflection>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_EVAL_END:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_EvalEnd>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_ROUTE>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Initialize::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize::Update()
{
    if(this->IsInState<AP_State_FlightAI_Initialize_ABORT>())
    {
        desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
    }
}

void AP_State_FlightAI_Initialize::OnEnter()
{


}

void AP_State_FlightAI_Initialize::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //the command coming in should be something related to the initialization conditions
    if(command == nullptr)
    {
        //there is no command, we therefore should just abort this as something is wrong
        return;
    }

    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();

    if(currentModeString != "GUIDED") {
        if(Owner().ControllersCollection()->Exist("modeController")){
            MAVLINKUXVControllers::ControllerSystemMode* ptr = dynamic_cast<MAVLINKUXVControllers::ControllerSystemMode*>(Owner().ControllersCollection()->At("modeController"));
            ptr->AddLambda_Finished(this, [this](const bool completed, const uint8_t finishCode){
                if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;
                else
                    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_ABORT;
            });

            MavlinkEntityKey sender = 255;
            MavlinkEntityKey target = Owner().getMAVLINKID();

            MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
            commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
            commandMode.vehicleMode = static_cast<uint8_t>(Owner().m_ArdupilotMode->getFlightModeFromString("GUIDED"));
            ptr->Send(commandMode,sender,target);
        }
        else
        {
            std::cout<<"For some reason the mode controller doesn't exist."<<std::endl;
        }
    }
}



} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"
#include "plane_flight_states/state_flight_AI_deflection.h"
#include "plane_flight_states/state_flight_AI_evalend.h"

#include "plane_flight_states/state_flight_AI_initialize_ABORT.h"
#include "plane_flight_states/state_flight_AI_initialize_ROUTE.h"

