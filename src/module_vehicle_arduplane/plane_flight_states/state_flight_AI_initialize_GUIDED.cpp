#include "state_flight_AI_initialize.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize_GUIDED::AP_State_FlightAI_Initialize_GUIDED():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_INITIALIZE_GUIDED"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_GUIDED;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_GUIDED;
}

void AP_State_FlightAI_Initialize_GUIDED::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Initialize_GUIDED::getClone() const
{
    return (new AP_State_FlightAI_Initialize_GUIDED(*this));
}

void AP_State_FlightAI_Initialize_GUIDED::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize_GUIDED(*this);
}

hsm::Transition AP_State_FlightAI_Initialize_GUIDED::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_FLIGHT_AI_IDLE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_GUIDED_Idle>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_GUIDED_Initialize>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_DEFLECTION:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_GUIDED_Deflection>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_AI_END:
        {
            rtn = hsm::InnerTransition<AP_State_FlightAI_Initialize_GUIDED_EvalEnd>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Initialize_GUIDED::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize_GUIDED::Update()
{

}

void AP_State_FlightAI_Initialize_GUIDED::OnEnter()
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

void AP_State_FlightAI_Initialize_GUIDED::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_guided_idle.h"
#include "plane_flight_states/state_flight_guided_spatial_item.h"
#include "plane_flight_states/state_flight_guided_queue.h"
#include "plane_flight_states/state_flight_guided_target_att.h"
#include "plane_flight_states/state_flight_guided_target_car.h"
#include "plane_flight_states/state_flight_guided_target_geo.h"

