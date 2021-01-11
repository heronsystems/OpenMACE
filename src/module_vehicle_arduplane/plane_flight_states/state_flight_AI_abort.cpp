#include "state_flight_AI_abort.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Abort::AP_State_FlightAI_Abort():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_ABORT)
{

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

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Abort::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    //Within the abort state we do not want to handle any further commands related to the system under test
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
    //This should never happen, but if it does, we will handle it with an empty descriptor
    MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();

    modeController->AddLambda_Finished(this, [this, modeController](const bool completed, const uint8_t finishCode){
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
        {
            std::cout<<"The abort is tranisioning to RTL!"<<std::endl;
        }
        modeController->Shutdown();
    });

    MavlinkEntityKey sender = 255;
    MavlinkEntityKey target = Owner().getMAVLINKID();

    MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
    commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
    commandMode.vehicleMode = static_cast<uint8_t>(PLANE_MODE::PLANE_MODE_RTL);
    modeController->Send(commandMode,sender,target);
}

void AP_State_FlightAI_Abort::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);

}



} //end of namespace ardupilot
} //end of namespace state


