#include "state_flight_AI_test_end.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_TestEnd::AP_State_FlightAI_TestEnd():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_TESTEND)
{

}

void AP_State_FlightAI_TestEnd::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_TestEnd::getClone() const
{
    return (new AP_State_FlightAI_TestEnd(*this));
}

void AP_State_FlightAI_TestEnd::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_TestEnd(*this);
}

hsm::Transition AP_State_FlightAI_TestEnd::GetTransition()
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

bool AP_State_FlightAI_TestEnd::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    //Within the abort state we do not want to handle any further commands related to the system under test
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_TestEnd::Update()
{

}

void AP_State_FlightAI_TestEnd::OnEnter()
{
    //This should never happen, but if it does, we will handle it with an empty descriptor
    MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();

    modeController->AddLambda_Finished(this, [this, modeController](const bool completed, const uint8_t finishCode){
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
        {
            std::cout<<"The abort is tranistioning to RTL!"<<std::endl;
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

void AP_State_FlightAI_TestEnd::OnEnter(const PLANE_MODE &mode)
{
    //We got into this state because externally the test has already ended and the mode has transitioned.
    //We just need to catch up to the current state. 
    UNUSED(mode); //For right now I would rather have the outer state machine check where to appropriately transition
    static_cast<ardupilot::state::AbstractStateArdupilot*>(GetOutermostState())->checkForDelayedTransition(mode);
    // ardupilot::state::AP_State_Flight* currentOuterState = GetState<ardupilot::state::AP_State_Flight>();
    // currentOuterState->checkForDelayedTransition(mode);
}

void AP_State_FlightAI_TestEnd::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);

}

} //end of namespace ardupilot
} //end of namespace state
