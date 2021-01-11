#include "state_flight_AI_initialize_ABORT.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize_ABORT::AP_State_FlightAI_Initialize_ABORT():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT)
{

}

void AP_State_FlightAI_Initialize_ABORT::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Initialize_ABORT::getClone() const
{
    return (new AP_State_FlightAI_Initialize_ABORT(*this));
}

void AP_State_FlightAI_Initialize_ABORT::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize_ABORT(*this);
}

hsm::Transition AP_State_FlightAI_Initialize_ABORT::GetTransition()
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

bool AP_State_FlightAI_Initialize_ABORT::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize_ABORT::Update()
{

}

void AP_State_FlightAI_Initialize_ABORT::OnEnter()
{
    //This really shouldn't happen
    OnEnter("");
}

void AP_State_FlightAI_Initialize_ABORT::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //This really shouldn't happen
    UNUSED(command);
    OnEnter("");
}

void AP_State_FlightAI_Initialize_ABORT::OnEnter(const std::string &descriptor)
{
    command_item::Action_EventTag writeDetail(LOGGING_EVENT_TAGS::ABORTED_TEST, descriptor);

    MavlinkEntityKey sender = 255;
    if(Owner().GlobalControllersCollection()->Exist("onboardLoggingController"))
        static_cast<MAVLINKUXVControllers::Controller_WriteEventToLog*>(Owner().GlobalControllersCollection()->At("onboardLoggingController"))->Broadcast(writeDetail, sender);
}


} //end of namespace ardupilot
} //end of namespace state
