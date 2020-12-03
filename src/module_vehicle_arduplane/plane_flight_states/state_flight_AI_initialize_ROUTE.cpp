#include "state_flight_AI_initialize_ROUTE.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize_ROUTE::AP_State_FlightAI_Initialize_ROUTE():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_INITIALIZE_ROUTE"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ROUTE;
}

void AP_State_FlightAI_Initialize_ROUTE::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Initialize_ROUTE::getClone() const
{
    return (new AP_State_FlightAI_Initialize_ROUTE(*this));
}

void AP_State_FlightAI_Initialize_ROUTE::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize_ROUTE(*this);
}

hsm::Transition AP_State_FlightAI_Initialize_ROUTE::GetTransition()
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
        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Initialize_ROUTE::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize_ROUTE::Update()
{

}

void AP_State_FlightAI_Initialize_ROUTE::OnEnter()
{

}

void AP_State_FlightAI_Initialize_ROUTE::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_AI_abort.h"


