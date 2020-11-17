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

}

void AP_State_FlightAI_Abort::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{

}



} //end of namespace ardupilot
} //end of namespace state


