#include "state_flight_guided_idle.h"

namespace ardupilot{
namespace state{

State_FlightGuided_Idle::State_FlightGuided_Idle():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_IDLE"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided_Idle::OnExit()
{

}

AbstractStateArdupilot* State_FlightGuided_Idle::getClone() const
{
    return (new State_FlightGuided_Idle(*this));
}

void State_FlightGuided_Idle::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_Idle(*this);
}

hsm::Transition State_FlightGuided_Idle::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO:
        {
            rtn = hsm::SiblingTransition<State_FlightGuided_GoTo>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_FlightGuided_Idle."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Idle::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_GOTO:
    {
        this->currentCommand = command->getClone();
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO;
        break;
    }
    default:
        break;
    }
}

void State_FlightGuided_Idle::Update()
{

}

void State_FlightGuided_Idle::OnEnter()
{

}

void State_FlightGuided_Idle::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_goto.h"
#include "ardupilot_states/state_flight_guided_queue.h"
