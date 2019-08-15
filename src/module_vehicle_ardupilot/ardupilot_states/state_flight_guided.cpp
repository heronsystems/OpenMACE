#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
}

void State_FlightGuided::OnExit()
{

}

AbstractStateArdupilot* State_FlightGuided::getClone() const
{
    return (new State_FlightGuided(*this));
}

void State_FlightGuided::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided(*this);
}

hsm::Transition State_FlightGuided::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided_Idle>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_MISSIONITEM:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided_MissionItem>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_QUEUE:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided_Queue>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    std::cout<<"We are trying to handle the command in the parent state state_flight_guided."<<std::endl;
    ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
    currentInnerState->handleCommand(command);
}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{
    //We have no command and therefore are just in the guided mode, we can tranisition to idle
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_mission_item.h"
#include "ardupilot_states/state_flight_guided_idle.h"
#include "ardupilot_states/state_flight_guided_queue.h"
#include "ardupilot_states/state_flight_guided_target_geo.h"

