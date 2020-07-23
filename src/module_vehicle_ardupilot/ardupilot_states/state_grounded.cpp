#include "state_grounded.h"

namespace ardupilot{
namespace state{

State_Grounded::State_Grounded():
    AbstractRootState()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_GROUNDED;
    desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
}

AbstractStateArdupilot* State_Grounded::getClone() const
{
    return (new State_Grounded(*this));
}

void State_Grounded::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Grounded(*this);
}

hsm::Transition State_Grounded::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::InnerEntryTransition<State_GroundedIdle>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMING:
        {
            return hsm::InnerEntryTransition<State_GroundedArming>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_ARMED:
        {
            return hsm::InnerEntryTransition<State_GroundedArmed>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_DISARMING:
        {
            return hsm::InnerEntryTransition<State_GroundedDisarming>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_DISARMED:
        {
            return hsm::InnerEntryTransition<State_GroundedDisarmed>();
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF:
        case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            std::cout<<"We should transition to the takeoff state!"<<std::endl;
            return hsm::SiblingTransition<State_Takeoff>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT:
        {
            std::cout<<"We are transitioning immediately from grounded to flight state."<<std::endl;
            return hsm::SiblingTransition<State_Flight>();
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_GROUNDED."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_Grounded::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    COMMANDTYPE commandType = command->getCommandType();
    switch (commandType) {
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    case COMMANDTYPE::CI_NAV_HOME:
    {
        AbstractRootState::handleCommand(command);
        break;
    }
    default:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        currentInnerState->handleCommand(command);
        break;
    }
    } //end of switch statement
}

void State_Grounded::Update()
{
    //this update should continue to check if the vehicle is not armed and as such remain in this state
}

void State_Grounded::OnEnter()
{
    StateData_MAVLINK* vehicleData = Owner().state;

    if(Owner().state->vehicleArm.get().getSystemArm())
    {
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_ARMED;
    }
    else if((Owner().state->vehicleArm.hasBeenSet()) && (!Owner().state->vehicleArm.get().getSystemArm())
            && (vehicleData->vehicleMode.get().getFlightModeString() != "STABILIZE"))
    {
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_DISARMED;
    }
    else
    {
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_IDLE;
    }
}

void State_Grounded::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_armed.h"
#include "ardupilot_states/state_grounded_arming.h"
#include "ardupilot_states/state_grounded_disarming.h"
#include "ardupilot_states/state_grounded_disarmed.h"
#include "ardupilot_states/state_grounded_idle.h"

#include "ardupilot_states/state_takeoff.h"

#include "ardupilot_states/state_flight.h"
