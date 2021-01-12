#include "state_grounded_armed.h"

namespace ardupilot{
namespace state{

State_GroundedArmed::State_GroundedArmed():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_GROUNDED_ARMED;
    desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_ARMED;
}

AbstractStateArdupilot* State_GroundedArmed::getClone() const
{
    return (new State_GroundedArmed(*this));
}

void State_GroundedArmed::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedArmed(*this);
}

hsm::Transition State_GroundedArmed::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_GROUNDED_DISARMING:
        {
            return hsm::SiblingTransition<State_GroundedDisarming>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED_IDLE:
        {
            return hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        case ArdupilotFlightState::STATE_TAKEOFF:
        case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
        case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
        {
            //The takeoff cases are handled as a sibling state transition by the parent state of STATE_GROUNDED
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_GroundedArmed::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    this->clearCommand();
    switch (command->getCommandType()) {
    case command_item::COMMANDTYPE::CI_ACT_ARM:
    {
        if(command->as<command_item::ActionArm>()->getRequestArm() == false)
            desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_DISARMING;
        break;
    }
    case command_item::COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF;
        GetImmediateOuterState()->setDesiredStateEnum(desiredStateEnum);
        static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateOuterState())->setCurrentCommand(command);
        break;
    }
    default:
        break;
    }
}

void State_GroundedArmed::Update()
{
    if(!Owner().state->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED_IDLE;
}

void State_GroundedArmed::OnEnter()
{

}

void State_GroundedArmed::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //When entering this case we will have already armed and therefore have no reason to enter the OnEnter() function
    if(command != nullptr)
        handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded_idle.h"
#include "ardupilot_states/state_grounded_disarming.h"
