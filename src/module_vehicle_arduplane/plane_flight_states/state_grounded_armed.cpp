#include "state_grounded_armed.h"

namespace ardupilot {
namespace state{

AP_State_GroundedArmed::AP_State_GroundedArmed():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_GROUNDED_ARMED)
{

}

AbstractStateArdupilot* AP_State_GroundedArmed::getClone() const
{
    return (new AP_State_GroundedArmed(*this));
}

void AP_State_GroundedArmed::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_GroundedArmed(*this);
}

hsm::Transition AP_State_GroundedArmed::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED_DISARMING:
        {
            return hsm::SiblingTransition<AP_State_GroundedDisarming>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_IDLE:
        {
            return hsm::SiblingTransition<AP_State_GroundedIdle>();
            break;
        }
        case Data::MACEHSMState::STATE_TAKEOFF:
        case Data::MACEHSMState::STATE_TAKEOFF_CLIMBING:
        case Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
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

bool AP_State_GroundedArmed::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    this->clearCommand();
    switch (command->getCommandType()) {
    case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM:
    {
        if(command->as<command_item::ActionArm>()->getRequestArm() == false)
        {
            _desiredState = Data::MACEHSMState::STATE_GROUNDED_DISARMING;
            success = true;
        }
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
    {
        _desiredState = Data::MACEHSMState::STATE_TAKEOFF;
        static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateOuterState())->setDesiredStateEnum(_desiredState);
        static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateOuterState())->setCurrentCommand(command);
        success = true;
        break;
    }
    default:
        break;
    }

    return success;
}

void AP_State_GroundedArmed::Update()
{
    if(!Owner().status->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_IDLE;
}

void AP_State_GroundedArmed::OnEnter()
{
    //the command was obviously to arm however, we do not know what the user intent was next
    static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateOuterState())->setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT);
}

void AP_State_GroundedArmed::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //When entering this case we will have already armed and therefore have no reason to enter the OnEnter() function
    if(command != nullptr)
        handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded_idle.h"
#include "plane_flight_states/state_grounded_disarming.h"
