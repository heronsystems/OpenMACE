#include "state_grounded_idle.h"

namespace ardupilot {
namespace state{

AP_State_GroundedIdle::AP_State_GroundedIdle():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_GROUNDED_IDLE)
{

}

AbstractStateArdupilot* AP_State_GroundedIdle::getClone() const
{
    return (new AP_State_GroundedIdle(*this));
}

void AP_State_GroundedIdle::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_GroundedIdle(*this);
}

hsm::Transition AP_State_GroundedIdle::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED_ARMING:
        {
            return hsm::SiblingTransition<AP_State_GroundedArming>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_ARMED:
        {
            return hsm::SiblingTransition<AP_State_GroundedArmed>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from AP_State_GroundedIdle."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_GroundedIdle::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    MAV_CMD type = command->getCommandType();

    switch (type) {
    case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM: //This should cause a state transition to the grounded_arming state
    {
        if(command->as<command_item::ActionArm>()->getRequestArm())
        {
            _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMING;
            success = true;
        }
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF: //This should cause a state transition to the grounded_arming state
    {
        //This is a case where we want to walk all the way through arming to takeoff
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMING;
        this->clearCommand();
        currentCommand = command;
        success = true;
    }
    default:
        break;
    }

    return success;
}

void AP_State_GroundedIdle::Update()
{
    if(Owner().status->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMED;
}

void AP_State_GroundedIdle::OnEnter()
{


}

void AP_State_GroundedIdle::OnExit()
{
    AbstractStateArdupilot::OnExit();
}

void AP_State_GroundedIdle::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();

    if(command != nullptr)
    {

    }
    else{
        this->OnEnter();
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded_arming.h"
#include "plane_flight_states/state_grounded_armed.h"
