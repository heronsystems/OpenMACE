#include "state_grounded_armed.h"

namespace ardupilot {
namespace state{

AP_State_GroundedArmed::AP_State_GroundedArmed():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMED"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_GROUNDED_ARMED;
    desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_ARMED;
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

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
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
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
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
    case command_item::COMMANDTYPE::CI_ACT_ARM:
    {
        if(command->as<command_item::ActionArm>()->getRequestArm() == false)
        {
            desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_DISARMING;
            success = true;
        }
        break;
    }
    case command_item::COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF;
        GetImmediateOuterState()->setDesiredStateEnum(desiredStateEnum);
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
        desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_IDLE;
}

void AP_State_GroundedArmed::OnEnter()
{
    //the command was obviously to arm however, we do not know what the user intent was next
    GetImmediateOuterState()->setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT);
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
