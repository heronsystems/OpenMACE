#include "state_flight_manual.h"

namespace ardupilot {
namespace state{

AP_State_FlightManual::AP_State_FlightManual():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_MANUAL)
{

}

AbstractStateArdupilot* AP_State_FlightManual::getClone() const
{
    return (new AP_State_FlightManual(*this));
}

void AP_State_FlightManual::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightManual(*this);
}

hsm::Transition AP_State_FlightManual::GetTransition()
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
            rtn = hsm::SiblingTransition<AP_State_GroundedDisarming>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_Flight_Manual."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightManual::handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    bool success = false;
    MAV_CMD type = command->getCommandType();

    switch (type) {
    case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM: //This should cause a state transition to the grounded_disarming state
    {
        if(!command->as<command_item::ActionArm>()->getRequestArm())
        {
            _desiredState = Data::MACEHSMState::STATE_GROUNDED_DISARMING;
            success = true;
        }
        break;
    }
    default:
        break;
    }

    return success;

    return false;
}

void AP_State_FlightManual::Update()
{

}

void AP_State_FlightManual::OnEnter()
{

}

void AP_State_FlightManual::OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded_disarming.h"
