#include "state_grounded.h"

namespace ardupilot {
namespace state{

AP_State_Grounded::AP_State_Grounded():
    AbstractRootState(Data::MACEHSMState::STATE_GROUNDED)
{

}

AbstractStateArdupilot* AP_State_Grounded::getClone() const
{
    return (new AP_State_Grounded(*this));
}

void AP_State_Grounded::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_Grounded(*this);
}

hsm::Transition AP_State_Grounded::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED_IDLE:
        {
            return hsm::InnerEntryTransition<AP_State_GroundedIdle>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_ARMING:
        {
            return hsm::InnerEntryTransition<AP_State_GroundedArming>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_ARMED:
        {
            return hsm::InnerEntryTransition<AP_State_GroundedArmed>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_DISARMING:
        {
            return hsm::InnerEntryTransition<AP_State_GroundedDisarming>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_DISARMED:
        {
            return hsm::InnerEntryTransition<AP_State_GroundedDisarmed>();
            break;
        }
        case Data::MACEHSMState::STATE_TAKEOFF:
        case Data::MACEHSMState::STATE_TAKEOFF_CLIMBING:
        case Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
        {
            std::cout<<"We should transition to the takeoff state!"<<std::endl;
            return hsm::SiblingTransition<AP_State_Takeoff>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT:
        {
            std::cout<<"We are transitioning immediately from grounded to flight state."<<std::endl;
            return hsm::SiblingTransition<AP_State_Flight>();
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_GROUNDED."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_Grounded::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    MAV_CMD commandType = command->getCommandType();
    switch (commandType) {
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
    case MAV_CMD::MAV_CMD_DO_SET_HOME:
    {
        success = AbstractRootState::handleCommand(command);
        break;
    }
    default:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        success = currentInnerState->handleCommand(command);
        break;
    }
    } //end of switch statement

    return success;
}

void AP_State_Grounded::Update()
{
    //this update should continue to check if the vehicle is not armed and as such remain in this state
}

void AP_State_Grounded::OnEnter()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(Owner().status->vehicleArm.get().getSystemArm())
    {
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMED;
    }
    else if((Owner().status->vehicleArm.hasBeenSet()) && (!Owner().status->vehicleArm.get().getSystemArm())
            && (vehicleStatus->vehicleMode.get().getFlightModeString() != "STABILIZE"))
    {
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_DISARMED;
    }
    else
    {
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_IDLE;
    }
}

void AP_State_Grounded::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded_armed.h"
#include "plane_flight_states/state_grounded_arming.h"
#include "plane_flight_states/state_grounded_disarming.h"
#include "plane_flight_states/state_grounded_disarmed.h"
#include "plane_flight_states/state_grounded_idle.h"

#include "plane_flight_states/state_takeoff.h"

#include "plane_flight_states/state_flight.h"
