#include "state_flight_guided.h"

namespace ardupilot {
namespace state{

AP_State_FlightGuided::AP_State_FlightGuided():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_GUIDED)
{

}

void AP_State_FlightGuided::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightGuided::getClone() const
{
    return (new AP_State_FlightGuided(*this));
}

void AP_State_FlightGuided::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightGuided(*this);
}

hsm::Transition AP_State_FlightGuided::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_Idle>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_ATTTARGET:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_AttTarget>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_CarTarget>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_GEOTARGET:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_GeoTarget>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_QUEUE:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_Queue>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED_SPATIALITEM:
        {
            rtn = hsm::InnerTransition<AP_State_FlightGuided_SpatialItem>(currentCommand);
            break;
        }
        default:
            std::cout << "I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(_desiredState) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightGuided::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case MAV_CMD::MAV_CMD_USER_1:
    {
        if(this->IsInState<AP_State_FlightGuided_SpatialItem>())
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }
        else
        {
            currentCommand = command->getClone();
            _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_SPATIALITEM;
            success = true;
        }
        break;
    }
    case MAV_CMD::MAV_CMD_USER_5:
    {
        currentCommand = command->getClone();
        command_item::Action_DynamicTarget* cmd = currentCommand->as<command_item::Action_DynamicTarget>();
        switch (cmd->getDynamicTarget()->getTargetType()) {
        case command_target::DynamicTarget::TargetTypes::KINEMATIC:
        {
            command_target::DynamicTarget_Kinematic* currentTarget = cmd->getDynamicTarget()->targetAs<command_target::DynamicTarget_Kinematic>();
            mace::pose::Position* targetPosition = currentTarget->getPosition();
            if(targetPosition != nullptr)
            {
                /* Since the positional element within the command is not null, we have to make sure it
                 * is of the coordinate frame type that can be supported within the appropriate guided mode.
                 * The only explicit case is if the coordinate system is geodetic, if not, the remaining can
                 * all be handled by the cartesian guided state.
                 */
                if(targetPosition->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
                {
                    if(this->IsInState<AP_State_FlightGuided_GeoTarget>())
                    {
                        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
                        success = currentInnerState->handleCommand(command);
                    }
                    else
                    {
                        _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_GEOTARGET;
                        success = true;
                    }
                }
                else
                {
                    if(this->IsInState<AP_State_FlightGuided_CarTarget>())
                    {
                        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
                        success = currentInnerState->handleCommand(command);
                    }
                    else
                    {
                        _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET;
                        success = true;
                    }
                }
            }
            else
            {
                if(this->IsInState<AP_State_FlightGuided_CarTarget>())
                {
                    ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
                    success = currentInnerState->handleCommand(command);
                }
                else
                {
                    _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET;
                    success = true;
                }
            }
            break;
        } //end of case command_target::DynamicTarget::TargetTypes::KINEMATIC
        case command_target::DynamicTarget::TargetTypes::ORIENTATION:
        {
            if(this->IsInState<AP_State_FlightGuided_AttTarget>())
            {
                ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
                success = currentInnerState->handleCommand(command);
            }
            else
            {
                _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_ATTTARGET;
                success = true;
            }
            break;
        } //end of case command_target::DynamicTarget::TargetTypes::ORIENTATION

        } //end of switch statement for target type
        break;
    } //end of case MAV_CMD::MAV_CMD_USER_5

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightGuided::Update()
{

}

void AP_State_FlightGuided::OnEnter()
{
    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
    if(currentModeString != "GUIDED") {
        std::cout<<"Somehow we have entered the guided state early!"<<std::endl;
    }
    else {
        //We have no command and therefore are just in the guided mode, we can tranisition to idle
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
    }
}

void AP_State_FlightGuided::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_guided_idle.h"
#include "plane_flight_states/state_flight_guided_spatial_item.h"
#include "plane_flight_states/state_flight_guided_queue.h"
#include "plane_flight_states/state_flight_guided_target_att.h"
#include "plane_flight_states/state_flight_guided_target_car.h"
#include "plane_flight_states/state_flight_guided_target_geo.h"

