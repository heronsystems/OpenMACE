#include "state_flight_guided.h"

namespace arducopter{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED;
}

void State_FlightGuided::OnExit()
{

}

AbstractStateArducopter* State_FlightGuided::getClone() const
{
    return (new State_FlightGuided(*this));
}

void State_FlightGuided::getClone(AbstractStateArducopter** state) const
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
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_IDLE:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_Idle>(currentCommand);
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_ATTTARGET:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_AttTarget>(currentCommand);
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_CARTARGET:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_CarTarget>(currentCommand);
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_GEOTARGET:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_GeoTarget>(currentCommand);
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_QUEUE:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_Queue>(currentCommand);
            break;
        }
        case ArducopterFlightState::STATE_FLIGHT_GUIDED_SPATIALITEM:
        {
            rtn = hsm::InnerTransition<State_FlightGuided_SpatialItem>(currentCommand);
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
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
    {
        if(this->IsInState<State_FlightGuided_SpatialItem>())
        {
            arducopter::state::AbstractStateArducopter* currentInnerState = static_cast<arducopter::state::AbstractStateArducopter*>(GetImmediateInnerState());
            currentInnerState->handleCommand(command);
        }
        else
        {
            currentCommand = command->getClone();
            desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_SPATIALITEM;
        }
        break;
    }
    case COMMANDTYPE::CI_ACT_TARGET:
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
                    if(this->IsInState<State_FlightGuided_GeoTarget>())
                    {
                        arducopter::state::AbstractStateArducopter* currentInnerState = static_cast<arducopter::state::AbstractStateArducopter*>(GetImmediateInnerState());
                        currentInnerState->handleCommand(command);
                    }
                    else
                        desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_GEOTARGET;
                }
                else
                {
                    if(this->IsInState<State_FlightGuided_CarTarget>())
                    {
                        arducopter::state::AbstractStateArducopter* currentInnerState = static_cast<arducopter::state::AbstractStateArducopter*>(GetImmediateInnerState());
                        currentInnerState->handleCommand(command);
                    }
                    else
                        desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_CARTARGET;
                }
            }
            else
            {
                if(this->IsInState<State_FlightGuided_CarTarget>())
                {
                    arducopter::state::AbstractStateArducopter* currentInnerState = static_cast<arducopter::state::AbstractStateArducopter*>(GetImmediateInnerState());
                    currentInnerState->handleCommand(command);
                }
                else
                    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_CARTARGET;
            }
            break;
        } //end of case command_target::DynamicTarget::TargetTypes::KINEMATIC
        case command_target::DynamicTarget::TargetTypes::ORIENTATION:
        {
            if(this->IsInState<State_FlightGuided_AttTarget>())
            {
                arducopter::state::AbstractStateArducopter* currentInnerState = static_cast<arducopter::state::AbstractStateArducopter*>(GetImmediateInnerState());
                currentInnerState->handleCommand(command);
            }
            else
                desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_ATTTARGET;
            break;
        } //end of case command_target::DynamicTarget::TargetTypes::ORIENTATION

        } //end of switch statement for target type
        break;
    } //end of case COMMANDTYPE::CI_ACT_TARGET

    default:
        break;

    } //end of switch statement command type
}

void State_FlightGuided::Update()
{

}

void State_FlightGuided::OnEnter()
{
    //We have no command and therefore are just in the guided mode, we can tranisition to idle
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //If we have a command we probably will be entering a state let us figure it out
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state

#include "flight_states/state_flight_guided_idle.h"
#include "flight_states/state_flight_guided_spatial_item.h"
#include "flight_states/state_flight_guided_queue.h"
#include "flight_states/state_flight_guided_target_att.h"
#include "flight_states/state_flight_guided_target_car.h"
#include "flight_states/state_flight_guided_target_geo.h"

