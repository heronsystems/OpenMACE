#include "state_flight_guided.h"

namespace ardupilot{
namespace state{

State_FlightGuided::State_FlightGuided():
    AbstractStateArdupilot(), guidedTimeout(nullptr), currentQueue(nullptr)
{
    guidedTimeout = new mavlink::GuidedTimeoutController(this, 1000);
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
}

void State_FlightGuided::OnExit()
{
//    guidedTimeout->stop();
//    delete guidedTimeout;

//    Owner().state->vehicleLocalPosition.RemoveNotifier(this);
//    Owner().mission->currentDynamicQueue_LocalCartesian.RemoveNotifier(this);

//    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
////    auto globalPtr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global<MAVLINKVehicleControllers::TargetControllerStructGlobal>*>(collection->At("globalGuidedController"));
////    if(globalPtr != nullptr)
////        globalPtr->Shutdown();
//    auto localPtr = static_cast<MAVLINKVehicleControllers::ControllerGuidedTargetItem_Local<MAVLINKVehicleControllers::TargetControllerStructLocal>*>(collection->At("localGuidedController"));
//    if(localPtr != nullptr)
//        localPtr->Shutdown();

//    delete currentQueue;
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
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided_GoTo>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_QUEUE:
        {
            rtn = hsm::InnerEntryTransition<State_FlightGuided_Queue>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    std::cout<<"We are trying to handle a command in here."<<std::endl;
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

void State_FlightGuided::initializeNewTargetList()
{
//    MissionItem::MissionKey associatedKey = currentQueue->getAssociatedMissionKey();
//    unsigned int associatedIndex = currentQueue->getAssociatedMissionItem();

//    MissionItem::MissionItemCurrent currentMissionItem(associatedKey,associatedIndex);
//    Owner().getCallbackInterface()->cbi_VehicleMissionItemCurrent(currentMissionItem);

//    unsigned int activeTargetIndex = currentQueue->getDynamicTargetList()->getActiveTargetItem();
//    const TargetItem::CartesianDynamicTarget target = *currentQueue->getDynamicTargetList()->getNextIncomplete();

//    guidedTimeout->updateTarget(target);
//    this->cbiArdupilotTimeout_TargetLocal(target);
}

void State_FlightGuided::handleGuidedState(const mace::pose::CartesianPosition_3D currentPosition, const unsigned int currentTargetIndex,
                                           const Data::ControllerState &state, const double targetDistance)
{
//    if(state == Data::ControllerState::ACHIEVED)
//    {

//        const TargetItem::CartesianDynamicTarget* newTarget = currentQueue->getDynamicTargetList()->markCompletionState(currentTargetIndex,TargetItem::DynamicTargetStorage::TargetCompletion::COMPLETE);
//        if(newTarget == nullptr)
//        {
//            std::cout<<"The are no more points in the queue"<<std::endl;
//            //if there are no more points in the queue this mission item is completed
//            MissionItem::MissionItemAchieved achievement(currentQueue->getAssociatedMissionKey(),currentQueue->getAssociatedMissionItem());
//            std::shared_ptr<MissionTopic::MissionItemReachedTopic> ptrMissionTopic = std::make_shared<MissionTopic::MissionItemReachedTopic>(achievement);
//            Owner().getCallbackInterface()->cbi_VehicleMissionData(Owner().getMAVLINKID(),ptrMissionTopic);
//        }
//        else //there is a new target
//        {
//            std::cout<<"The is another point in the queue"<<std::endl;
//            unsigned int currentTargetIndex = currentQueue->getDynamicTargetList()->getActiveTargetItem();
//            const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getTargetPointerAtIndex(currentTargetIndex);

//            //update the people that we have a new target
//            guidedTimeout->updateTarget(*target);
//            this->cbiArdupilotTimeout_TargetLocal(*target);

//            double distance = currentPosition.distanceBetween3D(target->getPosition());
//            Data::ControllerState guidedState = guidedProgress.newTargetItem(distance);
//            handleGuidedState(currentPosition, currentTargetIndex, guidedState, distance);
//        }
//        //advance to the next desired dynamic state
//    }
//    else //we are either hunting or tracking the state
//    {
//        if(Owner().mission->vehicleHomePosition.hasBeenSet())
//        {
//            const TargetItem::CartesianDynamicTarget* target = currentQueue->getDynamicTargetList()->getTargetPointerAtIndex(currentTargetIndex);
//            announceTargetState(*target,targetDistance);
//        }

//    }
}

void State_FlightGuided::announceTargetState(const command_target::DynamicTarget &target, const double &targetDistance)
{
//    command_item::SpatialHome home = Owner().mission->vehicleHomePosition.get();
//    mace::pose::GeodeticPosition_3D homePos(home.getPosition().getX(),home.getPosition().getY(),home.getPosition().getZ());
//    mace::pose::GeodeticPosition_3D targetPos;
//    DynamicsAid::LocalPositionToGlobal(homePos,target.getPosition(),targetPos);

//    Base3DPosition targetPositionCast(targetPos.getLatitude(),targetPos.getLongitude(),targetPos.getAltitude());
//    targetPositionCast.setCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT);
//    MissionTopic::VehicleTargetTopic currentTarget(Owner().getMAVLINKID(),targetPositionCast, targetDistance);
//    Owner().callTargetCallback(currentTarget);
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_goto.h"
#include "ardupilot_states/state_flight_guided_idle.h"
#include "ardupilot_states/state_flight_guided_queue.h"

