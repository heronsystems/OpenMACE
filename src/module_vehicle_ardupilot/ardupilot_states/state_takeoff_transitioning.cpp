#include "state_takeoff_transitioning.h"

namespace ardupilot{
namespace state{

State_TakeoffTransitioning::State_TakeoffTransitioning():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(1,10,10);
    std::cout<<"We are in the constructor of STATE_TAKEOFF_TRANSITIONING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING;
}

void State_TakeoffTransitioning::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_TakeoffTransitioning::getClone() const
{
    return (new State_TakeoffTransitioning(*this));
}

void State_TakeoffTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffTransitioning(*this);
}

hsm::Transition State_TakeoffTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_TAKEOFF_COMPLETE:
        {
            rtn = hsm::SiblingTransition<State_TakeoffComplete>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_TakeoffTransitioning."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_TakeoffTransitioning::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    clearCommand();
//    switch (command->getCommandType()) {
//    case COMMANDTYPE::CI_NAV_TAKEOFF:
//    {
//        this->currentCommand = command->getClone();
//        const command_item::SpatialTakeoff* cmd = currentCommand->as<command_item::SpatialTakeoff>();
//        if(cmd->getPosition().has3DPositionSet())
//        {
//            Owner().state->vehicleGlobalPosition.AddNotifier(this, [this, cmd]
//            {
//                if(cmd->getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
//                {
//                    StateGlobalPosition cmdPos(cmd->getPosition().getY(),cmd->getPosition().getX(),cmd->getPosition().getZ());
//                    StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
//                    double distance = fabs(currentPosition.distanceBetween2D(cmdPos));

//                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
//                    MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), cmdPos, distance, guidedState);
//                    Owner().callTargetCallback(vehicleTarget);

//                    if(guidedState == Data::ControllerState::ACHIEVED)
//                    {
//                        desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_COMPLETE;
//                    }
//                }
//            });

//            Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
//            auto takeoffTransition = new MAVLINKVehicleControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
//            takeoffTransition->AddLambda_Finished(this, [this,takeoffTransition](const bool completed, const uint8_t finishCode){
//                if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
//                    std::cout<<"We are not going to perform the transition portion of the takeoff."<<std::endl;
//                takeoffTransition->Shutdown();
//            });

//            takeoffTransition->setLambda_Shutdown([this, collection]()
//            {
//                auto ptr = collection->Remove("takeoffTransition");
//                delete ptr;
//            });

//            MavlinkEntityKey target = Owner().getMAVLINKID();
//            MavlinkEntityKey sender = 255;

//            Base3DPosition cmdPosition = cmd->getPosition();
//            command_item::SpatialWaypoint takeoffTarget(255,cmd->getTargetSystem());
//            takeoffTarget.setPosition(cmdPosition);
//            takeoffTransition->Send(takeoffTarget,sender,target);
//            collection->Insert("takeoffTransition",takeoffTransition);
//        }
//        break;
//    }
//    default:
//        break;
//    }
}

void State_TakeoffTransitioning::Update()
{

}

void State_TakeoffTransitioning::OnEnter()
{
    //By default I dont think there are any actions that we need to do
}

void State_TakeoffTransitioning::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_takeoff_complete.h"
