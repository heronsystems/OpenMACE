#include "state_takeoff_transitioning.h"

namespace ardupilot {
namespace state{

AP_State_TakeoffTransitioning::AP_State_TakeoffTransitioning():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING)
{
    guidedProgress = ArdupilotTargetProgess(1,10,10);
}

void AP_State_TakeoffTransitioning::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* AP_State_TakeoffTransitioning::getClone() const
{
    return (new AP_State_TakeoffTransitioning(*this));
}

void AP_State_TakeoffTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_TakeoffTransitioning(*this);
}

hsm::Transition AP_State_TakeoffTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_TAKEOFF_COMPLETE:
        {
            rtn = hsm::SiblingTransition<AP_State_TakeoffComplete>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from AP_State_TakeoffTransitioning."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_TakeoffTransitioning::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    clearCommand();
    switch (command->getCommandType()) {
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
    {
        this->currentCommand = command->getClone();
        const command_item::SpatialTakeoff* cmd = currentCommand->as<command_item::SpatialTakeoff>();
        if(cmd->getPosition()->getCoordinateSystemType() == CoordinateSystemTypes::GEODETIC)
        {
            Owner().state->vehicleGlobalPosition.AddNotifier(this, [this, cmd]
            {
                //Since we really care about the translational component we already know we are in the geodetic frame and therefore can straight cast
                mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                double distance = fabs(currentPosition.distanceBetween2D(cmd->getPosition()->positionAs<mace::pose::Abstract_GeodeticPosition>()));

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
//                MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), cmd->getPosition());
//                Owner().callTargetCallback(vehicleTarget);

                if(guidedState == Data::ControllerState::ACHIEVED)
                    _desiredState = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
            });
        }
        else if(cmd->getPosition()->getCoordinateSystemType() == CoordinateSystemTypes::CARTESIAN)
        {
            Owner().state->vehicleLocalPosition.AddNotifier(this, [this, cmd]
            {
                //Since we really care about the translational component we already know we are in the geodetic frame and therefore can straight cast
                mace::pose::CartesianPosition_3D currentPosition = Owner().state->vehicleLocalPosition.get();
                double distance = fabs(currentPosition.distanceBetween2D(cmd->getPosition()->positionAs<mace::pose::Abstract_CartesianPosition>()));

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
//                MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), cmd->getPosition());
//                Owner().callTargetCallback(vehicleTarget);

                if(guidedState == Data::ControllerState::ACHIEVED)
                    _desiredState = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
            });
        }


        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto takeoffTransition = new MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        takeoffTransition->AddLambda_Finished(this, [this,takeoffTransition](const bool completed, const uint8_t finishCode)
        {
            UNUSED(this);
            if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                std::cout<<"We are not going to perform the transition portion of the takeoff."<<std::endl;
            takeoffTransition->Shutdown();
        });

        takeoffTransition->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("takeoffTransition");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        command_item::SpatialWaypoint takeoffTarget(255,cmd->getTargetSystem());
        takeoffTarget.setPosition(cmd->getPosition());
        takeoffTransition->Send(takeoffTarget,sender,target);
        collection->Insert("takeoffTransition",takeoffTransition);
        success = true;

        break;
    }
    default:
        break;
    }

    return success;
}

void AP_State_TakeoffTransitioning::Update()
{

}

void AP_State_TakeoffTransitioning::OnEnter()
{
    //By default I dont think there are any actions that we need to do
}

void AP_State_TakeoffTransitioning::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_takeoff_complete.h"
