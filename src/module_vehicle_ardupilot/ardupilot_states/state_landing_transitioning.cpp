#include "state_landing_transitioning.h"

namespace ardupilot{
namespace state{

State_LandingTransitioning::State_LandingTransitioning():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(1,10,10);
    std::cout<<"We are in the constructor of STATE_LANDING_TRANSITIONING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
}

void State_LandingTransitioning::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_LandingTransitioning::getClone() const
{
    return (new State_LandingTransitioning(*this));
}

void State_LandingTransitioning::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingTransitioning(*this);
}

hsm::Transition State_LandingTransitioning::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingTransitioning::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_NAV_LAND:
    {
        currentCommand = command->getClone();
        const command_item::SpatialLand* cmd = currentCommand->as<command_item::SpatialLand>();
        if(cmd->getPosition()->areTranslationalComponentsValid())
        {
            switch (cmd->getPosition()->getCoordinateSystemType()) {
            case(CoordinateSystemTypes::GEODETIC):
            {
                Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd]
                {
                    mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.distanceBetween2D(cmd->getPosition()->positionAs<mace::pose::Abstract_GeodeticPosition>()));

                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);

//                    MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), cmd->getPosition());
//                    Owner().callTargetCallback(vehicleTarget);

                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
                    }
                     });
                break;
            }
            case(CoordinateSystemTypes::CARTESIAN):
            {
                desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
                break;
            }
            case(CoordinateSystemTypes::UNKNOWN):
            case (CoordinateSystemTypes::NOT_IMPLIED):
            {
                desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
                break;
            }
            }

            Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
            auto landingTransitioning = new MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
            landingTransitioning->AddLambda_Finished(this, [this,landingTransitioning](const bool completed, const uint8_t finishCode){
                UNUSED(this);
                if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                    std::cout<<"We are not going to perform the transition portion of the landing."<<std::endl;
                landingTransitioning->Shutdown();
            });

            landingTransitioning->setLambda_Shutdown([this, collection]()
            {
                UNUSED(this);
                auto ptr = collection->Remove("landingTransition");
                delete ptr;
            });

            MavlinkEntityKey target = Owner().getMAVLINKID();
            MavlinkEntityKey sender = 255;

            command_item::SpatialWaypoint landingTarget(255,static_cast<uint8_t>(cmd->getTargetSystem()));
            landingTarget.setPosition(cmd->getPosition());
            landingTransitioning->Send(landingTarget,sender,target);
            collection->Insert("landingTransition",landingTransitioning);
        }
        break;
    }
    default:
        break;
    }
}

void State_LandingTransitioning::Update()
{

}

void State_LandingTransitioning::OnEnter()
{

}

void State_LandingTransitioning::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state
