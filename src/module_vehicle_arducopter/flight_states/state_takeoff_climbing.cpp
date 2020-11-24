#include "state_takeoff_climbing.h"

namespace ardupilot {
namespace state{

State_TakeoffClimbing::State_TakeoffClimbing():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_CLIMBING"<<std::endl;
    guidedProgress = ArdupilotTargetProgess(1,10,10);
    currentStateEnum = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
    desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
}

void State_TakeoffClimbing::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_TakeoffClimbing::getClone() const
{
    return (new State_TakeoffClimbing(*this));
}

void State_TakeoffClimbing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_TakeoffClimbing(*this);
}

hsm::Transition State_TakeoffClimbing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
        {
            rtn = hsm::SiblingTransition<State_TakeoffTransitioning>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_TAKEOFF_COMPLETE:
        {
            rtn = hsm::SiblingTransition<State_TakeoffComplete>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_TakeoffClimbing::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        currentCommand = command->getClone();
        double targetAltitude = 0.0;
        const command_item::SpatialTakeoff* cmd = currentCommand->as<command_item::SpatialTakeoff>();
        if(cmd->getPosition()->is3D()) //we only proceed if at a minimum the altitude has been set
        {
            switch (cmd->getPosition()->getCoordinateSystemType()) {
            case CoordinateSystemTypes::GEODETIC:
            {
                const mace::pose::GeodeticPosition_3D* castPosition = cmd->getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
                if(castPosition->hasAltitudeBeenSet()) {
                    targetAltitude = castPosition->getAltitude();
                }
                break;
            }
            case CoordinateSystemTypes::CARTESIAN:
            {
                const mace::pose::CartesianPosition_3D* castPosition = cmd->getPosition()->positionAs<mace::pose::CartesianPosition_3D>();
                if(castPosition->hasZBeenSet()) {
                    targetAltitude = castPosition->getZPosition();
                }
                break;
            }
            case CoordinateSystemTypes::UNKNOWN:
            case CoordinateSystemTypes::NOT_IMPLIED:
            {
                //In these conditions we really don't know how to handle it yet
                break;
            }
            }

            mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
            mace::pose::GeodeticPosition_3D targetPosition(currentPosition.getLatitude(),
                                                           currentPosition.getLongitude(), targetAltitude);


            Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,cmd,targetPosition]
            {
                mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                double distance = fabs(currentPosition.deltaAltitude(&targetPosition));

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
//                MissionTopic::VehicleTargetTopic vehicleTarget(cmd->getTargetSystem(), &targetPosition);
//                Owner().callTargetCallback(vehicleTarget);

                if(guidedState == Data::ControllerState::ACHIEVED)
                {
                    if(cmd->getPosition()->areTranslationalComponentsValid())
                    {
                        desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING;
                    }
                    else
                    {
                        desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
                    }
                }
            });

            Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

            auto controllerClimb = new MAVLINKUXVControllers::CommandTakeoff(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
            controllerClimb->AddLambda_Finished(this, [this,controllerClimb](const bool completed, const uint8_t finishCode){
                if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                    GetImmediateOuterState()->setDesiredStateEnum(Data::MACEHSMState::STATE_GROUNDED);
                controllerClimb->Shutdown();
            });

            controllerClimb->setLambda_Shutdown([this, collection]()
            {
                UNUSED(this);
                auto ptr = collection->Remove("takeoffClimb");
                delete ptr;
            });

            MavlinkEntityKey target  = Owner().getMAVLINKID();
            MavlinkEntityKey sender = 255;

            controllerClimb->Send(*cmd,sender,target);
            collection->Insert("takeoffClimb", controllerClimb);

            success = true;
        }
        else
        {
            //There is no target component to climb to, there are several options in how we can handle this and we shall investigate at a later time
        }
        break;
    }
    default:
        break;
    }

    return success;
}

void State_TakeoffClimbing::Update()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    std::string modeStr = vehicleStatus->vehicleMode.get().getFlightModeString();
    if(modeStr == "BRAKE" || modeStr == "LOITER") {
        desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
    }
}

void State_TakeoffClimbing::OnEnter()
{
    //By default I dont think there are any actions that we need to do
}

void State_TakeoffClimbing::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_takeoff_transitioning.h"
#include "flight_states/state_takeoff_complete.h"
