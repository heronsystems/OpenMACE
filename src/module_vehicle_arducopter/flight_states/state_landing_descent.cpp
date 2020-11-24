#include "state_landing_descent.h"

namespace ardupilot {
namespace state{

State_LandingDescent::State_LandingDescent():
    AbstractStateArdupilot()
{
    guidedProgress = ArdupilotTargetProgess(0,10,10);
    std::cout<<"We are in the constructor of STATE_LANDING_DESCENT"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_LANDING_DESCENDING;
    desiredStateEnum = Data::MACEHSMState::STATE_LANDING_DESCENDING;
}

void State_LandingDescent::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleLocalPosition.RemoveNotifier(this);
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_LandingDescent::getClone() const
{
    return (new State_LandingDescent(*this));
}

void State_LandingDescent::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_LandingDescent(*this);
}

hsm::Transition State_LandingDescent::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_LANDING_COMPLETE:
        {
            if(currentCommand == nullptr)
            {

                rtn = hsm::SiblingTransition<State_LandingComplete>();
            }
            else
            {
                rtn = hsm::SiblingTransition<State_LandingComplete>(currentCommand);
            }
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_LANDING_DESCENT."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_LandingDescent::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    clearCommand();
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_NAV_LAND:
    {
        currentCommand = command->getClone();
        const command_item::SpatialLand* cmd = currentCommand->as<command_item::SpatialLand>();

        double targetAltitude = 0.0;
        if((cmd->getPosition() != nullptr) && (cmd->getPosition()->is3D()))
        {
            switch (cmd->getPosition()->getCoordinateSystemType()) {
            case CoordinateSystemTypes::GEODETIC:
            {
                const mace::pose::GeodeticPosition_3D* castPosition = cmd->getPosition()->positionAs<mace::pose::GeodeticPosition_3D>();
                if(castPosition->hasAltitudeBeenSet())
                    targetAltitude=castPosition->getAltitude();
                Owner().state->vehicleGlobalPosition.AddNotifier(this,[this,targetAltitude]
                {
                    mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.getAltitude() - targetAltitude);
                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);

                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        this->clearCommand();
                        desiredStateEnum = Data::MACEHSMState::STATE_LANDING_COMPLETE;
                    }
                });
                break;
            }
            case CoordinateSystemTypes::CARTESIAN:
            {
                const mace::pose::CartesianPosition_3D* castPosition = cmd->getPosition()->positionAs<mace::pose::CartesianPosition_3D>();
                if(castPosition->hasZBeenSet())
                    targetAltitude=castPosition->getAltitude();
                Owner().state->vehicleLocalPosition.AddNotifier(this,[this,targetAltitude]
                {
                    mace::pose::CartesianPosition_3D currentPosition = Owner().state->vehicleLocalPosition.get();
                    double distance = fabs(currentPosition.getAltitude() - targetAltitude);
                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);

                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        this->clearCommand();
                        desiredStateEnum = Data::MACEHSMState::STATE_LANDING_COMPLETE;
                    }
                });
                break;
            }
            case CoordinateSystemTypes::NOT_IMPLIED:
            case CoordinateSystemTypes::UNKNOWN:
            {
                break;
            }
            default:
                break;
            }
        }
        else
        {
            Owner().state->vehicleLocalPosition.AddNotifier(this,[this,targetAltitude]
            {
                mace::pose::CartesianPosition_3D currentPosition = Owner().state->vehicleLocalPosition.get();
                double distance = fabs(currentPosition.getAltitude() - targetAltitude);
                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);

                if(guidedState == Data::ControllerState::ACHIEVED)
                {
                    this->clearCommand();
                    desiredStateEnum = Data::MACEHSMState::STATE_LANDING_COMPLETE;
                }
            });
        }


        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerDescent = new MAVLINKUXVControllers::CommandLand(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerDescent->AddLambda_Finished(this, [this,controllerDescent](const bool completed, const uint8_t finishCode){
            if(!completed && (finishCode != MAV_RESULT_ACCEPTED))
                GetImmediateOuterState()->setDesiredStateEnum(Data::MACEHSMState::STATE_FLIGHT);
            controllerDescent->Shutdown();
        });

        controllerDescent->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("landingDescent");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        controllerDescent->Send(*cmd,sender,target);
        collection->Insert("landingDescent", controllerDescent);
        success = true;
        break;
    }
    default:
        break;
    }

    return success;
}

void State_LandingDescent::Update()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(!vehicleStatus->vehicleArm.get().getSystemArm())
        desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_IDLE;


    std::string modeStr = vehicleStatus->vehicleMode.get().getFlightModeString();
    if(modeStr == "BRAKE" || modeStr == "LOITER") {
        desiredStateEnum = Data::MACEHSMState::STATE_LANDING_COMPLETE;
    }
}

void State_LandingDescent::OnEnter()
{

}

void State_LandingDescent::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        handleCommand(command);
    }
    else
        OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_landing_complete.h"
