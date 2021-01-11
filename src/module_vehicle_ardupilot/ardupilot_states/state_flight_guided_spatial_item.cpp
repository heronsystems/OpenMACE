#include "state_flight_guided_spatial_item.h"

namespace ardupilot{
namespace state{

State_FlightGuided_SpatialItem::State_FlightGuided_SpatialItem():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_SPATIALITEM"<<std::endl;
    guidedProgress = ArdupilotTargetProgess(1,10,10);
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_SPATIALITEM;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_SPATIALITEM;
}

void State_FlightGuided_SpatialItem::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);

    if(Owner().ControllersCollection()->Exist("goToController")){
        MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>* ptr = dynamic_cast<MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>*>(Owner().ControllersCollection()->Remove("goToController"));
        delete ptr;
    }
}

AbstractStateArdupilot* State_FlightGuided_SpatialItem::getClone() const
{
    return (new State_FlightGuided_SpatialItem(*this));
}

void State_FlightGuided_SpatialItem::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_SpatialItem(*this);
}

hsm::Transition State_FlightGuided_SpatialItem::GetTransition()
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
            rtn = hsm::SiblingTransition<State_FlightGuided_Idle>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED_SPATIALITEM."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_SpatialItem::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool processedCommand = false;
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
    {
        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        const command_item::Action_ExecuteSpatialItem* cmd = currentCommand->as<command_item::Action_ExecuteSpatialItem>();
        /*
             * Current the arducopter branch only supports waypoints and yaw commands in this mode. Therefore,
             * we shall perform a preliminary check here to see if the command is of the correct type. If not,
             * the state shall return to a guided idle state.
             */
        if(cmd->getSpatialAction()->getCommandType() != COMMANDTYPE::CI_NAV_WAYPOINT)
        {
            break;
        }
        processSpatialWaypoint();
    }
    default: //The only case this guided state should be designed to address is the executate spatial item
        break;
    }

    return processedCommand;
}

void State_FlightGuided_SpatialItem::Update()
{

}

void State_FlightGuided_SpatialItem::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null, or of not the correct type. We therefore will
     * return to the idle state of the guided flight mode.
     */
}

void State_FlightGuided_SpatialItem::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if((command == nullptr) || (command->getCommandType() != COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM)) //if we are not executing a guided mission item this state doesnt care
    {
        this->OnEnter();
        return;
    }

    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are goto
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto goToController = new MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    goToController->AddLambda_Finished(this, [this](const bool completed, const uint8_t finishCode){
        if(!completed)
        {
            //for some reason a timeout has occured, should we handle this differently
            std::cout<<"A timeout has occured when trying to send a goto command."<<std::endl;
            desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
        }else if(finishCode != MAV_MISSION_ACCEPTED)
        {
            std::cout<<"The autopilot says there is an error with the goTo command."<<std::endl;
            desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
        }
        else if(completed && (finishCode == MAV_MISSION_ACCEPTED))
        {
            this->commandAccepted = true;
        }
    });

    collection->Insert("goToController",goToController);

    this->handleCommand(command);
}


void State_FlightGuided_SpatialItem::processSpatialWaypoint()
{
    const command_item::Action_ExecuteSpatialItem* cmd = currentCommand->as<command_item::Action_ExecuteSpatialItem>();
    AbstractSpatialActionPtr spatialCommand = cmd->getSpatialAction();

    //We want to first assume that this command has not been currently accepted
    this->commandAccepted = false;

    /*
     * Ken Fix: Eventually align the data types to one desired type of position element.
     * Also, it may not be necessary to call up another notifier if another guided command arrives.
     * Although, this is handled in the get/set notifier if the pointer is already the same it
     * just updates the underlying lambda function call
     */

    switch(spatialCommand->getPosition()->getCoordinateSystemType())
    {
    case CoordinateSystemTypes::GEODETIC:
    {
        const mace::pose::Abstract_GeodeticPosition* cmdPosition = spatialCommand->getPosition()->positionAs<mace::pose::Abstract_GeodeticPosition>();

        Owner().state->vehicleGlobalPosition.AddNotifier(this, [this, cmdPosition]
        {
            if(this->commandAccepted){

                mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                double distance = fabs(currentPosition.distanceTo(cmdPosition));

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
            }
        });
        break;
    }
    case CoordinateSystemTypes::CARTESIAN:
    {
        const mace::pose::Abstract_CartesianPosition* cmdPosition = cmd->getSpatialAction()->getPosition()->positionAs<mace::pose::Abstract_CartesianPosition>();

        Owner().state->vehicleLocalPosition.AddNotifier(this, [this, cmdPosition]
        {
            if(this->commandAccepted){

                mace::pose::CartesianPosition_3D currentPosition = Owner().state->vehicleLocalPosition.get();
                double distance = fabs(currentPosition.distanceTo(cmdPosition));

                Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
            }
        });
        break;
    }
    case CoordinateSystemTypes::UNKNOWN:
    case CoordinateSystemTypes::NOT_IMPLIED:
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
        return;
    }

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;
    command_item::SpatialWaypoint waypoint(sender, cmd->getTargetSystem());
    waypoint.setPosition(spatialCommand->getPosition());
    dynamic_cast<MAVLINKUXVControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>*>(Owner().ControllersCollection()->At("goToController"))->Send(waypoint, sender, target);
}

} //end of namespace ardupilot
} //end of namespace state

#include "state_flight_guided_idle.h"
