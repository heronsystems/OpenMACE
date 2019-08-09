#include "state_flight_guided_goto.h"

namespace ardupilot{
namespace state{

State_FlightGuided_GoTo::State_FlightGuided_GoTo():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_GOTO"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO;
}

void State_FlightGuided_GoTo::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
    dynamic_cast<MAVLINKVehicleControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>*>(Owner().ControllersCollection()->At("goToController"))->Shutdown();
}

AbstractStateArdupilot* State_FlightGuided_GoTo::getClone() const
{
    return (new State_FlightGuided_GoTo(*this));
}

void State_FlightGuided_GoTo::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_GoTo(*this);
}

hsm::Transition State_FlightGuided_GoTo::GetTransition()
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
            rtn = hsm::SiblingTransition<State_FlightGuided_Idle>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_GoTo::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
        case COMMANDTYPE::CI_ACT_GOTO:
        {
            //We want to keep this command in scope to perform the action
            this->currentCommand = command->getClone();

            const command_item::CommandGoTo* cmd = currentCommand->as<command_item::CommandGoTo>();

            //We want to first assume that this command has not been currently accepted
            this->commandAccepted = false;

            //Ken Fix: This position object is going to be assumed to be global geodetic with relative alt
            //We should consolidate this to the correct position type and then perform the transform here to make the command consistent
            //Also this should only use waypoints for now
            unsigned int targetSystem = cmd->getTargetSystem();

            /*
             * Ken Fix: Eventually align the data types to one desired type of position element.
             * Also, it may not be necessary to call up another notifier if another guided command arrives.
             * Although, this is handled in the get/set notifier if the pointer is already the same it
             * just updates the underlying lambda function call
             */
            switch(cmd->getSpatialCommand()->getPosition()->getCoordinateSystemType())
            {
            case CoordinateSystemTypes::GEODETIC:
            {
                const mace::pose::Abstract_GeodeticPosition* cmdPosition = cmd->getSpatialCommand()->getPosition()->positionAs<mace::pose::Abstract_GeodeticPosition>();

                Owner().state->vehicleGlobalPosition.AddNotifier(this, [this, cmdPosition, targetSystem]
                {
                    if(this->commandAccepted){

                        mace::pose::GeodeticPosition_3D currentPosition = Owner().state->vehicleGlobalPosition.get();
                        double distance = fabs(currentPosition.distanceTo(cmdPosition));

                        Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                        MissionTopic::VehicleTargetTopic vehicleTarget(targetSystem, cmdPosition, distance, guidedState);
                        Owner().callTargetCallback(vehicleTarget);

                        if(guidedState == Data::ControllerState::ACHIEVED)
                        {
                            desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
                        }
                    }

                });

                MavlinkEntityKey target = Owner().getMAVLINKID();
                MavlinkEntityKey sender = 255;
                command_item::SpatialWaypoint waypoint(sender, targetSystem);
                waypoint.setPosition(cmdPosition);
                dynamic_cast<MAVLINKVehicleControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>*>(Owner().ControllersCollection()->At("goToController"))->Send(waypoint, sender, target);
                break;
            }
            case CoordinateSystemTypes::CARTESIAN:
            case CoordinateSystemTypes::UNKNOWN:
            case CoordinateSystemTypes::NOT_IMPLIED:
            {
                break;
            }

            }
        }
        default:
            break;
        }
}

void State_FlightGuided_GoTo::Update()
{

}

void State_FlightGuided_GoTo::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null. We therefore will return to the idle state
     * of the guided flight mode.
     */
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided_GoTo::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr)
    {
        this->OnEnter();
        return;
    }

    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are goto
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
       auto goToController = new MAVLINKVehicleControllers::ControllerGuidedMissionItem<command_item::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
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

       goToController->setLambda_Shutdown([this, collection]()
       {
           UNUSED(this);
           auto ptr = collection->Remove("goToController");
           delete ptr;
       });

       collection->Insert("goToController",goToController);

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_idle.h"
#include "ardupilot_states/state_flight_guided_queue.h"
