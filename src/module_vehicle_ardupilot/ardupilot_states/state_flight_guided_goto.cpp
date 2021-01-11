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
            std::cout<<"I dont know how we ended up in this transition state from State_EStop."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_GoTo::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
    case COMMANDITEM::CI_ACT_GOTO:
    {

        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        const CommandItem::CommandGoTo* cmd = currentCommand->as<CommandItem::CommandGoTo>();

        //We want to first assume that this command has not been currently accepted
        this->commandAccepted = false;

        //Ken Fix: This position object is going to be assumed to be global geodetic with relative alt
        //We should consolidate this to the correct position type and then perform the transform here to make the command consistent
        //Also this should only use waypoints for now
        Base3DPosition cmdPosition = cmd->getSpatialCommand()->getPosition();
        int targetSystem = cmd->getTargetSystem();

        /*
         * Ken Fix: Eventually align the data types to one desired type of position element.
         * Also, it may not be necessary to call up another notifier if another guided command arrives.
         * Although, this is handled in the get/set notifier if the pointer is already the same it
         * just updates the underlying lambda function call
         */

        Owner().state->vehicleGlobalPosition.AddNotifier(this, [this, cmdPosition, targetSystem]
        {
            if(this->commandAccepted){
                if(cmdPosition.getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
                {
                    StateGlobalPosition cmdPos(cmdPosition.getY(),cmdPosition.getX(),cmdPosition.getZ());
                    StateGlobalPosition currentPosition = Owner().state->vehicleGlobalPosition.get();
                    double distance = fabs(currentPosition.distanceBetween3D(cmdPos));

                    Data::ControllerState guidedState = guidedProgress.updateTargetState(distance);
                    MissionTopic::VehicleTargetTopic vehicleTarget(targetSystem, cmdPos, distance, guidedState);
                    Owner().callTargetCallback(vehicleTarget);

                    if(guidedState == Data::ControllerState::ACHIEVED)
                    {
                        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
                    }

                }
                else{
                    std::cout<<"At the moment we cannot support goTo types of different coordinate frames."<<std::endl;
                }
            }

        });

        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto goToController = new MAVLINKVehicleControllers::ControllerGuidedMissionItem<CommandItem::SpatialWaypoint>(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        goToController->AddLambda_Finished(this, [this,goToController](const bool completed, const uint8_t finishCode){
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

            goToController->Shutdown();
        });

        goToController->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("goToController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;
        CommandItem::SpatialWaypoint waypoint(sender, cmd->getTargetSystem());
        waypoint.setPosition(cmdPosition);
        goToController->Send(waypoint, sender, target);
        collection->Insert("goToController",goToController);
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

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_idle.h"
#include "ardupilot_states/state_flight_guided_queue.h"
