#include "state_flight_guided_target.h"

namespace ardupilot{
namespace state{

State_FlightGuided_Target::State_FlightGuided_Target():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_TARGET"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_TARGET;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_TARGET;
}

void State_FlightGuided_Target::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
}

AbstractStateArdupilot* State_FlightGuided_Target::getClone() const
{
    return (new State_FlightGuided_Target(*this));
}

void State_FlightGuided_Target::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_Target(*this);
}

hsm::Transition State_FlightGuided_Target::GetTransition()
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
            std::cout<<"I dont know how we eneded up in this transition state from State_FlightGuided_Target."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Target::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
        case COMMANDTYPE::CI_ACT_TARGET:
        {

            //We want to keep this command in scope to perform the action
            this->currentCommand = command->getClone();

            const command_item::Action_DynamicTarget* cmd = currentCommand->as<command_item::Action_DynamicTarget>();

            MavlinkEntityKey target = Owner().getMAVLINKID();
            MavlinkEntityKey sender = 255;
            MAVLINKVehicleControllers::TargetControllerStruct_Global tgt;
            tgt.targetID = static_cast<uint8_t>(target);
            tgt.target = cmd->getDynamicTarget();
            ((MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global*)Owner().ControllersCollection()->At("GuidedGeodeticController"))->Send(tgt, sender, target);
        }
        default:
            break;
        }
}

void State_FlightGuided_Target::Update()
{

}

void State_FlightGuided_Target::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null. We therefore will return to the idle state
     * of the guided flight mode.
     */
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided_Target::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr)
    {
        this->OnEnter();
        return;
    }

    if(command->getCommandType() != COMMANDTYPE::CI_ACT_TARGET)
    {
        //we dont handle commands of this type in here
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
        return;
    }


    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto geodeticTargetController = new MAVLINKVehicleControllers::ControllerGuidedTargetItem_Global(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
       geodeticTargetController->AddLambda_Finished(this, [this,geodeticTargetController](const bool completed, const uint8_t finishCode){

           UNUSED(geodeticTargetController);
           std::cout<<"We have finished transmitting the guided state command."<<std::endl;
       });

       geodeticTargetController->setLambda_Shutdown([this, collection]()
       {
           UNUSED(this);
           auto ptr = collection->Remove("GuidedGeodeticController");
           delete ptr;
       });

       collection->Insert("GuidedGeodeticController",geodeticTargetController);

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_idle.h"
#include "ardupilot_states/state_flight_guided_queue.h"
