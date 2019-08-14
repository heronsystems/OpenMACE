#include "state_flight_guided_idle.h"

namespace ardupilot{
namespace state{

State_FlightGuided_Idle::State_FlightGuided_Idle():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_IDLE"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided_Idle::OnExit()
{

}

AbstractStateArdupilot* State_FlightGuided_Idle::getClone() const
{
    return (new State_FlightGuided_Idle(*this));
}

void State_FlightGuided_Idle::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_Idle(*this);
}

hsm::Transition State_FlightGuided_Idle::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO:
        {
            rtn = hsm::SiblingTransition<State_FlightGuided_GoTo>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED_TARGET:
        {
            rtn = hsm::SiblingTransition<State_FlightGuided_Target>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_FlightGuided_Idle."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Idle::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_GOTO:
    {
        this->currentCommand = command->getClone();
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GOTO;
        break;
    }
    case COMMANDTYPE::CI_ACT_TARGET:
    {
        this->currentCommand = command->getClone();
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_TARGET;
        break;
    }
    default:
        break;
    }
}

void State_FlightGuided_Idle::Update()
{

}

void State_FlightGuided_Idle::OnEnter()
{

    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto positionRequestController = new MAVLINKVehicleControllers::CommandMSGInterval(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    positionRequestController->AddLambda_Finished(this, [this,positionRequestController](const bool completed, const uint8_t finishCode){
        std::cout<<"We have received an acknowledgement of our request."<<std::endl;
        UNUSED(this); UNUSED(completed); UNUSED(finishCode);
        //we can check if they match and try again
        positionRequestController->Shutdown();
    });

    positionRequestController->setLambda_Shutdown([this, collection]()
    {
        UNUSED(this);
        auto ptr = collection->Remove("PositionIntervalRequest");
        delete ptr;
    });

    collection->Insert("PositionIntervalRequest",positionRequestController);


    MavlinkEntityKey target  = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    command_item::ActionMessageInterval msgInterval;
    msgInterval.setMessageID(85);
    msgInterval.setTargetSystem(Owner().getMAVLINKID());
    msgInterval.setMessageInterval(500000);
    positionRequestController->Send(msgInterval,sender,target);

}

void State_FlightGuided_Idle::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_guided_goto.h"
#include "ardupilot_states/state_flight_guided_queue.h"
#include "ardupilot_states/state_flight_guided_target_geo.h"
