#include "state_flight_guided_target_att.h"

namespace ardupilot{
namespace state{

State_FlightGuided_AttTarget::State_FlightGuided_AttTarget():
    AbstractStateArdupilot(), m_TimeoutController(10)
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_ATTTARGET"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_ATTTARGET;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_ATTTARGET;

    m_TimeoutController.connectTargetCallback(State_FlightGuided_AttTarget::retransmitGuidedCommand, this);
    Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, previousTime);

}

void State_FlightGuided_AttTarget::OnExit()
{
    std::cout<<"Exiting the attitude target state"<<std::endl;
    m_TimeoutController.stop();

    AbstractStateArdupilot::OnExit();
    std::cout<<"Exiting the attitude target state 2"<<std::endl;

    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
    if(Owner().ControllersCollection()->Exist("AttitudeTargetController")){
        MAVLINKUXVControllers::ControllerGuidedTargetItem_Attitude* ptr = dynamic_cast<MAVLINKUXVControllers::ControllerGuidedTargetItem_Attitude*>(Owner().ControllersCollection()->Remove("AttitudeTargetController"));
        delete ptr;
    }

    std::cout<<"Exiting the attitude target state 3"<<std::endl;
}

AbstractStateArdupilot* State_FlightGuided_AttTarget::getClone() const
{
    return (new State_FlightGuided_AttTarget(*this));
}

void State_FlightGuided_AttTarget::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_AttTarget(*this);
}

hsm::Transition State_FlightGuided_AttTarget::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED_CARTARGET."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_AttTarget::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool commandHandled = false;

    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_TARGET:
    {
        //stop the current controllers target transmission if it is running
        m_TimeoutController.clearTarget();

        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        //The command is a target, we therefore have to figure out what type of target it is
        command_item::Action_DynamicTarget* cmd = currentCommand->as<command_item::Action_DynamicTarget>();

        command_target::DynamicTarget* currentTarget = cmd->getDynamicTarget();
        if(currentTarget->getTargetType() == command_target::DynamicTarget::TargetTypes::ORIENTATION)
        {
            const mace::pose::AbstractRotation* currentRotation = currentTarget->targetAs<command_target::DynamicTarget_Orientation>()->getTargetOrientation();
            const mace::pose::Rotation_3D* rotation3D = currentRotation->rotationAs<mace::pose::Rotation_3D>();
            Data::EnvironmentTime currentTime;
            Data::EnvironmentTime::CurrentTime(Data::Devices::SYSTEMCLOCK, currentTime);
            uint64_t elapsedTime = currentTime - previousTime;
            count++;
        }

        constructAndSendTarget(*cmd);

        m_TimeoutController.registerCurrentTarget(cmd->getDynamicTarget());

        commandHandled = true;
        break;
    }
    default: //The only command this state is responsible for addressing is COMMANDTYPE::CI_ACT_TARGET
        break;
    }

    return commandHandled;
}

void State_FlightGuided_AttTarget::Update()
{

}

void State_FlightGuided_AttTarget::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null. We therefore will return to the idle state
     * of the guided flight mode.
     */
}

void State_FlightGuided_AttTarget::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr)
    {
        this->OnEnter();
        return;
    }

    if(command->getCommandType() != COMMANDTYPE::CI_ACT_TARGET)
    {
        //we dont handle commands of this type in here
        return;
    }


    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto attitudeTargetController = new MAVLINKUXVControllers::ControllerGuidedTargetItem_Attitude(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());

    collection->Insert("AttitudeTargetController",attitudeTargetController);

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state
