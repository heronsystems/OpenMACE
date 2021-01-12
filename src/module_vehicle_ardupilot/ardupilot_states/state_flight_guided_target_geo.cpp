#include "state_flight_guided_target_geo.h"

namespace ardupilot{
namespace state{

State_FlightGuided_GeoTarget::State_FlightGuided_GeoTarget():
    AbstractStateArdupilot(), m_TimeoutController(500)
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_GEOTARGET"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GEOTARGET;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED_GEOTARGET;

    m_TimeoutController.connectTargetCallback(State_FlightGuided_GeoTarget::retransmitGuidedCommand, this);

}

void State_FlightGuided_GeoTarget::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
    if(Owner().ControllersCollection()->Exist("GeodeticTargetController")){
        MAVLINKUXVControllers::ControllerGuidedTargetItem_Global* ptr = dynamic_cast<MAVLINKUXVControllers::ControllerGuidedTargetItem_Global*>(Owner().ControllersCollection()->Remove("GeodeticTargetController"));
        delete ptr;
    }

}

AbstractStateArdupilot* State_FlightGuided_GeoTarget::getClone() const
{
    return (new State_FlightGuided_GeoTarget(*this));
}

void State_FlightGuided_GeoTarget::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_FlightGuided_GeoTarget(*this);
}

hsm::Transition State_FlightGuided_GeoTarget::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED_GEOTARGET."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_GeoTarget::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
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
        if(cmd->getDynamicTarget()->getTargetType() == command_target::DynamicTarget::TargetTypes::KINEMATIC)
        {
            constructAndSendTarget(*cmd);

            /*
             * Determine if the velocity component is valid, and if so, update the timeout controller
             * with the appropriate target to ensure that upon the designated timeout, the controller
             * retransmits the command to the ardupilot.
             *
             * NOTE: Ardupilot requires that all the velocities be valid
             */
            command_target::DynamicTarget_Kinematic* castCommand = cmd->getDynamicTarget()->targetAs<command_target::DynamicTarget_Kinematic>();
            if((castCommand->getVelocity() != nullptr) && (castCommand->getVelocity()->areAllVelocitiesValid()))
            {
                m_TimeoutController.registerCurrentTarget(cmd->getDynamicTarget());
            }

            commandHandled = true;
        }
        break;
    }
    default:
        break;
    }

    return commandHandled;
}

void State_FlightGuided_GeoTarget::Update()
{

}

void State_FlightGuided_GeoTarget::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null. We therefore will return to the idle state
     * of the guided flight mode.
     */
}

void State_FlightGuided_GeoTarget::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
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

    auto geodeticTargetController = new MAVLINKUXVControllers::ControllerGuidedTargetItem_Global(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());

    collection->Insert("GeodeticTargetController",geodeticTargetController);

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state
