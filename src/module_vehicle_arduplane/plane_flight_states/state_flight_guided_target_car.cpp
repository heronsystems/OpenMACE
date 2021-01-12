#include "state_flight_guided_target_car.h"

namespace ardupilot {
namespace state{

AP_State_FlightGuided_CarTarget::AP_State_FlightGuided_CarTarget():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_FLIGHT_GUIDED_CARTARGET), m_TimeoutController(500)
{
    m_TimeoutController.connectTargetCallback(AP_State_FlightGuided_CarTarget::retransmitGuidedCommand, this);
}

void AP_State_FlightGuided_CarTarget::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleGlobalPosition.RemoveNotifier(this);
    if(Owner().ControllersCollection()->Exist("CartesianTargetController")){
        MAVLINKUXVControllers::ControllerGuidedTargetItem_Local* ptr = dynamic_cast<MAVLINKUXVControllers::ControllerGuidedTargetItem_Local*>(Owner().ControllersCollection()->Remove("CartesianTargetController"));
        delete ptr;
    }

}

AbstractStateArdupilot* AP_State_FlightGuided_CarTarget::getClone() const
{
    return (new AP_State_FlightGuided_CarTarget(*this));
}

void AP_State_FlightGuided_CarTarget::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightGuided_CarTarget(*this);
}

hsm::Transition AP_State_FlightGuided_CarTarget::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_FLIGHT_GUIDED_CARTARGET."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightGuided_CarTarget::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool commandHandled = false;

    switch (command->getCommandType()) {

    case MAV_CMD::MAV_CMD_USER_5:
    {
        //stop the current controllers target transmission if it is running
        m_TimeoutController.clearTarget();

        //We want to keep this command in scope to perform the action
        this->currentCommand = command->getClone();

        //The command is a target, we therefore have to figure out what type of target it is
        command_item::Action_DynamicTarget* cmd = currentCommand->as<command_item::Action_DynamicTarget>();
        if(cmd->getDynamicTarget()->getTargetType() == command_target::DynamicTarget::TargetTypes::KINEMATIC)
        {
            command_target::DynamicTarget_Kinematic* castCommand = cmd->getDynamicTarget()->targetAs<command_target::DynamicTarget_Kinematic>();
            constructAndSendTarget(*cmd);

            /*
             * Determine if the velocity component is valid, and if so, update the timeout controller
             * with the appropriate target to ensure that upon the designated timeout, the controller
             * retransmits the command to the arducopter.
             *
             * NOTE: Arducopter requires that all the velocities be valid
             */
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

void AP_State_FlightGuided_CarTarget::Update()
{

}

void AP_State_FlightGuided_CarTarget::OnEnter()
{
    /*
     * For some reason we have gotten into this state without a command,
     * or the command is null. We therefore will return to the idle state
     * of the guided flight mode.
     */
}

void AP_State_FlightGuided_CarTarget::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command == nullptr)
    {
        this->OnEnter();
        return;
    }

    if(command->getCommandType() != MAV_CMD::MAV_CMD_USER_5)
    {
        //we dont handle commands of this type in here
        return;
    }


    //Insert a new controller only one time in the guided state to manage the entirity of the commands that are of the dynamic target type
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto cartesianTargetController = new MAVLINKUXVControllers::ControllerGuidedTargetItem_Local(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    cartesianTargetController->updateTransformation(Owner().environment->getTransform_SwarmTOVehicleEKF());
    collection->Insert("CartesianTargetController",cartesianTargetController);

    this->handleCommand(command);
}

} //end of namespace ardupilot
} //end of namespace state
