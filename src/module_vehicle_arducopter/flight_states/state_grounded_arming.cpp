#include "state_grounded_arming.h"

namespace ardupilot {
namespace state{

State_GroundedArming::State_GroundedArming():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_GROUNDED_ARMING)
{

}

AbstractStateArdupilot* State_GroundedArming::getClone() const
{
    return (new State_GroundedArming(*this));
}

void State_GroundedArming::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedArming(*this);
}

hsm::Transition State_GroundedArming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED_IDLE:
        {
            rtn = hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_ARMED:
        {
            rtn = hsm::SiblingTransition<State_GroundedArmed>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_GROUNDED_ARMING."<<static_cast<int>(_desiredState)<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_GroundedArming::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    this->clearCommand();
    switch (command->getCommandType()) {
    default:
        break;
    }
    return true;
}

void State_GroundedArming::Update()
{
    /** We basically will wait in this state until the vehicle is armed
     * or an additional error has occured forcing the state to advance.
     * However, given that once an acknowledgement has been passed at this
     * point, in all likelyhood a mavlink vehicle is going to arm.
      */

    if(Owner().status->vehicleArm.get().getSystemArm())
    {
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMED;
    }
}

void State_GroundedArming::OnEnter()
{
    //when calling this function that means our intent is to arm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto controllerArm = new MAVLINKUXVControllers::CommandARM(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerArm->AddLambda_Finished(this, [this, controllerArm](const bool completed, const uint8_t finishCode){
        controllerArm->Shutdown();
        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
            _desiredState = Data::MACEHSMState::STATE_GROUNDED_IDLE;
    });

    controllerArm->setLambda_Shutdown([collection]()
    {
        auto ptr = collection->Remove("armController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    command_item::ActionArm action(255, Owner().getMAVLINKID());
    action.setVehicleArm(true);
    controllerArm->Send(action, sender, target);
    collection->Insert("armController", controllerArm);
}

void State_GroundedArming::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr) {
        this->currentCommand = command;
    }
}


} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_grounded_idle.h"
#include "flight_states/state_grounded_armed.h"
