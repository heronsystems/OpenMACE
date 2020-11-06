#include "state_grounded_arming.h"

namespace ardupilot {
namespace state{

AP_State_GroundedArming::AP_State_GroundedArming():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_ARMING"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_GROUNDED_ARMING;
    desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_ARMING;
    armingCheck = false;
}

AbstractStateArdupilot* AP_State_GroundedArming::getClone() const
{
    return (new AP_State_GroundedArming(*this));
}

void AP_State_GroundedArming::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_GroundedArming(*this);
}

hsm::Transition AP_State_GroundedArming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case Data::MACEHSMState::STATE_GROUNDED_IDLE:
        {
            rtn = hsm::SiblingTransition<AP_State_GroundedIdle>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED_ARMED:
        {
            rtn = hsm::SiblingTransition<AP_State_GroundedArmed>(currentCommand);
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from STATE_GROUNDED_ARMING."<<static_cast<int>(desiredStateEnum)<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_GroundedArming::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    this->clearCommand();
    switch (command->getCommandType()) {
    default:
        break;
    }

    return true;
}

void AP_State_GroundedArming::Update()
{
    /** We basically will wait in this state until the vehicle is armed
     * or an additional error has occured forcing the state to advance.
     * However, given that once an acknowledgement has been passed at this
     * point, in all likelyhood a mavlink vehicle is going to arm.
      */

    if(Owner().status->vehicleArm.get().getSystemArm())
    {
        desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_ARMED;
    }
}

void AP_State_GroundedArming::OnEnter()
{
    //when calling this function that means our intent is to arm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    auto controllerArm = new MAVLINKUXVControllers::CommandARM(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerArm->AddLambda_Finished(this, [this, controllerArm](const bool completed, const uint8_t finishCode){
        controllerArm->Shutdown();
        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
            desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_IDLE;
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

void AP_State_GroundedArming::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr) {
        this->currentCommand = command;
    }
}


} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded_idle.h"
#include "plane_flight_states/state_grounded_armed.h"
