#include "state_grounded_disarming.h"

namespace ardupilot {
namespace state{

AP_State_GroundedDisarming::AP_State_GroundedDisarming():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_GROUNDED_DISARMING)
{

}

AbstractStateArdupilot* AP_State_GroundedDisarming::getClone() const
{
    return (new AP_State_GroundedDisarming(*this));
}

void AP_State_GroundedDisarming::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_GroundedDisarming(*this);
}

hsm::Transition AP_State_GroundedDisarming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED_DISARMED:
        {
            rtn = hsm::SiblingTransition<AP_State_GroundedDisarmed>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from AP_State_GroundedDisarming."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_GroundedDisarming::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool handledCommand = false;

    MAV_CMD type = command->getCommandType();
    switch (type) {
    case MAV_CMD::MAV_CMD_COMPONENT_ARM_DISARM:
    {
        break;
    }
    default:
        clearCommand();
        currentCommand = command->getClone();
        break;
    }

    return handledCommand;
}

void AP_State_GroundedDisarming::Update()
{
    if(Owner().status->vehicleArm.get().getSystemArm() == false)
    {
        _desiredState = Data::MACEHSMState::STATE_GROUNDED_DISARMED;
    }
}

void AP_State_GroundedDisarming::OnEnter()
{
    //when calling this function that means our intent is to disarm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerArm = new MAVLINKUXVControllers::CommandARM(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerArm->AddLambda_Finished(this, [this,controllerArm](const bool completed, const uint8_t finishCode){
        controllerArm->Shutdown();
        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
            _desiredState = Data::MACEHSMState::STATE_GROUNDED_ARMED;
    });

    controllerArm->setLambda_Shutdown([this, collection]()
    {
        UNUSED(this);
        auto ptr = collection->Remove("disarmController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    command_item::ActionArm action(255, Owner().getMAVLINKID());
    action.setVehicleArm(false);
    controllerArm->Send(action,sender,target);
//    printf("Adding disarmController %x\n", controllerArm);
    collection->Insert("disarmController", controllerArm);
}

void AP_State_GroundedDisarming::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace

#include "plane_flight_states/state_grounded_armed.h"
#include "plane_flight_states/state_grounded_disarmed.h"
