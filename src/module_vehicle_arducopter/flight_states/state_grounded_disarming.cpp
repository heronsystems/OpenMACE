#include "state_grounded_disarming.h"

namespace arducopter{
namespace state{

State_GroundedDisarming::State_GroundedDisarming():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_DISARMING"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_GROUNDED_DISARMING;
    desiredStateEnum = ArducopterFlightState::STATE_GROUNDED_DISARMING;
}

AbstractStateArducopter* State_GroundedDisarming::getClone() const
{
    return (new State_GroundedDisarming(*this));
}

void State_GroundedDisarming::getClone(AbstractStateArducopter** state) const
{
    *state = new State_GroundedDisarming(*this);
}

hsm::Transition State_GroundedDisarming::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArducopterFlightState::STATE_GROUNDED_DISARMED:
        {
            rtn = hsm::SiblingTransition<State_GroundedDisarmed>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_GroundedDisarming."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_GroundedDisarming::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool handledCommand = false;

    COMMANDTYPE type = command->getCommandType();
    switch (type) {
    case COMMANDTYPE::CI_ACT_ARM:
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

void State_GroundedDisarming::Update()
{
    if(Owner().state->vehicleArm.get().getSystemArm() == false)
    {
        desiredStateEnum = ArducopterFlightState::STATE_GROUNDED_DISARMED;
    }
}

void State_GroundedDisarming::OnEnter()
{
    //when calling this function that means our intent is to disarm the vehicle
    //first let us send this relevant command
    //issue command to controller here, and then setup a callback to handle the result
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerArm = new MAVLINKUXVControllers::CommandARM(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerArm->AddLambda_Finished(this, [this,controllerArm](const bool completed, const uint8_t finishCode){
        controllerArm->Shutdown();
        if(!completed || (finishCode != MAV_RESULT_ACCEPTED))
            desiredStateEnum = ArducopterFlightState::STATE_GROUNDED_ARMED;
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
    printf("Adding disarmController %x\n", controllerArm);
    collection->Insert("disarmController",controllerArm);
}

void State_GroundedDisarming::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

} //end of namespace arducopter
} //end of namespace

#include "flight_states/state_grounded_armed.h"
#include "flight_states/state_grounded_disarmed.h"
