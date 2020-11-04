#include "state_grounded_disarmed.h"

namespace ardupilot {
namespace state{

State_GroundedDisarmed::State_GroundedDisarmed():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_GROUNDED_DISARMED"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_GROUNDED_DISARMED;
    desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_DISARMED;
}

AbstractStateArdupilot* State_GroundedDisarmed::getClone() const
{
    return (new State_GroundedDisarmed(*this));
}

void State_GroundedDisarmed::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_GroundedDisarmed(*this);
}

hsm::Transition State_GroundedDisarmed::GetTransition()
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
            rtn = hsm::SiblingTransition<State_GroundedIdle>();
            break;
        }
        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_GroundedDisarmed."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_GroundedDisarmed::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_GroundedDisarmed::Update()
{

}

void State_GroundedDisarmed::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        controllerSystemMode->Shutdown();
        //This does not matter as we shall transition to the idle state
        UNUSED(completed); UNUSED(finishCode);
        desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_IDLE;
    });

    controllerSystemMode->setLambda_Shutdown([this, collection]()
    {
        UNUSED(this);
        auto ptr = collection->Remove("State_GroundedDisarmed_modeController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
    commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
    commandMode.vehicleMode = static_cast<uint8_t>(Owner().m_ArdupilotMode->getFlightModeFromString("STABILIZE"));
    controllerSystemMode->Send(commandMode,sender,target);
    collection->Insert("State_GroundedDisarmed_modeController",controllerSystemMode);
}

void State_GroundedDisarmed::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);

    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(vehicleStatus->vehicleMode.get().getFlightModeString() == "STABILIZE")
        desiredStateEnum = Data::MACEHSMState::STATE_GROUNDED_IDLE;
    else
        this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace

#include "flight_states/state_grounded_idle.h"
