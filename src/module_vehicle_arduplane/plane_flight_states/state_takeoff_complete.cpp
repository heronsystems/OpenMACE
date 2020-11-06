#include "state_takeoff_complete.h"

namespace ardupilot {
namespace state{

AP_State_TakeoffComplete::AP_State_TakeoffComplete():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF_COMPLETE"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
    desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
}

AbstractStateArdupilot* AP_State_TakeoffComplete::getClone() const
{
    return (new AP_State_TakeoffComplete(*this));
}

void AP_State_TakeoffComplete::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_TakeoffComplete(*this);
}

hsm::Transition AP_State_TakeoffComplete::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    /*
     * We do not want to cause an immediate sibling transition from within an
     * inner state. Therefore, this transition is completed by the parent. There
     * is nothing left for us to do once within this state and should never
     * transition this far.
     */

    return rtn;
}

bool AP_State_TakeoffComplete::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return true;
}

void AP_State_TakeoffComplete::Update()
{

}

void AP_State_TakeoffComplete::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        controllerSystemMode->Shutdown();
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED;
//        else
//            desiredStateEnum = Data::MACEHSMState::STATE_TAKEOFF_COMPLETE;
    });

    controllerSystemMode->setLambda_Shutdown([this, collection]()
    {
        UNUSED(this);
        auto ptr = collection->Remove("AP_State_TakeoffComplete_modeController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
    commandMode.targetID = Owner().getMAVLINKID();
    commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("GUIDED");
    controllerSystemMode->Send(commandMode,sender,target);
    collection->Insert("AP_State_TakeoffComplete_modeController",controllerSystemMode);
}

void AP_State_TakeoffComplete::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

