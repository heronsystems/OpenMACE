#include "state_flight_AI_initialize_ABORT.h"

namespace ardupilot {
namespace state{

AP_State_FlightAI_Initialize_ABORT::AP_State_FlightAI_Initialize_ABORT():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_AI_INITIALIZE_ABORT"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_AI_INITIALIZE_ABORT;
}

void AP_State_FlightAI_Initialize_ABORT::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightAI_Initialize_ABORT::getClone() const
{
    return (new AP_State_FlightAI_Initialize_ABORT(*this));
}

void AP_State_FlightAI_Initialize_ABORT::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightAI_Initialize_ABORT(*this);
}

hsm::Transition AP_State_FlightAI_Initialize_ABORT::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {

        default:
            std::cout << "I dont know how we eneded up in this transition state from STATE_FLIGHT_GUIDED. STATE: " << MACEHSMStateToString(desiredStateEnum) << std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightAI_Initialize_ABORT::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {

    default:
        break;

    } //end of switch statement command type

    return success;
}

void AP_State_FlightAI_Initialize_ABORT::Update()
{

}

void AP_State_FlightAI_Initialize_ABORT::OnEnter()
{
    //This really shouldn't happen
    OnEnter("");
}

void AP_State_FlightAI_Initialize_ABORT::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    //This really shouldn't happen
    UNUSED(command);
    OnEnter("");
}

void AP_State_FlightAI_Initialize_ABORT::OnEnter(const std::string &descriptor)
{
    MAVLINKUXVControllers::LogDetails writeDetail;
    writeDetail.type = AI_EVENT_TYPE::AI_ABORTED_TEST;
    writeDetail.details = descriptor;

    MavlinkEntityKey sender = 255;
    static_cast<MAVLINKUXVControllers::Controller_WriteEventToLog*>(Owner().ControllersCollection()->At("onboardLoggingController"))->Broadcast(writeDetail, sender);
}


} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_guided_idle.h"
#include "plane_flight_states/state_flight_guided_spatial_item.h"
#include "plane_flight_states/state_flight_guided_queue.h"
#include "plane_flight_states/state_flight_guided_target_att.h"
#include "plane_flight_states/state_flight_guided_target_car.h"
#include "plane_flight_states/state_flight_guided_target_geo.h"

