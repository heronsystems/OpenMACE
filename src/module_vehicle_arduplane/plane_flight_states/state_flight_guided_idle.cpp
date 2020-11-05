#include "state_flight_guided_idle.h"

namespace ardupilot {
namespace state{

AP_State_FlightGuided_Idle::AP_State_FlightGuided_Idle():
    AbstractStateArdupilot()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_IDLE"<<std::endl;
    currentStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
    desiredStateEnum = Data::MACEHSMState::STATE_FLIGHT_GUIDED_IDLE;
}

void AP_State_FlightGuided_Idle::OnExit()
{

}

AbstractStateArdupilot* AP_State_FlightGuided_Idle::getClone() const
{
    return (new AP_State_FlightGuided_Idle(*this));
}

void AP_State_FlightGuided_Idle::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_FlightGuided_Idle(*this);
}

hsm::Transition AP_State_FlightGuided_Idle::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {

        default:
            std::cout<<"I dont know how we eneded up in this transition state from AP_State_FlightGuided_Idle."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_FlightGuided_Idle::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {

    default:
        break;
    }

    return false;
}

void AP_State_FlightGuided_Idle::Update()
{

}

void AP_State_FlightGuided_Idle::OnEnter()
{

}

void AP_State_FlightGuided_Idle::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_flight_guided_spatial_item.h"
#include "plane_flight_states/state_flight_guided_queue.h"
#include "plane_flight_states/state_flight_guided_target_car.h"
#include "plane_flight_states/state_flight_guided_target_geo.h"
