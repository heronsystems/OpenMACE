#include "state_flight_guided_idle.h"

namespace arducopter{
namespace state{

State_FlightGuided_Idle::State_FlightGuided_Idle():
    AbstractStateArducopter()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT_GUIDED_IDLE"<<std::endl;
    currentStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_IDLE;
    desiredStateEnum = ArducopterFlightState::STATE_FLIGHT_GUIDED_IDLE;
}

void State_FlightGuided_Idle::OnExit()
{

}

AbstractStateArducopter* State_FlightGuided_Idle::getClone() const
{
    return (new State_FlightGuided_Idle(*this));
}

void State_FlightGuided_Idle::getClone(AbstractStateArducopter** state) const
{
    *state = new State_FlightGuided_Idle(*this);
}

hsm::Transition State_FlightGuided_Idle::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {

        default:
            std::cout<<"I dont know how we eneded up in this transition state from State_FlightGuided_Idle."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_FlightGuided_Idle::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {

    default:
        break;
    }
}

void State_FlightGuided_Idle::Update()
{

}

void State_FlightGuided_Idle::OnEnter()
{

}

void State_FlightGuided_Idle::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
}

} //end of namespace arducopter
} //end of namespace state

#include "flight_states/state_flight_guided_spatial_item.h"
#include "flight_states/state_flight_guided_queue.h"
#include "flight_states/state_flight_guided_target_car.h"
#include "flight_states/state_flight_guided_target_geo.h"
