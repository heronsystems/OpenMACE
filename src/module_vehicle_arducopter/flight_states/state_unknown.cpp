#include "state_unknown.h"

namespace ardupilot {
namespace state{

State_Unknown::State_Unknown():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_UNKNOWN)
{

}

AbstractStateArdupilot* State_Unknown::getClone() const
{
    return (new State_Unknown(*this));
}

void State_Unknown::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Unknown(*this);
}

hsm::Transition State_Unknown::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_GROUNDED:
        {
            return hsm::SiblingTransition<State_Grounded>();
            break;
        }
        case Data::MACEHSMState::STATE_TAKEOFF:
        {
            return hsm::SiblingTransition<State_Takeoff>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT:
        {
            return hsm::SiblingTransition<State_Flight>();
            break;
        }
        case Data::MACEHSMState::STATE_LANDING:
        {
            return hsm::SiblingTransition<State_Landing>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_UNKNOWN."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_Unknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void State_Unknown::Update()
{
    if(!Owner().status->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED; //This is a definite case condition
    else
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT;
    }
}

void State_Unknown::OnEnter()
{
}

void State_Unknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_grounded.h"
#include "flight_states/state_takeoff.h"
#include "flight_states/state_flight.h"
#include "flight_states/state_landing.h"
