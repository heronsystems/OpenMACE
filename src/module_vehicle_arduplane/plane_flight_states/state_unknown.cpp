#include "state_unknown.h"

namespace ardupilot {
namespace state{

AP_State_Unknown::AP_State_Unknown():
    AbstractStateArdupilot(Data::MACEHSMState::STATE_UNKNOWN)
{

}

AbstractStateArdupilot* AP_State_Unknown::getClone() const
{
    return (new AP_State_Unknown(*this));
}

void AP_State_Unknown::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_Unknown(*this);
}

hsm::Transition AP_State_Unknown::GetTransition()
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
            return hsm::SiblingTransition<AP_State_Grounded>();
            break;
        }
        case Data::MACEHSMState::STATE_TAKEOFF:
        {
            return hsm::SiblingTransition<AP_State_Takeoff>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT:
        {
            return hsm::SiblingTransition<AP_State_Flight>();
            break;
        }
        case Data::MACEHSMState::STATE_LANDING:
        {
            return hsm::SiblingTransition<AP_State_Landing>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_UNKNOWN."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool AP_State_Unknown::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    return false;
}

void AP_State_Unknown::Update()
{
    if(!Owner().status->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED; //This is a definite case condition
    else
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT;
    }
}

void AP_State_Unknown::OnEnter()
{
}

void AP_State_Unknown::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    UNUSED(command);
    this->OnEnter();
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded.h"
#include "plane_flight_states/state_takeoff.h"
#include "plane_flight_states/state_flight.h"
#include "plane_flight_states/state_landing.h"
