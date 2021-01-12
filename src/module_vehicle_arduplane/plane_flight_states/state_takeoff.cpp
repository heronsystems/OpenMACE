#include "state_takeoff.h"

namespace ardupilot {
namespace state{

AP_State_Takeoff::AP_State_Takeoff():
    AbstractRootState(Data::MACEHSMState::STATE_TAKEOFF)
{

}

AbstractStateArdupilot* AP_State_Takeoff::getClone() const
{
    return (new AP_State_Takeoff(*this));
}

void AP_State_Takeoff::getClone(AbstractStateArdupilot** state) const
{
    *state = new AP_State_Takeoff(*this);
}

hsm::Transition AP_State_Takeoff::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        if(IsInInnerState<AP_State_TakeoffComplete>())
        {
            rtn = hsm::SiblingTransition<AP_State_Flight>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (_desiredState) {
            case Data::MACEHSMState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<AP_State_Grounded>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_TAKEOFF_CLIMBING:
            {
                rtn = hsm::InnerEntryTransition<AP_State_TakeoffClimbing>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<AP_State_TakeoffTransitioning>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_FLIGHT:
            {
                rtn = hsm::SiblingTransition<AP_State_Flight>(currentCommand);
                break;
            }
            default:
                std::cout<<"I dont know how we ended up in this transition state from STATE_TAKEOFF."<<std::endl;
                break;
            }
        }
    }
    return rtn;
}

bool AP_State_Takeoff::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    this->clearCommand();

    switch(command->getCommandType()) {
    case MAV_CMD::MAV_CMD_DO_SET_HOME:
    {
        success = AbstractRootState::handleCommand(command);
        break;
    }
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
    {
        MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();

        modeController->AddLambda_Finished(this, [this,modeController](const bool completed, const uint8_t finishCode){
            if((completed) && (finishCode == MAV_RESULT_ACCEPTED))
            {
                //if a mode change was issued while in the takeoff sequence we may have to handle it in a specific way based on the conditions
                _desiredState = Data::MACEHSMState::STATE_FLIGHT;
            }
            else
            {
                //we got issues?
            }
            modeController->Shutdown();
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = Owner().getMAVLINKID();
        commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString(command->as<command_item::ActionChangeMode>()->getRequestMode());
        modeController->Send(commandMode,sender,target);
        success = true;
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_TAKEOFF:
    {
        currentCommand = command->getClone();
        success = true;
        break;
    }
    default:
        break;
    } //end of switch statement

    return success;
}

void AP_State_Takeoff::Update()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(!vehicleStatus->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED;
    else
    {
        if(vehicleStatus->vehicleMode.get().getFlightModeString() == "TAKEOFF")
            _desiredState = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
    }
}

void AP_State_Takeoff::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode

    MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();

    modeController->AddLambda_Finished(this, [this,modeController](const bool completed, const uint8_t finishCode){
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            _desiredState = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
        else
            _desiredState = Data::MACEHSMState::STATE_GROUNDED;

        modeController->Shutdown();
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
    commandMode.targetID = Owner().getMAVLINKID();
    commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("TAKEOFF");
    modeController->Send(commandMode,sender,target);
}



void AP_State_Takeoff::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        this->OnEnter();
        this->handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_grounded.h"
#include "plane_flight_states/state_takeoff_climbing.h"
#include "plane_flight_states/state_takeoff_transitioning.h"
#include "plane_flight_states/state_takeoff_complete.h"
#include "plane_flight_states/state_flight.h"
