#include "state_takeoff.h"

namespace ardupilot {
namespace state{

State_Takeoff::State_Takeoff():
    AbstractRootState(Data::MACEHSMState::STATE_TAKEOFF)
{

}

AbstractStateArdupilot* State_Takeoff::getClone() const
{
    return (new State_Takeoff(*this));
}

void State_Takeoff::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Takeoff(*this);
}

hsm::Transition State_Takeoff::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        if(IsInInnerState<State_TakeoffComplete>())
        {
            rtn = hsm::SiblingTransition<State_Flight>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (_desiredState) {
            case Data::MACEHSMState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<State_Grounded>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_TAKEOFF_CLIMBING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffClimbing>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_TAKEOFF_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffTransitioning>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_FLIGHT:
            {
                rtn = hsm::SiblingTransition<State_Flight>(currentCommand);
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

bool State_Takeoff::handleCommand(const std::shared_ptr<command_item::AbstractCommandItem> command)
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
        success = AbstractStateArdupilot::handleCommand(command);
        MAVLINKUXVControllers::VehicleController::ControllerSystemMode* modeController = (MAVLINKUXVControllers::VehicleController::ControllerSystemMode*)Owner().ControllersCollection()->At("modeController");
        modeController->AddLambda_Finished(this, [this,modeController](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
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

void State_Takeoff::Update()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(!vehicleStatus->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED;
    else
    {
        if(vehicleStatus->vehicleMode.get().getFlightModeString() == "GUIDED")
            _desiredState = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
    }
}

void State_Takeoff::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerSystemMode = new MAVLINKUXVControllers::VehicleController::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        controllerSystemMode->Shutdown();
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            _desiredState = Data::MACEHSMState::STATE_TAKEOFF_CLIMBING;
        else
            _desiredState = Data::MACEHSMState::STATE_GROUNDED;
    });

    controllerSystemMode->setLambda_Shutdown([this, collection]()
    {
        UNUSED(this);
        auto ptr = collection->Remove("modeController");
        delete ptr;
    });

    MavlinkEntityKey target = Owner().getMAVLINKID();
    MavlinkEntityKey sender = 255;

    MAVLINKUXVControllers::VehicleController::VehicleMode_Struct commandMode;
    commandMode.targetID = Owner().getMAVLINKID();
    commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("GUIDED");
    controllerSystemMode->Send(commandMode,sender,target);
    collection->Insert("modeController",controllerSystemMode);
}



void State_Takeoff::OnEnter(const std::shared_ptr<command_item::AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        this->OnEnter();
        this->handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_grounded.h"
#include "flight_states/state_takeoff_climbing.h"
#include "flight_states/state_takeoff_transitioning.h"
#include "flight_states/state_takeoff_complete.h"
#include "flight_states/state_flight.h"
