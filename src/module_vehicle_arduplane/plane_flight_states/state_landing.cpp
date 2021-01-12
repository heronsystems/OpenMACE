#include "state_landing.h"

namespace ardupilot{
namespace state{

AP_State_Landing::AP_State_Landing():
    AbstractRootState(Data::MACEHSMState::STATE_LANDING)
{

}

AbstractStateArdupilot* AP_State_Landing::getClone() const
{
    return (new AP_State_Landing(*this));
}

void AP_State_Landing::getClone(AbstractStateArdupilot **state) const
{
    *state = new AP_State_Landing(*this);
}

hsm::Transition AP_State_Landing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {
        if(IsInInnerState<AP_State_LandingComplete>())
        {
            rtn = hsm::SiblingTransition<AP_State_Grounded>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (_desiredState) {
            case Data::MACEHSMState::STATE_LANDING_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<AP_State_LandingTransitioning>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_LANDING_DESCENDING:
            {
                rtn = hsm::InnerEntryTransition<AP_State_LandingDescent>(currentCommand);
                break;
            }
            case Data::MACEHSMState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<AP_State_Grounded>(currentCommand);
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

bool AP_State_Landing::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    this->clearCommand();

    switch(command->getCommandType())
    {
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
    case MAV_CMD::MAV_CMD_DO_SET_HOME:
    {
        success = AbstractRootState::handleCommand(command);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LAND:
    {
        this->currentCommand = command->getClone();
        //check that the vehicle is truely armed and switch us into the guided mode
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            controllerSystemMode->Shutdown();
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                _desiredState = Data::MACEHSMState::STATE_LANDING_TRANSITIONING;
            else
                _desiredState = Data::MACEHSMState::STATE_FLIGHT;
        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("AP_State_Landing_modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
        commandMode.vehicleMode = static_cast<uint8_t>(Owner().m_ArdupilotMode->getFlightModeFromString("GUIDED"));
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("AP_State_Landing_modeController",controllerSystemMode);

        success = true;

        break;
    }
    default:
        break;
    } //end of switch statement

    return success;
}

void AP_State_Landing::Update()
{
    StatusData_MAVLINK* vehicleStatus = Owner().status;

    if(!vehicleStatus->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED;
    else
    {
        if(vehicleStatus->vehicleMode.get().getFlightModeString() == "GUIDED")
            _desiredState = Data::MACEHSMState::STATE_LANDING_TRANSITIONING;
    }
}

//this function would be called when issuing a mode change
void AP_State_Landing::OnEnter()
{
    _desiredState = Data::MACEHSMState::STATE_LANDING_DESCENDING;
}

//this function is only called from the GUI
void AP_State_Landing::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        switch (command->getCommandType()) {
        case MAV_CMD::MAV_CMD_NAV_LAND:
            if((command->as<command_item::SpatialLand>()->getPosition() != nullptr)
                    && (command->as<command_item::SpatialLand>()->getPosition()->areTranslationalComponentsValid()))
            {
                handleCommand(command);
            }
            else
            {
                currentCommand = command->getClone();
                this->OnEnter();
            }
            break;
        default:
            break;
        }
    }
    else
    {
        this->OnEnter();
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "plane_flight_states/state_landing_descent.h"
#include "plane_flight_states/state_landing_transitioning.h"
#include "plane_flight_states/state_landing_complete.h"
#include "plane_flight_states/state_grounded.h"
#include "plane_flight_states/state_flight.h"
