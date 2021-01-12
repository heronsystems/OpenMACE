#include "state_flight.h"

namespace ardupilot{
namespace state{

State_Flight::State_Flight():
    AbstractRootState(Data::MACEHSMState::STATE_FLIGHT)
{

}

void State_Flight::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().status->vehicleMode.RemoveNotifier(this);
}

AbstractStateArdupilot* State_Flight::getClone() const
{
    return (new State_Flight(*this));
}

void State_Flight::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Flight(*this);
}

hsm::Transition State_Flight::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(_currentState != _desiredState)
    {

        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (_desiredState) {
        case Data::MACEHSMState::STATE_FLIGHT_AUTO:
        {
            rtn = hsm::InnerTransition<State_FlightAuto>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_BRAKE:
        {
            rtn = hsm::InnerTransition<State_FlightBrake>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_GUIDED:
        {
            rtn = hsm::InnerTransition<State_FlightGuided>();
            break;
        }
        case Data::MACEHSMState::STATE_LANDING:
        {
            rtn = hsm::SiblingTransition<State_Landing>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_LOITER:
        {
            rtn = hsm::InnerTransition<State_FlightLoiter>(currentCommand);
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_MANUAL:
        {
            rtn = hsm::InnerTransition<State_FlightManual>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_RTL:
        {
            rtn = hsm::InnerTransition<State_FlightRTL>();
            break;
        }
        case Data::MACEHSMState::STATE_GROUNDED:
        {
            rtn = hsm::SiblingTransition<State_Grounded>();
            break;
        }
        case Data::MACEHSMState::STATE_FLIGHT_UNKNOWN:
        {
            rtn = hsm::InnerTransition<State_FlightUnknown>();
            break;
        }
        default:
            std::cout<<"I dont know how we ended up in this transition state from STATE_FLIGHT."<<std::endl;
            break;
        }
    }
    return rtn;
}

bool State_Flight::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    MAV_CMD commandType = command->getCommandType();
    switch (commandType) {
    case MAV_CMD::MAV_CMD_DO_PAUSE_CONTINUE:
    {
        int vehicleMode = 0;
        bool executeModeChange = false;
        const command_item::ActionMissionCommand* cmd = command->as<command_item::ActionMissionCommand>();
        if(cmd->getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_PAUSE)
        {
            executeModeChange = true;
            if(Data::isSystemTypeRotary(Owner().status->vehicleHeartbeat.get().getType()))
            {
                vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("BRAKE");
            }
            else{
                vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("LOITER");
            }
        }
        else if(cmd->getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_START)
        {
            if(!this->IsInState<State_FlightGuided>())
            {
                executeModeChange = true;
                vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString("AUTO");
            }
        }

        if(executeModeChange)
        {
            Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
            auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
            controllerSystemMode->AddLambda_Finished(this, [controllerSystemMode](const bool completed, const uint8_t finishCode){
                UNUSED(completed); UNUSED(finishCode);
                controllerSystemMode->Shutdown();
            });

            controllerSystemMode->setLambda_Shutdown([collection]()
            {
                auto ptr = collection->Remove("modeController");
                delete ptr;
            });



            MavlinkEntityKey target = Owner().getMAVLINKID();
            MavlinkEntityKey sender = 255;

            MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
            commandMode.targetID = Owner().getMAVLINKID();
            commandMode.vehicleMode = vehicleMode;
            controllerSystemMode->Send(commandMode,sender,target);
            collection->Insert("modeController",controllerSystemMode);
            success = true;
        }
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }
        break;
    }
    case MAV_CMD::MAV_CMD_DO_SET_HOME:
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
    {
        success = AbstractRootState::handleCommand(command);
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_LAND:
    {
        currentCommand = command;
        _desiredState = Data::MACEHSMState::STATE_LANDING;
        success = true;
        break;
    }
    case MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH:
    {
        const command_item::SpatialRTL* cmd = command->as<command_item::SpatialRTL>();

        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerRTL = new MAVLINKUXVControllers::CommandRTL(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerRTL->AddLambda_Finished(this, [this,controllerRTL](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                _desiredState = Data::MACEHSMState::STATE_FLIGHT_RTL;
            controllerRTL->Shutdown();
        });

        controllerRTL->setLambda_Shutdown([collection]()
        {
            auto ptr = collection->Remove("RTLController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        controllerRTL->Send(*cmd,sender,target);
        collection->Insert("RTLController",controllerRTL);
        success = true;
        break;
    }
    case MAV_CMD::MAV_CMD_USER_1:
    {
        if(!this->IsInState<State_FlightGuided>())
            std::cout<<"We are currently not in a state of flight mode guided, and therefore the command cannot be accepted."<<std::endl;
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }

        break;
    }
    case MAV_CMD::MAV_CMD_USER_5:
    {
        if(!this->IsInState<State_FlightGuided>())
            std::cout<<"We are currently not in a state of flight mode guided, and therefore the command cannot be accepted."<<std::endl;
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            success = currentInnerState->handleCommand(command);
        }

        break;
    }
    default:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        success = currentInnerState->handleCommand(command);
        break;
    }
} //end of switch statement

    return success;
}

void State_Flight::Update()
{
    //mode changes are directly handled via add notifier events established in the OnEnter() method

    if(!Owner().status->vehicleArm.get().getSystemArm())
        _desiredState = Data::MACEHSMState::STATE_GROUNDED;
}

void State_Flight::OnEnter()
{
    //This will only handle when the mode has actually changed
    Owner().status->vehicleMode.AddNotifier(this,[this]
    {
        std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
        checkTransitionFromMode(currentModeString);
    });

    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().status->vehicleMode.get().getFlightModeString();
    checkTransitionFromMode(currentModeString);
}

void State_Flight::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    this->OnEnter();
    if(command != nullptr)
    {
        handleCommand(command);
    }
}

void State_Flight::checkTransitionFromMode(const std::string &mode)
{
    if(mode == "AUTO")
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_AUTO;
    }
    else if(mode == "BRAKE")
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_BRAKE;
    }
    else if(mode == "GUIDED")
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_GUIDED;
    }
    else if(mode == "LAND")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we are
        _desiredState = Data::MACEHSMState::STATE_LANDING;
    }
    else if(mode == "LOITER")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we are
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_LOITER;
    }
    else if(mode == "STABILIZE")
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_MANUAL;
    }
    else if(mode == "RTL")
    {
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_RTL;
    }
    else{
        _desiredState = Data::MACEHSMState::STATE_FLIGHT_UNKNOWN;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "flight_states/state_flight_auto.h"
#include "flight_states/state_flight_brake.h"
#include "flight_states/state_flight_guided.h"
#include "flight_states/state_flight_land.h"
#include "flight_states/state_flight_loiter.h"
#include "flight_states/state_flight_manual.h"
#include "flight_states/state_flight_rtl.h"
#include "flight_states/state_flight_unknown.h"

#include "flight_states/state_landing.h"
#include "flight_states/state_grounded.h"



