#include "state_flight.h"

namespace ardupilot{
namespace state{

State_Flight::State_Flight():
    AbstractRootState()
{
    std::cout<<"We are in the constructor of STATE_FLIGHT"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_FLIGHT;
    desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
}

void State_Flight::OnExit()
{
    AbstractStateArdupilot::OnExit();
    Owner().state->vehicleMode.RemoveNotifier(this);
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

    if(currentStateEnum != desiredStateEnum)
    {

        //this means we want to chage the state of the vehicle for some reason
        //this could be caused by a command, action sensed by the vehicle, or
        //for various other peripheral reasons
        switch (desiredStateEnum) {
        case ArdupilotFlightState::STATE_FLIGHT_AUTO:
        {
            rtn = hsm::InnerTransition<State_FlightAuto>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_BRAKE:
        {
            rtn = hsm::InnerTransition<State_FlightBrake>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_GUIDED:
        {
            rtn = hsm::InnerTransition<State_FlightGuided>();
            break;
        }
        case ArdupilotFlightState::STATE_LANDING:
        {
            rtn = hsm::SiblingTransition<State_Landing>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_LOITER:
        {
            rtn = hsm::InnerTransition<State_FlightLoiter>(currentCommand);
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_MANUAL:
        {
            rtn = hsm::InnerTransition<State_FlightManual>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_RTL:
        {
            rtn = hsm::InnerTransition<State_FlightRTL>();
            break;
        }
        case ArdupilotFlightState::STATE_GROUNDED:
        {
            rtn = hsm::SiblingTransition<State_Grounded>();
            break;
        }
        case ArdupilotFlightState::STATE_FLIGHT_UNKNOWN:
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
    COMMANDTYPE commandType = command->getCommandType();
    switch (commandType) {
    case COMMANDTYPE::CI_ACT_MISSIONCMD:
    {
        int vehicleMode = 0;
        bool executeModeChange = false;
        const command_item::ActionMissionCommand* cmd = command->as<command_item::ActionMissionCommand>();
        if(cmd->getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_PAUSE)
        {
            executeModeChange = true;
            if(Data::isSystemTypeRotary(Owner().state->vehicleHeartbeat.get().getType()))
            {
                vehicleMode = Owner().ardupilotMode.getFlightModeFromString("BRAKE");
            }
            else{
                vehicleMode = Owner().ardupilotMode.getFlightModeFromString("LOITER");
            }
        }
        else if(cmd->getMissionCommandAction() == Data::MissionCommandAction::MISSIONCA_START)
        {
            if(!this->IsInState<State_FlightGuided>())
            {
                executeModeChange = true;
                vehicleMode = Owner().ardupilotMode.getFlightModeFromString("AUTO");
            }
        }

        if(executeModeChange)
        {
            Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
            auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
            controllerSystemMode->AddLambda_Finished(this, [controllerSystemMode](const bool completed, const uint8_t finishCode){

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
        }
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            currentInnerState->handleCommand(command);
        }
        break;
    }
    case COMMANDTYPE::CI_NAV_HOME:
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    {
        AbstractRootState::handleCommand(command);
        break;
    }
    case COMMANDTYPE::CI_NAV_LAND:
    {
        currentCommand = command;
        desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
        break;
    }
    case COMMANDTYPE::CI_NAV_RETURN_TO_LAUNCH:
    {
        const command_item::SpatialRTL* cmd = command->as<command_item::SpatialRTL>();

        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerRTL = new MAVLINKUXVControllers::CommandRTL(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerRTL->AddLambda_Finished(this, [this,controllerRTL](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_RTL;
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
        break;
    }
    case COMMANDTYPE::CI_ACT_EXECUTE_SPATIAL_ITEM:
    {
        if(!this->IsInState<State_FlightGuided>())
            std::cout<<"We are currently not in a state of flight mode guided, and therefore the command cannot be accepted."<<std::endl;
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            currentInnerState->handleCommand(command);
        }

        break;
    }
    case COMMANDTYPE::CI_ACT_TARGET:
    {
        if(!this->IsInState<State_FlightGuided>())
            std::cout<<"We are currently not in a state of flight mode guided, and therefore the command cannot be accepted."<<std::endl;
        else
        {
            ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
            currentInnerState->handleCommand(command);
        }

        break;
    }
    default:
    {
        ardupilot::state::AbstractStateArdupilot* currentInnerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(GetImmediateInnerState());
        currentInnerState->handleCommand(command);
        break;
    }
} //end of switch statement
}

void State_Flight::Update()
{
    //mode changes are directly handled via add notifier events established in the OnEnter() method

    if(!Owner().state->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
}

void State_Flight::OnEnter()
{
    //This will only handle when the mode has actually changed
    Owner().state->vehicleMode.AddNotifier(this,[this]
    {
        std::string currentModeString = Owner().state->vehicleMode.get().getFlightModeString();
        checkTransitionFromMode(currentModeString);
    });

    //This helps us based on the current conditions in the present moment
    std::string currentModeString = Owner().state->vehicleMode.get().getFlightModeString();
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
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_AUTO;
    }
    else if(mode == "BRAKE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_BRAKE;
    }
    else if(mode == "GUIDED")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_GUIDED;
    }
    else if(mode == "LAND")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we are
        desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
    }
    else if(mode == "LOITER")
    {
        //This event is handled differently than the land command issued from the GUI
        //A mode change we really have no way to track the progress of where we are
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_LOITER;
    }
    else if(mode == "STABILIZE")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_MANUAL;
    }
    else if(mode == "RTL")
    {
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_RTL;
    }
    else{
        desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT_UNKNOWN;
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_flight_auto.h"
#include "ardupilot_states/state_flight_brake.h"
#include "ardupilot_states/state_flight_guided.h"
#include "ardupilot_states/state_flight_land.h"
#include "ardupilot_states/state_flight_loiter.h"
#include "ardupilot_states/state_flight_manual.h"
#include "ardupilot_states/state_flight_rtl.h"
#include "ardupilot_states/state_flight_unknown.h"

#include "ardupilot_states/state_landing.h"
#include "ardupilot_states/state_grounded.h"



