#include "state_landing.h"

namespace ardupilot{
namespace state{

State_Landing::State_Landing():
    AbstractRootState()
{
    std::cout<<"We are in the constructor of STATE_LANDING"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_LANDING;
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING;
}

AbstractStateArdupilot* State_Landing::getClone() const
{
    return (new State_Landing(*this));
}

void State_Landing::getClone(AbstractStateArdupilot** state) const
{
    *state = new State_Landing(*this);
}

hsm::Transition State_Landing::GetTransition()
{
    hsm::Transition rtn = hsm::NoTransition();

    if(currentStateEnum != desiredStateEnum)
    {
        if(IsInInnerState<State_LandingComplete>())
        {
            rtn = hsm::SiblingTransition<State_Grounded>();
        }
        else
        {
            //this means we want to chage the state of the vehicle for some reason
            //this could be caused by a command, action sensed by the vehicle, or
            //for various other peripheral reasons
            switch (desiredStateEnum) {
            case ArdupilotFlightState::STATE_LANDING_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<State_LandingTransitioning>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_LANDING_DESCENDING:
            {
                rtn = hsm::InnerEntryTransition<State_LandingDescent>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<State_Grounded>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_FLIGHT:
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

bool State_Landing::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    this->clearCommand();

    switch(command->getCommandType())
    {
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    case COMMANDTYPE::CI_NAV_HOME:
    {
        AbstractRootState::handleCommand(command);
        break;
    }
    case COMMANDTYPE::CI_NAV_LAND:
    {
        this->currentCommand = command->getClone();
        //check that the vehicle is truely armed and switch us into the guided mode
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
            controllerSystemMode->Shutdown();
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
                desiredStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
            else
                desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = static_cast<uint8_t>(Owner().getMAVLINKID());
        commandMode.vehicleMode = static_cast<uint8_t>(Owner().ardupilotMode.getFlightModeFromString("GUIDED"));
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("modeController",controllerSystemMode);

        break;
    }
    default:
        break;
    } //end of switch statement
}

void State_Landing::Update()
{
    StateData_MAVLINK* vehicleData = Owner().state;

    if(!vehicleData->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
    else
    {
        if(vehicleData->vehicleMode.get().getFlightModeString() == "GUIDED")
            desiredStateEnum = ArdupilotFlightState::STATE_LANDING_TRANSITIONING;
    }
}

//this function would be called when issuing a mode change
void State_Landing::OnEnter()
{
    desiredStateEnum = ArdupilotFlightState::STATE_LANDING_DESCENDING;
}

//this function is only called from the GUI
void State_Landing::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        switch (command->getCommandType()) {
        case COMMANDTYPE::CI_NAV_LAND:
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

#include "ardupilot_states/state_landing_descent.h"
#include "ardupilot_states/state_landing_transitioning.h"
#include "ardupilot_states/state_landing_complete.h"
#include "ardupilot_states/state_grounded.h"
#include "ardupilot_states/state_flight.h"
