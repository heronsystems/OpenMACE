#include "state_takeoff.h"

namespace ardupilot{
namespace state{

State_Takeoff::State_Takeoff():
    AbstractRootState()
{
    std::cout<<"We are in the constructor of STATE_TAKEOFF"<<std::endl;
    currentStateEnum = ArdupilotFlightState::STATE_TAKEOFF;
    desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF;
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

    if(currentStateEnum != desiredStateEnum)
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
            switch (desiredStateEnum) {
            case ArdupilotFlightState::STATE_GROUNDED:
            {
                rtn = hsm::SiblingTransition<State_Grounded>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_TAKEOFF_CLIMBING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffClimbing>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_TAKEOFF_TRANSITIONING:
            {
                rtn = hsm::InnerEntryTransition<State_TakeoffTransitioning>(currentCommand);
                break;
            }
            case ArdupilotFlightState::STATE_FLIGHT:
            {
                rtn = hsm::SiblingTransition<State_Flight>(currentCommand);
            }
            default:
                std::cout<<"I dont know how we ended up in this transition state from STATE_TAKEOFF."<<std::endl;
                break;
            }
        }
    }
    return rtn;
}

bool State_Takeoff::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    this->clearCommand();

    switch(command->getCommandType()) {
    case COMMANDTYPE::CI_NAV_HOME:
    {
        AbstractRootState::handleCommand(command);
        break;
    }
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    {
        AbstractStateArdupilot::handleCommand(command);
        MAVLINKUXVControllers::ControllerSystemMode* modeController = (MAVLINKUXVControllers::ControllerSystemMode*)Owner().ControllersCollection()->At("modeController");
        modeController->AddLambda_Finished(this, [this,modeController](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {
                //if a mode change was issued while in the takeoff sequence we may have to handle it in a specific way based on the conditions
                desiredStateEnum = ArdupilotFlightState::STATE_FLIGHT;
            }
            else
            {
                //we got issues?
            }
            modeController->Shutdown();
        });
        break;
    }
    case COMMANDTYPE::CI_NAV_TAKEOFF:
    {
        currentCommand = command->getClone();
        break;
    }
    default:
        break;
    } //end of switch statement
}

void State_Takeoff::Update()
{
    StateData_MAVLINK* vehicleData = Owner().state;

    if(!vehicleData->vehicleArm.get().getSystemArm())
        desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
    else
    {
        if(vehicleData->vehicleMode.get().getFlightModeString() == "GUIDED")
            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
    }
}

void State_Takeoff::OnEnter()
{
    //check that the vehicle is truely armed and switch us into the guided mode
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
    auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
    controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){
        controllerSystemMode->Shutdown();
        if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            desiredStateEnum = ArdupilotFlightState::STATE_TAKEOFF_CLIMBING;
        else
            desiredStateEnum = ArdupilotFlightState::STATE_GROUNDED;
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
    commandMode.targetID = Owner().getMAVLINKID();
    commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString("GUIDED");
    controllerSystemMode->Send(commandMode,sender,target);
    collection->Insert("modeController",controllerSystemMode);
}



void State_Takeoff::OnEnter(const std::shared_ptr<AbstractCommandItem> command)
{
    if(command != nullptr)
    {
        this->OnEnter();
        this->handleCommand(command);
    }
}

} //end of namespace ardupilot
} //end of namespace state

#include "ardupilot_states/state_grounded.h"
#include "ardupilot_states/state_takeoff_climbing.h"
#include "ardupilot_states/state_takeoff_transitioning.h"
#include "ardupilot_states/state_takeoff_complete.h"
#include "ardupilot_states/state_flight.h"
