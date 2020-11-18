#include "abstract_state_ardupilot.h"

namespace ardupilot{
namespace state{

AbstractStateArdupilot::AbstractStateArdupilot() :
    currentCommand(nullptr),
    currentCommandSet(false)

{

}

AbstractStateArdupilot::AbstractStateArdupilot(const Data::MACEHSMState &enteredState):
    currentCommand(nullptr),
    currentCommandSet(false)

{
    setCurrentStateEnum(enteredState);
    setDesiredStateEnum(enteredState);
    updateOwner_ProgressionOfHSM();
}

AbstractStateArdupilot::AbstractStateArdupilot(const AbstractStateArdupilot &copy)
{

    if(copy.currentCommandSet == true)
    {
        this->currentCommand = copy.currentCommand->getClone();
        this->currentCommandSet = true;
    }
    else {
        this->currentCommandSet = false;
    }

    currentStateEnum = copy.currentStateEnum;
    desiredStateEnum = copy.desiredStateEnum;
}

AbstractStateArdupilot::~AbstractStateArdupilot()
{
    clearCommand();
}

void AbstractStateArdupilot::OnExit()
{
    Owner().ControllersCollection()->ForAll([this](Controllers::IController<mavlink_message_t, int>* controller){
        controller->RemoveHost(this);
    });

    clearCommand();
}

void AbstractStateArdupilot::clearCommand()
{
    currentCommandSet = false;
}

void AbstractStateArdupilot::setCurrentCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    clearCommand();
    this->currentCommand = command;
}

bool AbstractStateArdupilot::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool commandHandled = false;
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    {
        Controllers::ControllerCollection<mavlink_message_t, int> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this, controllerSystemMode](const bool completed, const uint8_t finishCode){
            UNUSED(completed); UNUSED(finishCode); UNUSED(this);
            controllerSystemMode->Shutdown();
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
        commandMode.vehicleMode = Owner().m_ArdupilotMode->getFlightModeFromString(command->as<command_item::ActionChangeMode>()->getRequestMode());
        controllerSystemMode->Send(commandMode, sender, target);

        collection->Insert("modeController", controllerSystemMode);
        commandHandled = true;
        break;
    }
    default:
        break;
    }

    return commandHandled;
}

bool AbstractStateArdupilot::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    UNUSED(msg);
    throw std::runtime_error("States should no longer handle mavlink messages");
    return false;
}

void AbstractStateArdupilot::updateOwner_ProgressionOfHSM()
{
    Owner()._currentHSMState.set(currentStateEnum);
}


} //end of namespace state
} //end of namespace ardupilot
