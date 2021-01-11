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
    MaceLog::Alert("[ARDUPILOT] Entered state: " + MACEHSMStateToString(enteredState));
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
    _currentState = copy._currentState;
    _desiredState = copy._desiredState;
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
    this->currentCommand = command->getClone();
}

bool AbstractStateArdupilot::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool commandHandled = false;
    switch (command->getCommandType()) {
    default:
        break;
    }

    return commandHandled;
}

bool AbstractStateArdupilot::handleMAVLINKMessage(const mavlink_message_t &msg)
{
    UNUSED(msg);
    throw std::runtime_error("States should no longer handle mavlink messages");

    /*
    int systemID = msg.sysid;

    MaceCore::ModuleCharacteristic sender;
    sender.ModuleID = systemID;
    sender.Class = MaceCore::ModuleClasses::VEHICLE_COMMS;

    bool consumed = false;
    std::unordered_map<std::string, Controllers::IController<mavlink_message_t>*>::iterator it;

    m_ControllerFactory->controllerMutex.lock();
    for(it=m_ControllerFactory->controllers.begin(); it!=m_ControllerFactory->controllers.end(); ++it)
    {
        Controllers::IController<mavlink_message_t>* obj = it->second;
        consumed = obj->ReceiveMessage(&msg, sender);
    }
    m_ControllerFactory->controllerMutex.unlock();


    if(!consumed)
    {
        State* innerState = GetImmediateInnerState();
        if(innerState == this)
        {
            printf("!!!!!! WARNING: Immediate Inner State is equal to the outer state. This is a non-op that will result in infinte recursion. Ignoring but it probably points to a larger bug\n");
            return consumed;
        }

        if(innerState != nullptr)
        {
            ardupilot::state::AbstractStateArdupilot* castChild = static_cast<ardupilot::state::AbstractStateArdupilot*>(innerState);
            consumed = castChild->handleMAVLINKMessage(msg);
        }

    }
    return consumed;
    */
    return false;
}

void AbstractStateArdupilot::updateOwner_ProgressionOfHSM()
{
//    Owner()._currentHSMState.set(currentStateEnum);
}

MAVLINKUXVControllers::ControllerSystemMode* AbstractStateArdupilot::prepareModeController()
{
    Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();

    //If there is an old mode controller still running, allow us to shut it down
    if(collection->Exist("modeController"))
    {
        auto modeControllerOld = static_cast<MAVLINKUXVControllers::ControllerSystemMode*>(collection->At("modeController"));
        if(modeControllerOld != nullptr)
        {
            std::cout << "Shutting down previous mode controller, which was still active" << std::endl;
            modeControllerOld->Shutdown();
            // Need to wait for the old controller to be shutdown.
            std::unique_lock<std::mutex> controllerLock(m_mutex_ModeController);
            while (!m_oldModeControllerShutdown)
                m_condition_ModeController.wait(controllerLock);
            m_oldModeControllerShutdown = false;
            controllerLock.unlock();
        }
    }
    //create "stateless" mode controller that exists within the module itself
    MAVLINKUXVControllers::ControllerSystemMode* modeController = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());

//    modeController->AddLambda_Finished(this, [this, modeController](const bool completed, const uint8_t finishCode){
//        UNUSED(this); UNUSED(completed); UNUSED(finishCode);
//        modeController->Shutdown();
//    });

    modeController->setLambda_Shutdown([this, modeController]() mutable
    {
        auto ptr = static_cast<MAVLINKUXVControllers::ControllerSystemMode*>(Owner().ControllersCollection()->At("modeController"));
        if (ptr != nullptr)
        {
            auto ptr = Owner().ControllersCollection()->Remove("modeController");
            delete ptr;
        }
        std::lock_guard<std::mutex> guard(m_mutex_ModeController);
        m_oldModeControllerShutdown = true;
        m_condition_ModeController.notify_one();
    });
    collection->Insert("modeController", modeController);

    return modeController;
}

} //end of namespace state
} //end of namespace ardupilot
