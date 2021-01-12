#include "abstract_root_state.h"

namespace ardupilot {
namespace state {
AbstractRootState::AbstractRootState():
    AbstractStateArdupilot()
{

}

AbstractRootState::AbstractRootState(const Data::MACEHSMState &enteredState):
    AbstractStateArdupilot(enteredState)
{

}

AbstractRootState::AbstractRootState(const AbstractRootState &copy):
    AbstractStateArdupilot(copy)
{

}

bool AbstractRootState::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    bool success = false;
    switch (command->getCommandType()) {
    case MAV_CMD::MAV_CMD_DO_SET_MODE:
    {

        MAVLINKUXVControllers::ControllerSystemMode* modeController = AbstractStateArdupilot::prepareModeController();

        modeController->AddLambda_Finished(this, [this, modeController](const bool completed, const uint8_t finishCode){
            UNUSED(this); UNUSED(completed); UNUSED(finishCode);
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

    case MAV_CMD::MAV_CMD_DO_SET_HOME:
    {
        command_item::SpatialHome* cmd = new command_item::SpatialHome(*command->as<command_item::SpatialHome>());

        cmd->setTargetSystem(Owner().getMAVLINKID());

        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemHome = new MAVLINKUXVControllers::Command_HomePositionSet(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemHome->AddLambda_Finished(this, [this,controllerSystemHome,cmd](const bool completed, const uint8_t finishCode){
            UNUSED(this); UNUSED(cmd);
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {

            }
            controllerSystemHome->Shutdown();
        });

        controllerSystemHome->setLambda_Shutdown([this, collection]()
        {
            UNUSED(this);
            auto ptr = collection->Remove("setHomeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;
        controllerSystemHome->Send(*cmd,sender,target);
        collection->Insert("setHomeController",controllerSystemHome);
        success = true;
        break;
    }
    default:
        success = AbstractStateArdupilot::handleCommand(command);
        break;
    }

    return success;
}

} //end of namespace state
} //end of namespace ardupilot
