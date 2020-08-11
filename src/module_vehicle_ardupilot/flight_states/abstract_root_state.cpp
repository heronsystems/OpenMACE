#include "abstract_root_state.h"

namespace ardupilot {
namespace state {
AbstractRootState::AbstractRootState():
    AbstractStateArdupilot()
{

}

AbstractRootState::AbstractRootState(const AbstractRootState &copy):
    AbstractStateArdupilot(copy)
{

}

bool AbstractRootState::handleCommand(const std::shared_ptr<AbstractCommandItem> command)
{
    switch (command->getCommandType()) {
    case COMMANDTYPE::CI_ACT_CHANGEMODE:
    {
        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemMode = new MAVLINKUXVControllers::ControllerSystemMode(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemMode->AddLambda_Finished(this, [this,controllerSystemMode](const bool completed, const uint8_t finishCode){

            controllerSystemMode->Shutdown();
        });

        controllerSystemMode->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("modeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;

        MAVLINKUXVControllers::MAVLINKModeStruct commandMode;
        commandMode.targetID = Owner().getMAVLINKID();
        commandMode.vehicleMode = Owner().ardupilotMode.getFlightModeFromString(command->as<command_item::ActionChangeMode>()->getRequestMode());
        controllerSystemMode->Send(commandMode,sender,target);
        collection->Insert("modeController",controllerSystemMode);
        break;
    }

    case COMMANDTYPE::CI_NAV_HOME:
    {
        command_item::SpatialHome* cmd = new command_item::SpatialHome(*command->as<command_item::SpatialHome>());

        cmd->setTargetSystem(Owner().getMAVLINKID());

        Controllers::ControllerCollection<mavlink_message_t, MavlinkEntityKey> *collection = Owner().ControllersCollection();
        auto controllerSystemHome = new MAVLINKUXVControllers::Command_HomePositionSet(&Owner(), Owner().GetControllerQueue(), Owner().getCommsObject()->getLinkChannel());
        controllerSystemHome->AddLambda_Finished(this, [this,controllerSystemHome,cmd](const bool completed, const uint8_t finishCode){
            if(completed && (finishCode == MAV_RESULT_ACCEPTED))
            {

            }
            controllerSystemHome->Shutdown();
        });

        controllerSystemHome->setLambda_Shutdown([this, collection]()
        {
            auto ptr = collection->Remove("setHomeController");
            delete ptr;
        });

        MavlinkEntityKey target = Owner().getMAVLINKID();
        MavlinkEntityKey sender = 255;
        controllerSystemHome->Send(*cmd,sender,target);
        collection->Insert("setHomeController",controllerSystemHome);
        break;
    }
    default:
        break;
    }
}

} //end of namespace state
} //end of namespace ardupilot
