#ifndef MAVLINK_CONTROLLER_COMMAND_ARM_H
#define MAVLINK_CONTROLLER_COMMAND_ARM_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers {

namespace VehicleController {

class CommandARM : public Controller_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>
{
public:

    //!
    //! \brief CommandTakeoff This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandARM(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>(cb, queue, linkChan, "CommandArm")
    {

    }

    virtual ~CommandARM() = default;

    virtual void FillCommand(const command_item::ActionArm &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());
        cmd.param1 = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::ActionArm &data) const
    {
        data.setTargetSystem(message.target_system);
        data.setVehicleArm(message.param1);
    }

};

} //end of namespace VehicleController

namespace ModuleController {

class CommandARM : public ModuleController_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>
{
public:

    //!
    //! \brief CommandTakeoff::CommandTakeoff This constructor is used to communicate between MACE instances
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandARM(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        ModuleController_GenericLongCommand<command_item::ActionArm, MAV_CMD_COMPONENT_ARM_DISARM>(cb, queue, linkChan, "CommandArm")
    {

    }

    virtual ~CommandARM() = default;

    virtual void FillCommand(const command_item::ActionArm &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = static_cast<uint8_t>(commandItem.getTargetSystem());
        cmd.param1 = commandItem.getRequestArm();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::ActionArm &data) const
    {
        data.setTargetSystem(message.target_system);
        data.setVehicleArm(message.param1);
    }
};

} //end of namespace ModuleController


} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_COMMAND_ARM_H
