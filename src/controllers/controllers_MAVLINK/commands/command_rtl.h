#ifndef MAVLINK_CONTROLLER_COMMAND_RTL_H
#define MAVLINK_CONTROLLER_COMMAND_RTL_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKUXVControllers {

namespace VehicleController {

class CommandRTL : public Controller_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>
{
public:
    //!
    //! \brief CommandRTL This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandRTL(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan, "CommandRTL")
    {

    }

    virtual ~CommandRTL() = default;

    virtual void FillCommand(const command_item::SpatialRTL &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialRTL &data) const
    {
        data.setTargetSystem(message.target_system);
    }

    virtual bool Finish_Receive(const mavlink_command_ack_t &msg, const MavlinkEntityKey &sender, uint8_t & ack, MavlinkEntityKey &queueObj)
    {
        if(msg.command != MAV_CMD_NAV_RETURN_TO_LAUNCH)
        {
            return false;
        }

        queueObj = sender;
        ack = msg.result;
        return true;
    }

};

} //end of namespace VehicleController

namespace ModuleController {

class CommandRTL : public ModuleController_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>
{
public:
    //!
    //! \brief CommandRTL This constructor is used to communicate to an explicit vehicle instance
    //! \param cb
    //! \param queue
    //! \param linkChan
    //!
    CommandRTL(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        ModuleController_GenericLongCommand<command_item::SpatialRTL, MAV_CMD_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan, "CommandRTL")
    {

    }

    virtual ~CommandRTL() = default;

    virtual void FillCommand(const command_item::SpatialRTL &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialRTL &data) const
    {
        data.setTargetSystem(message.target_system);
    }
};

} //end of namespace ModuleController

} //end of namespace MAVLINKUXVControllers

#endif // MAVLINK_CONTROLLER_COMMAND_RTL_H
