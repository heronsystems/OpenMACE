#ifndef MODULE_VEHICLE_MAVLINK_COMMAND_LAND_H
#define MODULE_VEHICLE_MAVLINK_COMMAND_LAND_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

#include "mavlink.h"

namespace MAVLINKUXVControllers {


class CommandLand : public Controller_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>
{
public:
    CommandLand(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialLand, MAV_CMD_NAV_LAND>(cb, queue, linkChan)
    {

    }

    virtual ~CommandLand() = default;

protected:

    virtual void FillCommand(const command_item::SpatialLand &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
//        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
//        {
//            cmd.param5 = commandItem.position->getX() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param6 = commandItem.position->getY() * pow(10,7); //this doesnt mean anything for ardupilot
//            cmd.param7 = commandItem.position->getZ() * 1000; //this doesnt mean anything for ardupilot
//        }
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialLand &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};


}

#endif // MODULE_VEHICLE_MAVLINK_COMMAND_LAND_H
