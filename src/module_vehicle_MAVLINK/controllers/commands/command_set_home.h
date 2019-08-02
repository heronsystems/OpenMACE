#ifndef COMMAND_SET_HOME_H
#define COMMAND_SET_HOME_H

#include "generic_long_command.h"
#include "generic_int_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace MAVLINKVehicleControllers{


class Command_SetHomeLong : public Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>
{
public:
    Command_SetHomeLong(const Controllers::IMessageNotifier<mavlink_message_t, int> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>(cb, queue, linkChan)
    {

    }

    virtual ~Command_SetHomeLong() = default;

protected:

    virtual void FillCommand(const command_item::SpatialHome &commandItem, mavlink_command_long_t &cmd) const
    {
        cmd.target_system = commandItem.getTargetSystem();
        cmd.param5 = commandItem.position->getX();
        cmd.param6 = commandItem.position->getY();
        cmd.param7 = commandItem.position->getZ();
    }

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialHome &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

class Command_SetHomeInt : public Controller_GenericIntCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>
{
public:
    Command_SetHomeInt(const Controllers::IMessageNotifier<mavlink_message_t, MavlinkEntityKey> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericIntCommand<command_item::SpatialHome, MAV_CMD_DO_SET_HOME>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const command_item::SpatialHome &commandItem, mavlink_command_int_t &cmd) const
    {
        std::cout<<"Ken: Be careful of the coordinate frame handlings here. Fix in future."<<std::endl;
        cmd.target_system = commandItem.getTargetSystem();
        if(commandItem.getPosition().getCoordinateFrame() == Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT)
            cmd.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; //KEN: For now, but should majorly fix
        else
            cmd.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        cmd.x = commandItem.position->getX() * pow(10,7);
        cmd.y = commandItem.position->getY() * pow(10,7);
        cmd.z = commandItem.position->getZ();
    }

    virtual void BuildCommand(const mavlink_command_int_t &message, command_item::SpatialHome &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};

} //end of namespace MAVLINKVehicleControllers


#endif // COMMAND_SET_HOME_H
