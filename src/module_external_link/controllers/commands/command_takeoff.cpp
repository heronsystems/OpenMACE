#include "command_takeoff.h"

namespace ExternalLink {

    CommandTakeoff::CommandTakeoff(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan) :
        Controller_GenericLongCommand<command_item::SpatialTakeoff, (uint8_t)command_item::COMMANDTYPE::CI_NAV_TAKEOFF>(cb, queue, linkChan)
    {

    }

    void CommandTakeoff::FillCommand(const command_item::SpatialTakeoff &commandItem, mace_command_long_t &cmd) const
    {
        if(commandItem.position->has2DPositionSet())
        {
            cmd.param1 = 1.0;
            cmd.param5 = commandItem.position->getX();
            cmd.param6 = commandItem.position->getY();
        }
        cmd.param7 = commandItem.position->getZ();
    }

    void CommandTakeoff::BuildCommand(const mace_command_long_t &message, SpatialTakeoff &data) const
    {
        if(message.param1 > 0.0)
        {
            data.position->setX(message.param5);
            data.position->setY(message.param6);
        }
        data.position->setZ(message.param7);
    }


}
