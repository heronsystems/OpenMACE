#ifndef COMMAND_TAKEOFF_H
#define COMMAND_TAKEOFF_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace Controllers {

template <typename MESSAGETYPE>
class CommandTakeoff : public Controller_GenericLongCommand<MESSAGETYPE, command_item::SpatialTakeoff, (uint8_t)command_item::COMMANDITEM::CI_NAV_TAKEOFF>
{
public:
    CommandTakeoff(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericLongCommand<MESSAGETYPE, command_item::SpatialTakeoff, (uint8_t)command_item::COMMANDITEM::CI_NAV_TAKEOFF>(cb, queue, linkChan)
    {

    }

protected:

    virtual void FillCommand(const command_item::SpatialTakeoff &commandItem, mace_command_long_t &cmd) const
    {
        if(commandItem.position->has2DPositionSet())
        {
            cmd.param1 = 1.0;
            cmd.param5 = commandItem.position->getX();
            cmd.param6 = commandItem.position->getY();
        }
        cmd.param7 = commandItem.position->getZ();
    }

    virtual void BuildCommand(const mace_command_long_t &message, command_item::SpatialTakeoff &data) const
    {
        if(message.param1 > 0.0)
        {
            data->position->setX(message.param5);
            data->position->setY(message.param6);
        }
        data->setTargetSystem(message.target_system);
        data->position->setZ(message.param7);
    }
};


}

#endif // COMMAND_TAKEOFF_H
