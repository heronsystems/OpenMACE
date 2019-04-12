#ifndef COMMAND_LAND_H
#define COMMAND_LAND_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace Controllers {


template <typename MESSAGETYPE>
class CommandLand : public Controller_GenericLongCommand<MESSAGETYPE, CommandItem::SpatialLand, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND>
{
public:
    CommandLand(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericLongCommand<MESSAGETYPE, CommandItem::SpatialLand, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_LAND>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const CommandItem::SpatialLand &commandItem, mace_command_long_t &cmd) const
    {
        if(commandItem.position->isCoordinateFrame(Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT))
        {
            cmd.param5 = commandItem.position->getX() * pow(10,7);
            cmd.param6 = commandItem.position->getY() * pow(10,7);
            cmd.param7 = commandItem.position->getZ() * 1000;
        }
    }

    virtual void BuildCommand(const mace_command_long_t &message, CommandItem::SpatialLand &data) const
    {
        data->setTargetSystem(message.target_system);
        data->position->setX(message.param5);
        data->position->setY(message.param6);
        data->position->setZ(message.param7);
    }
};


}

#endif // COMMAND_LAND_H
