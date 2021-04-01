#ifndef COMMAND_RTL_H
#define COMMAND_RTL_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace Controllers {


template <typename MESSAGETYPE>
class CommandRTL : public Controller_GenericShortCommand<MESSAGETYPE, command_item::SpatialRTL, (uint8_t)command_item::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH>
{
public:
    CommandRTL(const IMessageNotifier<MESSAGETYPE> *cb, MessageModuleTransmissionQueue<MESSAGETYPE> *queue, int linkChan) :
        Controller_GenericShortCommand<MESSAGETYPE, command_item::SpatialRTL, (uint8_t)command_item::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH>(cb, queue, linkChan)
    {

    }

    protected:

    virtual void FillCommand(const command_item::SpatialRTL &commandItem, mace_command_short_t &cmd) const
    {
        UNUSED(commandItem);
        UNUSED(cmd);
    }

    virtual void BuildCommand(const mace_command_short_t &message, command_item::SpatialRTL &data) const
    {
        UNUSED(message);
        UNUSED(data);
    }
};


}

#endif // COMMAND_RTL_H
