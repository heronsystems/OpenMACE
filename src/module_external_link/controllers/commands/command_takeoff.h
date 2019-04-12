#ifndef COMMAND_TAKEOFF_H
#define COMMAND_TAKEOFF_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {

class CommandTakeoff : public Controller_GenericLongCommand<CommandItem::SpatialTakeoff, (uint8_t)CommandItem::COMMANDITEM::CI_NAV_TAKEOFF>
{
public:

    CommandTakeoff(const Controllers::IMessageNotifier<mace_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const CommandItem::SpatialTakeoff &commandItem, mace_command_long_t &cmd) const;

    virtual void BuildCommand(const mace_command_long_t &message, CommandItem::SpatialTakeoff &data) const;

};


}

#endif // COMMAND_TAKEOFF_H
