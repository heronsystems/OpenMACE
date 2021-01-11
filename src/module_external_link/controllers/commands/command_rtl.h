#ifndef COMMAND_RTL_H
#define COMMAND_RTL_H

#include "generic_short_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandRTL : public Controller_GenericShortCommand<command_item::SpatialRTL, MAV_CMD::MAV_CMD_NAV_RETURN_TO_LAUNCH>
{
public:

    CommandRTL(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const command_item::SpatialRTL &commandItem, mavlink_command_short_t &cmd) const;

    virtual void BuildCommand(const mavlink_command_short_t &message, SpatialRTL &data) const;
};


}

#endif // COMMAND_RTL_H
