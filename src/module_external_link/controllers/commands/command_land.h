#ifndef COMMAND_LAND_H
#define COMMAND_LAND_H

#include "generic_long_command.h"

#include "data_generic_command_item_topic/command_item_topic_components.h"

namespace ExternalLink {


class CommandLand : public Controller_GenericLongCommand<command_item::SpatialLand, MAV_CMD::MAV_CMD_NAV_LAND>
{
public:

    CommandLand(const Controllers::IMessageNotifier<mavlink_message_t, MaceCore::ModuleCharacteristic> *cb, TransmitQueue *queue, int linkChan);

protected:

    virtual void FillCommand(const command_item::SpatialLand &commandItem, mavlink_command_long_t &cmd) const;

    virtual void BuildCommand(const mavlink_command_long_t &message, command_item::SpatialLand &data) const;
};


}

#endif // COMMAND_LAND_H
