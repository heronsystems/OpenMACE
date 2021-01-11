#ifndef COMMAND_ITEM_TOPIC_H
#define COMMAND_ITEM_TOPIC_H

#include "data/i_topic_component_data_object.h"
#include "data_generic_command_item/command_item_components.h"

using namespace command_item;

namespace CommandTopic{

extern const char CommandItemTopic_name[];
extern const MaceCore::TopicComponentStructure CommandItemTopic_structure;

class CommandItemTopic :public Data::NamedTopicComponentDataObject<CommandItemTopic_name, &CommandItemTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

public:
    CommandItemTopic(const std::shared_ptr<command_item::AbstractCommandItem> &command)
    {
        this->setCommandItem(command);
    }

    void setCommandItem(const std::shared_ptr<command_item::AbstractCommandItem> &command)
    {
        this->commandItem = command;
    }

    std::shared_ptr<command_item::AbstractCommandItem> getCommandItem() const
    {
        return commandItem;
    }

    /*
    int getOriginatingSystem() const
    {
        return commandItem->getOriginatingSystem();
    }

    int getTargetSystem() const
    {
        return commandItem->getTargetSystem();
    }
    */

    MAV_CMD getCommandType() const
    {
        return commandItem->getCommandType();
    }

private:
    std::shared_ptr<command_item::AbstractCommandItem> commandItem;
};

} //end of namespace CommandTopic


#endif // COMMAND_ITEM_TOPIC_H
