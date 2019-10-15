#ifndef COMMAND_ITEM_TOPIC_ACK_H
#define COMMAND_ITEM_TOPIC_ACK_H

#include "data/i_topic_component_data_object.h"

#include "data_generic_command_item/command_item_ack.h"

using namespace command_item;

namespace CommandTopic {

extern const char DataCommandItemTopicACK_name[];
extern const MaceCore::TopicComponentStructure DataCommandItemTopicACK_structure;

class CommandItemTopic_ACK : public command_item::CommandItemACK, public Data::NamedTopicComponentDataObject<DataCommandItemTopicACK_name, &DataCommandItemTopicACK_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    CommandItemTopic_ACK();
    CommandItemTopic_ACK(const command_item::CommandItemACK &copyObj);
};

} //end of namespace CommandTopic

#endif // COMMAND_ITEM_TOPIC_ACK_H
