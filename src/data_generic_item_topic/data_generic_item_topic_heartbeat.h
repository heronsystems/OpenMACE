#ifndef DATA_GENERIC_ITEM_TOPIC_HEARTBEAT_H
#define DATA_GENERIC_ITEM_TOPIC_HEARTBEAT_H

#include "data_generic_item/data_generic_item_heartbeat.h"
#include "data/i_topic_component_data_object.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicHeartbeat_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicHeartbeat_structure;

class DataGenericItemTopic_Heartbeat : public DataGenericItem::DataGenericItem_Heartbeat, public Data::NamedTopicComponentDataObject<DataGenericItemTopicHeartbeat_name, &DataGenericItemTopicHeartbeat_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataGenericItemTopic_Heartbeat();
    DataGenericItemTopic_Heartbeat(const DataGenericItem::DataGenericItem_Heartbeat &copyObj);
};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_HEARTBEAT_H
