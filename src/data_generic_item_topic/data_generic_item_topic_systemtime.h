#ifndef DATA_GENERIC_ITEM_TOPIC_SYSTEMTIME_H
#define DATA_GENERIC_ITEM_TOPIC_SYSTEMTIME_H


#include "data_generic_item/data_generic_item_systemtime.h"
#include "data/i_topic_component_data_object.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicSystemTime_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicSystemTime_structure;

class DataGenericItemTopic_SystemTime : public DataGenericItem::DataGenericItem_SystemTime, public Data::NamedTopicComponentDataObject<DataGenericItemTopicSystemTime_name, &DataGenericItemTopicSystemTime_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataGenericItemTopic_SystemTime();
    DataGenericItemTopic_SystemTime(const DataGenericItem::DataGenericItem_SystemTime &copyObj);
};


} //end of namespace DataGenericItemTopic


#endif // DATA_GENERIC_ITEM_TOPIC_SYSTEMTIME_H
