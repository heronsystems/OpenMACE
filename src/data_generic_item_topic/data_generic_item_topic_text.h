#ifndef DATA_GENERIC_ITEM_TOPIC_TEXT_H
#define DATA_GENERIC_ITEM_TOPIC_TEXT_H

#include "data_generic_item/data_generic_item_text.h"
#include "data/i_topic_component_data_object.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicText_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicText_structure;

class DataGenericItemTopic_Text : public DataGenericItem::DataGenericItem_Text, public Data::NamedTopicComponentDataObject<DataGenericItemTopicText_name, &DataGenericItemTopicText_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataGenericItemTopic_Text();
    DataGenericItemTopic_Text(const DataGenericItem::DataGenericItem_Text &copyObj);
};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_TEXT_H
