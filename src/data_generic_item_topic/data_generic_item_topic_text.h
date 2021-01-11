#ifndef DATA_GENERIC_ITEM_TOPIC_TEXT_H
#define DATA_GENERIC_ITEM_TOPIC_TEXT_H

#include "data_generic_item/data_generic_item_text.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicText_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicText_structure;

class DataGenericItemTopic_Text : public JSONConverter, public DataGenericItem::DataGenericItem_Text, public Data::NamedTopicComponentDataObject<DataGenericItemTopicText_name, &DataGenericItemTopicText_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

    DataGenericItemTopic_Text();
    DataGenericItemTopic_Text(const DataGenericItem::DataGenericItem_Text &copyObj);
};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_TEXT_H
