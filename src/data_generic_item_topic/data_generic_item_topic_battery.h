#ifndef DATA_GENERIC_ITEM_TOPIC_FUEL_H
#define DATA_GENERIC_ITEM_TOPIC_FUEL_H

#include "data_generic_item/data_generic_item_battery.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicBattery_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicBattery_structure;

class DataGenericItemTopic_Battery : public DataGenericItem::DataGenericItem_Battery, public JSONConverter, public Data::NamedTopicComponentDataObject<DataGenericItemTopicBattery_name, &DataGenericItemTopicBattery_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

    DataGenericItemTopic_Battery();
    DataGenericItemTopic_Battery(const DataGenericItem::DataGenericItem_Battery &copyObj);
};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_FUEL_H
