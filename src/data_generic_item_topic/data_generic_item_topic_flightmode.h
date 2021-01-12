#ifndef DATA_GENERIC_ITEM_TOPIC_FLIGHTMODE_H
#define DATA_GENERIC_ITEM_TOPIC_FLIGHTMODE_H

#include "data_generic_item/data_generic_item_flightmode.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicFlightMode_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicFlightMode_structure;

class DataGenericItemTopic_FlightMode : public JSONConverter, public DataGenericItem::DataGenericItem_FlightMode, public Data::NamedTopicComponentDataObject<DataGenericItemTopicFlightMode_name, &DataGenericItemTopicFlightMode_structure>
{
public:

    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);
    
    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

    DataGenericItemTopic_FlightMode();
    DataGenericItemTopic_FlightMode(const DataGenericItem::DataGenericItem_FlightMode &copyObj);
};

} //end of namespace DataGenericItemTopic
#endif // DATA_GENERIC_ITEM_TOPIC_FLIGHTMODE_H
