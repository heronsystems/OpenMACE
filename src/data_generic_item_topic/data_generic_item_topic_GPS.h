#ifndef DATA_GENERIC_ITEM_TOPIC_GPS_H
#define DATA_GENERIC_ITEM_TOPIC_GPS_H

#include "data_generic_item/data_generic_item_GPS.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicGPS_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicGPS_structure;

class DataGenericItemTopic_GPS : public JSONConverter, public DataGenericItem::DataGenericItem_GPS, public Data::NamedTopicComponentDataObject<DataGenericItemTopicGPS_name, &DataGenericItemTopicGPS_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    virtual void fromJSON(const QJsonDocument &inputJSON) ;

    virtual std::string toCSV(const std::string &delimiter) const;

    DataGenericItemTopic_GPS();
    DataGenericItemTopic_GPS(const DataGenericItem::DataGenericItem_GPS &copyObj);

};

} //end of namespace DataGenericItemTopic

#endif // DATA_GENERIC_ITEM_TOPIC_GPS_H
