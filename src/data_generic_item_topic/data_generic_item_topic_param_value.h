#ifndef DATAGENERICITEMTOPIC_PARAMVALUE_H
#define DATAGENERICITEMTOPIC_PARAMVALUE_H


class DataGenericItemTopic_ParamValue
{
public:
    DataGenericItemTopic_ParamValue();
};

#include "data_generic_item/data_generic_item_param_value.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicParamValue_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicParamValue_structure;

class DataGenericItemTopic_ParamValue : public DataGenericItem::DataGenericItem_ParamValue, public JSONConverter, public Data::NamedTopicComponentDataObject<DataGenericItemTopicParamValue_name, &DataGenericItemTopicParamValue_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    DataGenericItemTopic_ParamValue();
    DataGenericItemTopic_ParamValue(const DataGenericItem::DataGenericItem_ParamValue &copyObj);
};

} //end of namespace DataGenericItemTopic


#endif // DATAGENERICITEMTOPIC_PARAMVALUE_H
