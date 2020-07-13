#ifndef DATA_GENERIC_ITEM_TOPIC_SYSTEM_ARM_H
#define DATA_GENERIC_ITEM_TOPIC_SYSTEM_ARM_H

#include "data_generic_item/data_generic_item_system_arm.h"
#include "data/i_topic_component_data_object.h"
#include "data/jsonconverter.h"

namespace DataGenericItemTopic {

extern const char DataGenericItemTopicSystemArm_name[];
extern const MaceCore::TopicComponentStructure DataGenericItemTopicSystemArm_structure;

class DataGenericItemTopic_SystemArm : public JSONConverter, public DataGenericItem::DataGenericItem_SystemArm, public Data::NamedTopicComponentDataObject<DataGenericItemTopicSystemArm_name, &DataGenericItemTopicSystemArm_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    virtual QJsonObject toJSON(const int &vehicleID, const std::string &dataType) const;

    DataGenericItemTopic_SystemArm();
    DataGenericItemTopic_SystemArm(const DataGenericItem::DataGenericItem_SystemArm &copyObj);
};

} //end of namespace DataGenericItemTopic


#endif // DATA_GENERIC_ITEM_TOPIC_SYSTEM_ARM_H
