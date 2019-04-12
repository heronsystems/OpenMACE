#ifndef COMMAND_TOPIC_LAND_H
#define COMMAND_TOPIC_LAND_H


#include "data_generic_state_item/base_3d_position.h"
#include "data/i_topic_component_data_object.h"

#include "data_generic_state_item_topic/prototype_topic_global_position.h"

namespace DataCommandTopic {

extern const char DataCommandTopicLand_name[];
extern const MaceCore::TopicComponentStructure DataCommandTopicLand_structure;

class DataCommandTopic_Land : public Data::NamedTopicComponentDataObject<DataCommandTopicLand_name, &DataCommandTopicLand_structure>
{
private:

    bool m_PositionSet;
    DataStateTopic::PrototypeTopicGlobalPosition m_position;

public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataCommandTopic_Land();
    DataCommandTopic_Land(const DataState::StateGlobalPosition &copyObj);
};

} //end of namespace DataCommandTopic

#endif // COMMAND_TOPIC_LAND_H
