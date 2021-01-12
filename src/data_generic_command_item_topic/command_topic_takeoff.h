#ifndef COMMAND_TOPIC_TAKEOFF_H
#define COMMAND_TOPIC_TAKEOFF_H

#include "data_generic_state_item/base_3d_position.h"
#include "data/i_topic_component_data_object.h"

#include "data_generic_state_item_topic/prototype_topic_global_position.h"

namespace DataCommandTopic {

extern const char DataCommandTopicTakeoff_name[];
extern const MaceCore::TopicComponentStructure DataCommandTopicTakeoff_structure;

class DataCommandTopic_Takeoff : public DataStateTopic::PrototypeTopicGlobalPosition, public Data::NamedTopicComponentDataObject<DataCommandTopicTakeoff_name, &DataCommandTopicTakeoff_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;

    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    DataCommandTopic_Takeoff();
    DataCommandTopic_Takeoff(const DataState::Base3DPosition &copyObj);
};

} //end of namespace DataCommandTopic

#endif // COMMAND_TOPIC_TAKEOFF_H
