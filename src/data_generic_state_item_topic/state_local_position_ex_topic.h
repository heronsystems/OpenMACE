#ifndef STATE_LOCAL_POSITION_EX_TOPIC_H
#define STATE_LOCAL_POSITION_EX_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_local_position_ex.h"

namespace DataStateTopic {

extern const char LocalPositionTopicEx_name[];
extern const MaceCore::TopicComponentStructure LocalPositionTopicEx_structure;

class StateLocalPositionExTopic : public DataState::StateLocalPositionEx, public Data::NamedTopicComponentDataObject<LocalPositionTopicEx_name, &LocalPositionTopicEx_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateLocalPositionExTopic();
    StateLocalPositionExTopic(const DataState::StateLocalPositionEx &copyObj);

};

} //end of namespace DataStateTopic

#endif // STATE_LOCAL_POSITION_EX_TOPIC_H
