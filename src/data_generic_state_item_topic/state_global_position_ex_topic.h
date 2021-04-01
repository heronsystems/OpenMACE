#ifndef STATE_GLOBAL_POSITION_EX_TOPIC_H
#define STATE_GLOBAL_POSITION_EX_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_global_position_ex.h"

namespace DataStateTopic {

extern const char GlobalPositionTopicEx_name[];
extern const MaceCore::TopicComponentStructure GlobalPositionTopicEx_structure;

class StateGlobalPositionExTopic : public DataState::StateGlobalPositionEx, public Data::NamedTopicComponentDataObject<GlobalPositionTopicEx_name, &GlobalPositionTopicEx_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateGlobalPositionExTopic();
    StateGlobalPositionExTopic(const DataState::StateGlobalPositionEx &copyObj);

};

} //end of namespace DataStateTopic

#endif // STATE_GLOBAL_POSITION_EX_TOPIC_H
