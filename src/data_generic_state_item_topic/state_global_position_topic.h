#ifndef STATE_GLOBAL_POSITION_TOPIC_H
#define STATE_GLOBAL_POSITION_TOPIC_H

#include "data/i_topic_component_data_object.h"

#include "data/coordinate_frame.h"
#include "data_generic_state_item/state_global_position.h"

namespace DataStateTopic {

extern const char GlobalPositionTopic_name[];
extern const MaceCore::TopicComponentStructure GlobalPositionTopic_structure;

class StateGlobalPositionTopic : public DataState::StateGlobalPosition, public Data::NamedTopicComponentDataObject<GlobalPositionTopic_name, &GlobalPositionTopic_structure>
{
public:
    virtual MaceCore::TopicDatagram GenerateDatagram() const;
    virtual void CreateFromDatagram(const MaceCore::TopicDatagram &datagram);

    StateGlobalPositionTopic();
    StateGlobalPositionTopic(const DataStateTopic::StateGlobalPositionTopic &copyObj);
    StateGlobalPositionTopic(const DataState::StateGlobalPosition &posObj);

};

} //end of namespace DataStateTopic

#endif // STATE_GLOBAL_POSITION_TOPIC_H
